// AntMiner: ant-colony reference miner.
// Usage: AntMiner [Node IP] [Node Port] [MiningID] [Signing Seed] [Threads] --task FILE

#include <chrono>
#include <thread>
#include <mutex>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <atomic>
#include <memory>

#ifndef _MSC_VER
#include <signal.h>
#endif

#include "score_bpp9000.h"
#include "keyUtils.h"
#include "network.h"
#include "ant_colony_message.h"

// Actual miner software will modify the strategy accordingly
// Demo is greedy exploration rate: 1 in this many mining rounds extends a random resolved
// node or the root instead of the best, so the search does not get stuck in one basin.
static constexpr unsigned int ANT_EXPLORE_ONE_IN = 8U;

// The reference bpp9000 scorer with the deployed production parameters
using AntMinerT = score_bpp9000::Miner<
    score_bpp9000::NUMBER_OF_INPUT_NEURONS,
    score_bpp9000::NUMBER_OF_OUTPUT_NEURONS,
    score_bpp9000::SEQUENCE_LENGTH,
    score_bpp9000::WINDOW_WIDTH,
    score_bpp9000::MAX_NUMBER_OF_TICKS,
    score_bpp9000::NUMBER_OF_NEIGHBORS,
    score_bpp9000::POPULATION_THRESHOLD,
    score_bpp9000::NUMBER_OF_MUTATIONS,
    score_bpp9000::SOLUTION_THRESHOLD>;

static std::atomic<char> state(0);

#ifdef _MSC_VER
static BOOL WINAPI ctrlCHandlerRoutine(DWORD dwCtrlType)
{
    if (!state)
    {
        state = 1;
    }
    else
    {
        std::exit(1);
    }
    return TRUE;
}
#else
static void ctrlCHandlerRoutine(int signum)
{
    if (!state)
    {
        state = 1;
    }
    else
    {
        std::exit(1);
    }
}
#endif

static void consoleCtrlHandler()
{
#ifdef _MSC_VER
    SetConsoleCtrlHandler(ctrlCHandlerRoutine, TRUE);
#else
    signal(SIGINT, ctrlCHandlerRoutine);
#endif
}

static char* nodeIp = NULL;
static int nodePort = 0;

static int waitForResponse(ServerSocket& sock, unsigned char wantedType, char* payload, unsigned int payloadCapacity)
{
    static char scratch[1024 * 1024];
    for (int attempt = 0; attempt < 64; attempt++)
    {
        RequestResponseHeader header;
        if (!sock.receiveData((char*)&header, sizeof(header)))
        {
            return -1;
        }
        unsigned int remaining = header.size() - sizeof(header);
        if (header.type() == wantedType)
        {
            if (remaining > payloadCapacity)
            {
                return -1;
            }
            if (remaining > 0 && !sock.receiveData(payload, remaining))
            {
                return -1;
            }
            return (int)remaining;
        }
        // Drain and discard.
        while (remaining > 0)
        {
            unsigned int chunk = remaining < sizeof(scratch) ? remaining : (unsigned int)sizeof(scratch);
            if (!sock.receiveData(scratch, chunk))
            {
                return -1;
            }
            remaining -= chunk;
        }
    }
    return -1;
}

static bool sendRequest(ServerSocket& sock, unsigned char type, const void* payload, unsigned int payloadSize)
{
    struct
    {
        RequestResponseHeader header;
        char payload[128];
    } packet;
    packet.header.setSize(sizeof(RequestResponseHeader) + payloadSize);
    packet.header.randomizeDejavu();
    packet.header.setType(type);
    if (payloadSize > 0)
    {
        memcpy(packet.payload, payload, payloadSize);
    }
    return sock.sendData((char*)&packet, sizeof(RequestResponseHeader) + payloadSize);
}

static bool queryCurrentTickInfo(ServerSocket& sock, RespondCurrentTickInfo& out)
{
    if (!sendRequest(sock, REQUEST_CURRENT_TICK_INFO, NULL, 0))
    {
        return false;
    }
    return waitForResponse(sock, RESPOND_CURRENT_TICK_INFO, (char*)&out, sizeof(out)) == (int)sizeof(out);
}

static bool queryEpochContext(ServerSocket& sock, RespondAntEpochContext& out)
{
    if (!sendRequest(sock, REQUEST_ANT_EPOCH_CONTEXT, NULL, 0))
    {
        return false;
    }
    return waitForResponse(sock, RESPOND_ANT_EPOCH_CONTEXT, (char*)&out, sizeof(out)) == (int)sizeof(out);
}

// Derive the anchor digest for anchorTick as the node does: fetch the tick's stored TickData
// (REQUEST_TICK_DATA) and hash K12(anchorTick || K12(TickData))
// Returns 1 with anchorDigest filled, 0 when the node holds no TickData, -1 on network failure.
static int fetchAnchorDigest(ServerSocket& sock, unsigned int anchorTick, unsigned char anchorDigest[32])
{
    RequestedTickData request;
    request.tick = anchorTick;
    if (!sendRequest(sock, REQUEST_TICK_DATA, &request, sizeof(request)))
    {
        return -1;
    }

    static unsigned char tickData[TICK_DATA_SIZE];

    for (int attempt = 0; attempt < 64; attempt++)
    {
        RequestResponseHeader header;
        if (!sock.receiveData((char*)&header, sizeof(header)))
        {
            return -1;
        }
        unsigned int remaining = header.size() - sizeof(header);
        if (header.type() == END_RESPONSE && remaining == 0)
        {
            return 0;
        }
        if (header.type() == BROADCAST_FUTURE_TICK_DATA && remaining == TICK_DATA_SIZE)
        {
            if (!sock.receiveData((char*)tickData, TICK_DATA_SIZE))
            {
                return -1;
            }
            // Exactly the buffer the node feeds KangarooTwelve for etalonTick.transactionDigest.
            unsigned char transactionDigest[32];
            KangarooTwelve(tickData, TICK_DATA_SIZE, transactionDigest, 32);

            unsigned char input[36];
            memcpy(input, &anchorTick, 4);
            memcpy(input + 4, transactionDigest, 32);
            KangarooTwelve(input, 36, anchorDigest, 32);
            return 1;
        }
        // A TickData of unexpected size means this node runs a core build whose TickData layout
        // differs from TICK_DATA_SIZE
        if (header.type() == BROADCAST_FUTURE_TICK_DATA)
        {
            printf("TickData size mismatch: node sent %u bytes, expected %u - check core constants\n",
                remaining, TICK_DATA_SIZE);
        }
        // Drain and discard other traffic.
        char scratch[4096];
        while (remaining > 0)
        {
            unsigned int chunk = remaining < sizeof(scratch) ? remaining : (unsigned int)sizeof(scratch);
            if (!sock.receiveData(scratch, chunk))
            {
                return -1;
            }
            remaining -= chunk;
        }
    }
    return -1;
}

// Ticks the node holds no data for carry no anchor, so step back from fromTick until one does
#define MAX_ANCHOR_WALK_BACK 16U

// Returns 1 with anchorTick/anchorDigest filled, 0 when nothing anchored was found above minTick
// within the walk-back bound (retry later), -1 on network failure.
static int resolveAnchorDigest(ServerSocket& sock, unsigned int fromTick, unsigned int minTick,
    unsigned int& anchorTick, unsigned char anchorDigest[32])
{
    // When the anchor tick can not be fetched or an empty tick. Look back more in the past
    for (unsigned int tick = fromTick; tick > minTick && (fromTick - tick) < MAX_ANCHOR_WALK_BACK; tick--)
    {
        const int fetched = fetchAnchorDigest(sock, tick, anchorDigest);
        if (fetched < 0)
        {
            return -1;
        }
        if (fetched == 1)
        {
            anchorTick = tick;
            return 1;
        }
    }
    return 0;
}

// One page of the identity-tree listing starting at fromIndex (Operator signed message)
// Returns false on network failure; nextIndex is 0 when there are no more records
static bool queryIdentityTree(ServerSocket& sock, const unsigned char* signingSubseed, const unsigned char* signingPublicKey,
    const unsigned char* pubkey, unsigned int fromIndex, std::vector<AntIdentityTreeNode>& outEntries, unsigned int& nextIndex)
{
    struct
    {
        RequestResponseHeader header;
        RequestAntIdentityTree request;
        unsigned char signature[64];
    } packet;
    packet.header.setSize(sizeof(packet));
    packet.header.randomizeDejavu();
    packet.header.setType(REQUEST_ANT_IDENTITY_TREE);
    memcpy(packet.request.pubkey, pubkey, 32);
    packet.request.fromIndex = fromIndex;
    packet.request.padding = 0;
    unsigned char digest[32];
    KangarooTwelve((const unsigned char*)&packet.request, sizeof(RequestAntIdentityTree), digest, 32);
    // FourQ encode() writes the signature with an aligned 32-byte store; sign into a 32-byte-aligned
    // buffer, then copy into the (possibly unaligned) packet field.
    alignas(32) unsigned char sig[64];
    sign(signingSubseed, signingPublicKey, digest, sig);
    memcpy(packet.signature, sig, 64);
    if (!sock.sendData((char*)&packet, sizeof(packet)))
    {
        return false;
    }
    char buffer[sizeof(RespondAntIdentityTreeHeader) + 64 * sizeof(AntIdentityTreeNode)];
    const int received = waitForResponse(sock, RESPOND_ANT_IDENTITY_TREE, buffer, sizeof(buffer));
    if (received < (int)sizeof(RespondAntIdentityTreeHeader))
    {
        return false;
    }
    const RespondAntIdentityTreeHeader* header = (const RespondAntIdentityTreeHeader*)buffer;
    if (header->itemSize != sizeof(AntIdentityTreeNode))
    {
        printf("Identity-tree item size mismatch (node %u, miner %u) - wire structs out of sync!\n",
            header->itemSize, (unsigned int)sizeof(AntIdentityTreeNode));
        return false;
    }
    const AntIdentityTreeNode* entries = (const AntIdentityTreeNode*)(buffer + sizeof(RespondAntIdentityTreeHeader));
    for (unsigned int i = 0; i < header->count; i++)
    {
        outEntries.push_back(entries[i]);
    }
    nextIndex = header->nextIndex;
    return true;
}

// Page the whole identity-tree listing for one pubkey
static bool fetchIdentityTree(ServerSocket& sock, const unsigned char* signingSubseed, const unsigned char* signingPublicKey,
    const unsigned char* pubkey, std::vector<AntIdentityTreeNode>& out)
{
    std::vector<AntIdentityTreeNode> entries;
    unsigned int fromIndex = 0;
    do
    {
        unsigned int nextIndex = 0;
        if (!queryIdentityTree(sock, signingSubseed, signingPublicKey, pubkey, fromIndex, entries, nextIndex))
        {
            return false;
        }
        fromIndex = nextIndex;
    } while (fromIndex != 0);
    out.swap(entries);
    return true;
}

// The node's canonical ANN wire form: LUT rows only, row k = neuron updatedNeuronIndices[k]
// (dense by updated-neuron position). This miner's ANN struct carries neuron states and keeps LUT
// rows at absolute neuron indices, so wire bytes and local bytes are never comparable directly.
static constexpr unsigned long long CANONICAL_ANN_BYTES = AntMinerT::maxNumberOfNeurons * AntMinerT::lutSize;

// Fetch one stored node's canonical ANN LUT back from the node (Operator signed message).
// Returns 1 with outCanonicalLut filled, 0 when the node answered without a usable ANN, -1 on network failure
static int queryParentAnn(ServerSocket& sock, const unsigned char* operatorSubseed, const unsigned char* operatorPublicKey,
    unsigned int refTick, unsigned int refSolutionIndexInTick, unsigned char* outCanonicalLut)
{
    struct
    {
        RequestResponseHeader header;
        RequestAntParentAnn request;
        unsigned char signature[64];
    } packet;
    packet.header.setSize(sizeof(packet));
    packet.header.randomizeDejavu();
    packet.header.setType(REQUEST_ANT_PARENT_ANN);
    packet.request.parentRefTick = refTick;
    packet.request.parentRefSolutionIndexInTick = refSolutionIndexInTick;
    unsigned char digest[32];
    KangarooTwelve((const unsigned char*)&packet.request, sizeof(RequestAntParentAnn), digest, 32);
    alignas(32) unsigned char sig[64];
    sign(operatorSubseed, operatorPublicKey, digest, sig);
    memcpy(packet.signature, sig, 64);
    if (!sock.sendData((char*)&packet, sizeof(packet)))
    {
        return -1;
    }
    char buffer[sizeof(RespondAntParentAnnHeader) + CANONICAL_ANN_BYTES];
    const int received = waitForResponse(sock, RESPOND_ANT_PARENT_ANN, buffer, sizeof(buffer));
    if (received < (int)sizeof(RespondAntParentAnnHeader))
    {
        return -1;
    }
    const RespondAntParentAnnHeader* respHeader = (const RespondAntParentAnnHeader*)buffer;
    if (respHeader->status != ANT_PARENT_ANN_STATUS_OK
        || respHeader->annSizeBytes != CANONICAL_ANN_BYTES
        || received < (int)(sizeof(RespondAntParentAnnHeader) + CANONICAL_ANN_BYTES))
    {
        return 0;
    }
    memcpy(outCanonicalLut, buffer + sizeof(RespondAntParentAnnHeader), CANONICAL_ANN_BYTES);
    return 1;
}

// Compare this miner's neuron-indexed LUT rows against the node's canonical bytes through the
// updated-neuron mapping. Only the live rows are compared; the canonical tail is padding.
static bool annMatchesCanonicalLut(const AntMinerT& m, const AntMinerT::ANN& local, const unsigned char* canonicalLut)
{
    for (unsigned long long k = 0; k < m.numberOfUpdatedNeurons; k++)
    {
        const unsigned long long n = m.updatedNeuronIndices[k];
        if (memcmp(&local.lut[n * AntMinerT::lutSize], &canonicalLut[k * AntMinerT::lutSize], AntMinerT::lutSize) != 0)
        {
            return false;
        }
    }
    return true;
}

// Rebuild a local ANN from the node's canonical LUT
static void annFromCanonicalLut(const AntMinerT& m, const unsigned char* canonicalLut,
    const AntMinerT::ANN& rootAnn, AntMinerT::ANN& out)
{
    memcpy(&out, &rootAnn, sizeof(AntMinerT::ANN));
    for (unsigned long long k = 0; k < m.numberOfUpdatedNeurons; k++)
    {
        const unsigned long long n = m.updatedNeuronIndices[k];
        memcpy(&out.lut[n * AntMinerT::lutSize], &canonicalLut[k * AntMinerT::lutSize], AntMinerT::lutSize);
    }
}

// One node of this miner's own tree (isolated per-identity trees)
struct OwnNode
{
    unsigned char nonce[32];
    unsigned int score;
    unsigned int anchorTick;                // tick number the mining was anchored to
    unsigned int depth;                     // 1 for root children
    unsigned int parentTick;          // this node's parentRef
    unsigned int parentSolutionIndexInTick;
    bool refKnown;                          // selfRef learned from the node's identity-tree listing
    bool lutChecked;                        // local ANN verified once against the node's stored ANN
    bool lutMismatch;                       // stored ANN differed; never extend this node
    unsigned int resolveAttempts;           // resolve cycles seen while still unresolved (mismatch detector)
    unsigned int selfTick;
    unsigned int selfSolutionIndexInTick;
    AntMinerT::ANN ann;                     // this node's evolved ANN (bestANN at mining time)
};

// Children a parent already holds, from the node's listing plus anything submitted since the last
// resolve cycle. The listing is what the node's own cap check uses and it survives a restart; only
// unresolved own nodes are added, a resolved one is already counted in the listing.
static unsigned int childCountOf(const std::vector<AntIdentityTreeNode>& listing,
    const std::vector<OwnNode>& ownNodes, unsigned int parentTick, unsigned int parentSolutionIndexInTick)
{
    const bool isRoot = (parentTick == ROOT_TICK && parentSolutionIndexInTick == ROOT_INDEX_IN_TICK);
    unsigned int count = 0;
    for (const AntIdentityTreeNode& entry : listing)
    {
        if (isRoot)
        {
            // ROOT has no entry of its own, so count the depth-1 records instead.
            count += (entry.depth == 1U) ? 1U : 0U;
        }
        else if (entry.selfTick == parentTick && entry.selfSolutionIndexInTick == parentSolutionIndexInTick)
        {
            count = entry.childCount;
            break;
        }
    }
    for (const OwnNode& node : ownNodes)
    {
        if (!node.refKnown
            && node.parentTick == parentTick
            && node.parentSolutionIndexInTick == parentSolutionIndexInTick)
        {
            count++;
        }
    }
    return count;
}

// Submit one ant solution as a BroadcastMessage whose decrypted gammingKey[0] selects
// MESSAGE_TYPE_ANT_SOLUTION
static bool submitAntSolution(ServerSocket& sock,
    const unsigned char* signingSubseed, const unsigned char* signingPrivateKey, const unsigned char* signingPublicKey,
    const unsigned char* computorPublicKey,
    unsigned int parentTick, unsigned int parentSolutionIndexInTick,
    unsigned int anchorTick, unsigned int claimedScore, const unsigned char* nonce)
{
    struct
    {
        RequestResponseHeader header;
        Message message;
        unsigned char payload[sizeof(AntSolutionBroadcastPayload)];
        unsigned char signature[64];
    } packet;

    packet.header.setSize(sizeof(packet));
    packet.header.zeroDejavu();
    packet.header.setType(BROADCAST_MESSAGE);

    memcpy(packet.message.sourcePublicKey, signingPublicKey, 32);
    memcpy(packet.message.destinationPublicKey, computorPublicKey, 32);

    unsigned char sharedKeyAndGammingNonce[64];
    memset(sharedKeyAndGammingNonce, 0, 32);
    if (memcmp(computorPublicKey, signingPublicKey, 32) == 0)
    {
        getSharedKey(signingPrivateKey, computorPublicKey, sharedKeyAndGammingNonce);
    }
    unsigned char gammingKey[32];
    do
    {
        _rdrand64_step((unsigned long long*)&packet.message.gammingNonce[0]);
        _rdrand64_step((unsigned long long*)&packet.message.gammingNonce[8]);
        _rdrand64_step((unsigned long long*)&packet.message.gammingNonce[16]);
        _rdrand64_step((unsigned long long*)&packet.message.gammingNonce[24]);
        memcpy(&sharedKeyAndGammingNonce[32], packet.message.gammingNonce, 32);
        KangarooTwelve(sharedKeyAndGammingNonce, 64, gammingKey, 32);
    } while (gammingKey[0] != MESSAGE_TYPE_ANT_SOLUTION);

    AntSolutionBroadcastPayload plain;
    plain.parentTick = parentTick;
    plain.parentSolutionIndexInTick = parentSolutionIndexInTick;
    plain.anchorTick = anchorTick;
    plain.claimedScore = claimedScore;
    memcpy(plain.nonce, nonce, 32);

    unsigned char gamma[sizeof(plain)];
    KangarooTwelve(gammingKey, 32, gamma, sizeof(gamma));
    const unsigned char* plainBytes = (const unsigned char*)&plain;
    for (unsigned int i = 0; i < sizeof(plain); i++)
    {
        packet.payload[i] = plainBytes[i] ^ gamma[i];
    }

    unsigned char digest[32];
    KangarooTwelve(
        (unsigned char*)&packet + sizeof(RequestResponseHeader),
        sizeof(packet) - sizeof(RequestResponseHeader) - 64,
        digest,
        32);
    // FourQ encode() writes the signature with an aligned 32-byte store; sign into a 32-byte-aligned
    // buffer, then copy into the (possibly unaligned) packet field.
    alignas(32) unsigned char sig[64];
    sign(signingSubseed, signingPublicKey, digest, sig);
    memcpy(packet.signature, sig, 64);

    return sock.sendData((char*)&packet, sizeof(packet));
}

// Shared mining job: what the workers evolve against
struct MineJob
{
    AntMinerT::ANN parentAnn;
    unsigned int parentTick;
    unsigned int parentSolutionIndexInTick;
    unsigned int parentScore;
    // 0 for a ROOT parent
    unsigned int parentDepth;
    unsigned int anchorTick;
    unsigned char anchorDigest[32];
    unsigned int threshold;
    bool valid;
};
static MineJob gJob;
static std::mutex gJobMutex;

// A solution that cleared threshold and parent score on a worker; the coordinator applies the
// per-parent child-cap gate and submits.
struct MineResult
{
    unsigned char nonce[32];
    unsigned int score;
    unsigned int anchorTick;
    unsigned int parentTick;
    unsigned int parentSolutionIndexInTick;
    unsigned int parentScore;
    unsigned int parentDepth;
    AntMinerT::ANN ann;
};
static std::vector<MineResult> gResults;
static std::mutex gResultsMutex;
static std::atomic<unsigned long long> gIterations(0);

// The bpp9000 task (topology + target series) the node scores against. A miner must use the same
// pinned task or its scores will not match. Reads the raw file and returns pointers into the kept
// buffer; the header hashes are not re-verified here (the node enforces them).
static std::vector<unsigned char> gTaskBuffer;
static const unsigned char* gTopoBlock = nullptr;
static const unsigned char* gDataBlock = nullptr;
static unsigned char gTaskTopoHash[32];   // K12 of the loaded topology block, compared to the node's canonical hash
static unsigned char gTaskDataHash[32];   // K12 of the loaded data block

static bool loadBpp9000TaskFile(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (!f)
    {
        printf("Cannot open task file: %s\n", path);
        return false;
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < (long)sizeof(task_file::TaskFileHeader))
    {
        printf("Task file too small.\n");
        fclose(f);
        return false;
    }
    gTaskBuffer.resize((size_t)sz);
    if (fread(gTaskBuffer.data(), 1, (size_t)sz, f) != (size_t)sz)
    {
        printf("Task file read failed.\n");
        fclose(f);
        return false;
    }
    fclose(f);

    const task_file::TaskFileHeader* h = (const task_file::TaskFileHeader*)gTaskBuffer.data();
    if (h->magic != task_file::MAGIC)
    {
        printf("Task file bad magic.\n");
        return false;
    }
    if (h->numPairs < score_bpp9000::SEQUENCE_LENGTH)
    {
        printf("Task file has fewer pairs than the scored sequence length.\n");
        return false;
    }
    const unsigned long long topoBytes = task_file::topologyBytes(h->numInputTrits, h->numOutputTrits, h->population, h->numNeighbors);
    // The canonical hashes cover only the scored sequenceLength pairs; pairs beyond them are an
    // informational holdout tail.
    const unsigned long long dataBytesLen = task_file::dataBytes(h->numInputTrits, h->numOutputTrits, score_bpp9000::SEQUENCE_LENGTH);
    gTopoBlock = gTaskBuffer.data() + sizeof(task_file::TaskFileHeader);
    gDataBlock = gTopoBlock + topoBytes;
    KangarooTwelve(gTopoBlock, (unsigned int)topoBytes, gTaskTopoHash, 32);
    KangarooTwelve(gDataBlock, (unsigned int)dataBytesLen, gTaskDataHash, 32);
    printf("Loaded task: N=%u M=%u P=%u T=%llu.\n", h->numInputTrits, h->numOutputTrits, h->population, (unsigned long long)h->numPairs);
    return true;
}

// Worker: pure compute, never touches the network. Own engine, shared read-only pool.
static void mineWorker(const unsigned char* pool, const unsigned char* computorPublicKey)
{
    auto miner = std::make_unique<AntMinerT>();
    miner->setPool(pool);
    miner->loadTaskFromMemory(gTopoBlock, gDataBlock);

    unsigned char pubkey[32];
    memcpy(pubkey, computorPublicKey, 32);

    while (!state)
    {
        MineJob job;
        {
            std::lock_guard<std::mutex> guard(gJobMutex);
            job = gJob;
        }
        if (!job.valid)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        unsigned char nonce[32];
        _rdrand64_step((unsigned long long*)&nonce[0]);
        _rdrand64_step((unsigned long long*)&nonce[8]);
        _rdrand64_step((unsigned long long*)&nonce[16]);
        _rdrand64_step((unsigned long long*)&nonce[24]);
        // Make sure the nonce is canonical
        nonce[0] = 1;                                                                     // AlgoType::Bpp9000
        nonce[1] = (unsigned char)((nonce[1] % score_bpp9000::MAX_LUT_ENTRIES_PER_STEP) + 1); // L in [1, 10]
        nonce[2] = (unsigned char)(nonce[2] % (score_bpp9000::NUMBER_OF_MUTATIONS + 1));       // K in [0, 100]
        const unsigned int score = miner->computeScoreFromParent(job.parentAnn.lut, pubkey, nonce, job.anchorDigest);
        gIterations++;

        // Lower is better: pass the threshold, strictly beat the parent. INVALID_SCORE_VALUE
        // (0xFFFFFFFF) is a timeout/non-canonical result and is filtered by the <= threshold test.
        if (score <= job.threshold && score < job.parentScore)
        {
            MineResult result;
            memcpy(result.nonce, nonce, 32);
            result.score = score;
            result.anchorTick = job.anchorTick;
            result.parentTick = job.parentTick;
            result.parentSolutionIndexInTick = job.parentSolutionIndexInTick;
            result.parentScore = job.parentScore;
            result.parentDepth = job.parentDepth;
            memcpy(&result.ann, &miner->bestANN, sizeof(AntMinerT::ANN));
            std::lock_guard<std::mutex> guard(gResultsMutex);
            gResults.push_back(result);
        }
    }
}

int main(int argc, char* argv[])
{
    if (argc < 5 || argc > 11)
    {
        printf("Usage: AntMiner [Node IP] [Node Port] [MiningID] [Signing Seed] [Threads] --task FILE [--operator SEED]\n");
        printf("  Threads:     mining thread count; default = hardware cores - 1\n");
        printf("  --task FILE: the pinned bpp9000 task file (required)\n");
        printf("  --operator SEED: node operator seed for identity-tree queries; default = Signing Seed\n");
        return 1;
    }
    int requestedThreads = 0;
    const char* taskFilePath = nullptr;
    const char* operatorSeed = nullptr;
    for (int i = 5; i < argc; i++)
    {
        if (strcmp(argv[i], "--task") == 0 && i + 1 < argc)
        {
            taskFilePath = argv[++i];
        }
        else if (strcmp(argv[i], "--operator") == 0 && i + 1 < argc)
        {
            operatorSeed = argv[++i];
        }
        else
        {
            requestedThreads = std::atoi(argv[i]);
        }
    }
    nodeIp = argv[1];
    nodePort = std::atoi(argv[2]);
    char* miningID = argv[3];
    char* signingSeed = argv[4];

    consoleCtrlHandler();

    unsigned char computorPublicKey[32];
    getPublicKeyFromIdentity(miningID, computorPublicKey);

    unsigned char signingSubseed[32];
    unsigned char signingPrivateKey[32];
    unsigned char signingPublicKey[32];
    getSubseedFromSeed((unsigned char*)signingSeed, signingSubseed);
    getPrivateKeyFromSubSeed(signingSubseed, signingPrivateKey);
    getPublicKeyFromPrivateKey(signingPrivateKey, signingPublicKey);

    unsigned char operatorSubseed[32];
    unsigned char operatorPublicKey[32];
    if (operatorSeed != nullptr)
    {
        unsigned char operatorPrivateKey[32];
        getSubseedFromSeed((unsigned char*)operatorSeed, operatorSubseed);
        getPrivateKeyFromSubSeed(operatorSubseed, operatorPrivateKey);
        getPublicKeyFromPrivateKey(operatorPrivateKey, operatorPublicKey);
    }
    else
    {
        memcpy(operatorSubseed, signingSubseed, 32);
        memcpy(operatorPublicKey, signingPublicKey, 32);
    }

    printf("AntMiner is launched. Connecting to %s:%d, mining for %s\n", nodeIp, nodePort, miningID);

    ServerSocket sock;
    if (!sock.establishConnection(nodeIp, nodePort))
    {
        return 1;
    }

    RespondCurrentTickInfo tickInfo;
    if (!queryCurrentTickInfo(sock, tickInfo))
    {
        printf("Failed to query current tick info.\n");
        return 1;
    }
    printf("Epoch %u, tick %u (initial %u)\n", tickInfo.epoch, tickInfo.tick, tickInfo.initialTick);

    RespondAntEpochContext epochContext;
    if (!queryEpochContext(sock, epochContext))
    {
        printf("Failed to query ant epoch context.\n");
        return 1;
    }
    bool digestIsZero = true;
    for (int i = 0; i < 32; i++)
    {
        if (epochContext.spectrumDigest[i] != 0)
        {
            digestIsZero = false;
        }
    }
    if (digestIsZero)
    {
        printf("Node returned a zero spectrum digest - ant colony not initialized on the node?\n");
        return 1;
    }
    printf("Ant epoch context: threshold %u, freshness window %u ticks, %u solutions so far\n",
        epochContext.threshold, epochContext.freshnessWindow, epochContext.solutionCount);

    // Full pipeline start
    std::vector<unsigned char> sharedPool;
    std::unique_ptr<AntMinerT> miner;
    AntMinerT::ANN rootAnn;
    memset(&rootAnn, 0, sizeof(rootAnn));
    memset(&gJob, 0, sizeof(gJob));
    std::vector<std::thread> workers;
    {
        if (!taskFilePath || !loadBpp9000TaskFile(taskFilePath))
        {
            printf("A --task FILE is required for mining.\n");
            return 1;
        }
        if (memcmp(gTaskTopoHash, epochContext.topologyHash, 32) != 0
            || memcmp(gTaskDataHash, epochContext.dataHash, 32) != 0)
        {
            printf("TASK MISMATCH: your --task file is not the one the node scores against.\n");
            printf("  topology block: %s\n", memcmp(gTaskTopoHash, epochContext.topologyHash, 32) ? "DIFFERS" : "ok");
            printf("  data block    : %s\n", memcmp(gTaskDataHash, epochContext.dataHash, 32) ? "DIFFERS" : "ok");
            printf("Get the epoch's canonical bpp9000 task file - a wrong task wastes work and forfeits computor deposits.\n");
            return 1;
        }
        sharedPool.resize(POOL_VEC_PADDING_SIZE);
        generateRandom2Pool(epochContext.spectrumDigest, sharedPool.data());
        miner = std::make_unique<AntMinerT>();
        miner->setPool(sharedPool.data());
        miner->loadTaskFromMemory(gTopoBlock, gDataBlock);
        miner->deriveRootANN(epochContext.spectrumDigest, rootAnn);
        printf("Epoch root derived.\n");

        unsigned int threadCount = std::thread::hardware_concurrency();
        threadCount = (threadCount > 1) ? (threadCount - 1) : 1;
        if (requestedThreads > 0)
        {
            threadCount = (unsigned int)requestedThreads;
        }
        for (unsigned int t = 0; t < threadCount; t++)
        {
            workers.emplace_back(mineWorker, sharedPool.data(), computorPublicKey);
        }
        printf("%u mining threads started.\n", threadCount);
    }

    std::vector<OwnNode> ownNodes;
    // Latest identity-tree listing (refreshed each resolve cycle) used to resolve our
    // submitted nodes' selfRefs.
    std::vector<AntIdentityTreeNode> listing;
    unsigned long long submitted = 0;
    unsigned long long staleSkipped = 0;   // hits dropped because the anchor left the freshness window before submit
    unsigned int maxSubmitAge = 0;          // worst anchor age (ticks) among submitted solutions, vs freshnessWindow
    auto lastResolveTime = std::chrono::steady_clock::now();
    auto lastLutCheckTime = std::chrono::steady_clock::now();

    unsigned int lastAnchorCandidateTick = 0xFFFFFFFFU;
    unsigned int cachedAnchorTick = 0xFFFFFFFFU;
    unsigned char cachedAnchorDigest[32];

    // Adopt the tree this identity already holds on the node, so a restart continues from the
    // on-chain frontier instead of climbing again from ROOT. One tree walk plus one ANN fetch per
    // node; nothing is stored locally.
    {
        std::vector<AntIdentityTreeNode> existing;
        if (!fetchIdentityTree(sock, operatorSubseed, operatorPublicKey, computorPublicKey, existing))
        {
            printf("WARNING: could not read this identity's existing tree%s - mining restarts from ROOT.\n",
                (operatorSeed != nullptr) ? "" : " (no --operator seed given; the tree query is operator-signed)");
        }
        else
        {
            listing = existing;
            unsigned int usable = 0;
            unsigned int withoutAnn = 0;
            unsigned int bestScore = 0xFFFFFFFFU;
            unsigned int maxDepth = 0;
            for (const AntIdentityTreeNode& entry : existing)
            {
                OwnNode node;
                memset(&node, 0, sizeof(node));   // nonce stays zero: unused for a node we did not mine
                node.score = entry.score;
                node.anchorTick = entry.anchorTick;
                node.depth = entry.depth;
                node.parentTick = entry.parentTick;
                node.parentSolutionIndexInTick = entry.parentSolutionIndexInTick;
                node.selfTick = entry.selfTick;
                node.selfSolutionIndexInTick = entry.selfSolutionIndexInTick;
                node.refKnown = true;

                unsigned char storedLut[CANONICAL_ANN_BYTES];
                const int result = queryParentAnn(sock, operatorSubseed, operatorPublicKey,
                    entry.selfTick, entry.selfSolutionIndexInTick, storedLut);
                if (result < 0)
                {
                    printf("WARNING: network failure while adopting the existing tree - adopted %zu of %zu nodes.\n",
                        ownNodes.size(), existing.size());
                    break;
                }
                if (result == 0)
                {
                    // No ANN for a ref the node just listed - the colony moved under us. Keep the
                    // node so the resolve loop claims its entry, but never extend it.
                    node.lutChecked = true;
                    node.lutMismatch = true;
                    withoutAnn++;
                }
                else
                {
                    // Came from the node, so there is nothing to verify.
                    annFromCanonicalLut(*miner, storedLut, rootAnn, node.ann);
                    node.lutChecked = true;
                    node.lutMismatch = false;
                    usable++;
                    if (entry.score < bestScore)
                    {
                        bestScore = entry.score;
                    }
                    if (entry.depth > maxDepth)
                    {
                        maxDepth = entry.depth;
                    }
                }
                ownNodes.push_back(node);
            }
            if (!existing.empty())
            {
                printf("Adopted %u existing tree nodes (best score %u, max depth %u)",
                    usable, usable ? bestScore : 0U, maxDepth);
                if (withoutAnn)
                {
                    printf(", %u without a stored ANN (not extendable)", withoutAnn);
                }
                printf("\n");
            }
        }
    }

    while (!state)
    {
        // Parent selection: the best own node whose selfRef is known (deepest frontier),
        // or ROOT when none is resolved yet.
        // Pool miner can tune here to select the parent that maximize their miner score
        const OwnNode* parentNode = NULL;
        for (const OwnNode& node : ownNodes)
        {
            if (node.refKnown && !node.lutMismatch && (parentNode == NULL || node.score < parentNode->score))
            {
                parentNode = &node;
            }
        }

        // Exploration (breadth): 1 in ANT_EXPLORE_ONE_IN, jump to a random resolved node or the
        // root instead of the best, so we do not get stuck descending a single basin.
        unsigned long long exploreRoll = 0;
        _rdrand64_step(&exploreRoll);
        if ((exploreRoll % ANT_EXPLORE_ONE_IN) == 0)
        {
            parentNode = NULL;   // root: a fresh basin
            if (!ownNodes.empty())
            {
                const OwnNode& pick = ownNodes[(exploreRoll >> 8) % ownNodes.size()];
                if (pick.refKnown && !pick.lutMismatch)
                {
                    parentNode = &pick;
                }
            }
        }
        const unsigned int parentTick = parentNode ? parentNode->selfTick : ROOT_TICK;
        const unsigned int parentSolutionIndexInTick = parentNode ? parentNode->selfSolutionIndexInTick : ROOT_INDEX_IN_TICK;
        const unsigned int parentScore = parentNode ? parentNode->score : 0xFFFFFFFFU;   // root: WORST
        const AntMinerT::ANN& parentAnn = parentNode ? parentNode->ann : rootAnn;

        // Anchor-first: the anchor digest is part of the child RNG seed, so the anchor is chosen
        // BEFORE mining. Anchor at the latest COMPLETED tick (current - 1), stepping back past any
        // tick the node stored no TickData for since those carry no anchor. The digest is derived
        // from standard messages only and refetched when the tick advances, so the anchor stays
        // close to the current tick, well within the freshness window.
        if (!queryCurrentTickInfo(sock, tickInfo))
        {
            printf("Tick info query failed, reconnecting...\n");
            sock.closeConnection();
            while (!state && !sock.establishConnection(nodeIp, nodePort))
            {
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            continue;
        }
        if (tickInfo.tick <= tickInfo.initialTick)
        {
            // No completed tick in this epoch yet - previous-epoch anchors are not in the ring.
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        const unsigned int candidateTick = tickInfo.tick - 1U;
        if (candidateTick != lastAnchorCandidateTick)
        {
            unsigned int resolvedTick = 0;
            const int resolved = resolveAnchorDigest(sock, candidateTick, tickInfo.initialTick,
                resolvedTick, cachedAnchorDigest);
            if (resolved < 0)
            {
                printf("Anchor fetch failed, reconnecting...\n");
                sock.closeConnection();
                while (!state && !sock.establishConnection(nodeIp, nodePort))
                {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
                continue;
            }
            if (resolved == 0)
            {
                // Nothing anchored in the walk-back window yet - retry next iteration.
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            lastAnchorCandidateTick = candidateTick;
            cachedAnchorTick = resolvedTick;
            // Same 8-byte prefix the node prints on F3 ("AntColony: anchor tick T digest=...");
            // compare to spot an anchor-digest divergence at a specific tick. The vote counts are a
            // node-health readout: a large misaligned share means the node may not match the network.
            unsigned long long digestPrefix = 0;
            memcpy(&digestPrefix, cachedAnchorDigest, 8);
            // printf("Anchor tick %u digest=%llu (votes aligned %u, misaligned %u)\n",
            //     cachedAnchorTick, digestPrefix,
            //     (unsigned int)tickInfo.numberOfAlignedVotes, (unsigned int)tickInfo.numberOfMisalignedVotes);
        }

        {
            // Publish the job for the mining threads (parent, anchor, or threshold may have changed).
            {
                std::lock_guard<std::mutex> guard(gJobMutex);
                memcpy(&gJob.parentAnn, &parentAnn, sizeof(AntMinerT::ANN));
                gJob.parentTick = parentTick;
                gJob.parentSolutionIndexInTick = parentSolutionIndexInTick;
                gJob.parentScore = parentScore;
                gJob.parentDepth = parentNode ? parentNode->depth : 0U;
                gJob.anchorTick = cachedAnchorTick;
                memcpy(gJob.anchorDigest, cachedAnchorDigest, 32);
                gJob.threshold = epochContext.threshold;
                gJob.valid = true;
            }

            // Drain worker hits: apply the per-parent child-cap gate, submit, track as own nodes.
            std::vector<MineResult> results;
            {
                std::lock_guard<std::mutex> guard(gResultsMutex);
                results.swap(gResults);
            }
            bool submitFailed = false;
            for (size_t i = 0; i < results.size(); i++)
            {
                const MineResult& r = results[i];
                // On-time gate: if the anchor already left the freshness window, the node rejects it as
                // stale and forfeits the deposit. Drop it and count the miss instead of submitting late.
                if (tickInfo.tick > r.anchorTick + epochContext.freshnessWindow)
                {
                    staleSkipped++;
                    continue;
                }
                // Threshold and beat-parent are already guaranteed by the mining; only the child
                // cap is left. 0 = unbound. Counted from the node's listing, which survives a
                // restart, plus submissions the listing has not caught up with yet. Submitting
                // over the cap wastes the solution and risks the computor's deposit.
                if (epochContext.maxChildrenPerParent != 0
                    && childCountOf(listing, ownNodes, r.parentTick, r.parentSolutionIndexInTick)
                        >= epochContext.maxChildrenPerParent)
                {
                    continue;
                }

                if (!submitAntSolution(sock, signingSubseed, signingPrivateKey, signingPublicKey, computorPublicKey,
                    r.parentTick, r.parentSolutionIndexInTick, r.anchorTick, r.score, r.nonce))
                {
                    // Requeue this and the remaining results; they stay valid within the
                    // freshness window and get another chance after the reconnect.
                    std::lock_guard<std::mutex> guard(gResultsMutex);
                    gResults.insert(gResults.end(), results.begin() + i, results.end());
                    submitFailed = true;
                    break;
                }
                submitted++;
                const unsigned int anchorAge = tickInfo.tick - r.anchorTick;
                if (anchorAge > maxSubmitAge)
                {
                    maxSubmitAge = anchorAge;
                }
                OwnNode node;
                memcpy(node.nonce, r.nonce, 32);
                node.score = r.score;
                node.anchorTick = r.anchorTick;
                node.depth = r.parentDepth + 1U;
                node.parentTick = r.parentTick;
                node.parentSolutionIndexInTick = r.parentSolutionIndexInTick;
                node.refKnown = false;
                node.lutChecked = false;
                node.lutMismatch = false;
                node.resolveAttempts = 0;
                node.selfTick = 0;
                node.selfSolutionIndexInTick = 0;
                memcpy(&node.ann, &r.ann, sizeof(AntMinerT::ANN));
                ownNodes.push_back(node);
                printf("Submitted: depth %u, score %u (parent score %u), anchor %u\n",
                    node.depth, r.score, r.parentScore, r.anchorTick);
            }
            if (submitFailed)
            {
                printf("Submit failed, reconnecting...\n");
                sock.closeConnection();
                while (!state && !sock.establishConnection(nodeIp, nodePort))
                {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
                continue;
            }
        }

        // Periodically resolve our submitted nodes' selfRefs from the node's listing and
        // report tree growth (the proof-of-working readout).
        const auto now = std::chrono::steady_clock::now();

        // Verify each resolved own node's local ANN against the node's stored ANN, once per node.
        // A mismatch means children mined from the local copy would score differently on-chain,
        // wasting work and deposits, so the node is excluded from parent selection.
        if (std::chrono::duration_cast<std::chrono::seconds>(now - lastLutCheckTime).count() >= 60)
        {
            lastLutCheckTime = now;
            unsigned int attempted = 0;
            unsigned int checkedNow = 0;
            unsigned int mismatches = 0;
            unsigned int unavailable = 0;
            bool netFailed = false;
            for (OwnNode& node : ownNodes)
            {
                if (!node.refKnown || node.lutChecked)
                {
                    continue;
                }
                attempted++;
                unsigned char storedLut[CANONICAL_ANN_BYTES];
                const int result = queryParentAnn(sock, operatorSubseed, operatorPublicKey,
                    node.selfTick, node.selfSolutionIndexInTick, storedLut);
                if (result < 0)
                {
                    netFailed = true;
                    break;
                }
                if (result == 0)
                {
                    unavailable++;
                    continue;
                }
                node.lutChecked = true;
                checkedNow++;
                if (!annMatchesCanonicalLut(*miner, node.ann, storedLut))
                {
                    node.lutMismatch = true;
                    mismatches++;
                    printf("WARNING: LUT mismatch for own node (tick %u, index %u, score %u) - local ANN differs from the node's stored ANN, excluded from parent selection\n",
                        node.selfTick, node.selfSolutionIndexInTick, node.score);
                }
            }
            if (attempted)
            {
                printf("LUT check: attempted %u, verified %u, mismatched %u, unavailable %u%s\n",
                    attempted, checkedNow, mismatches, unavailable, netFailed ? ", aborted on network failure" : "");
            }
        }

        if (std::chrono::duration_cast<std::chrono::seconds>(now - lastResolveTime).count() >= 10)
        {
            lastResolveTime = now;

            std::vector<AntIdentityTreeNode> entries;
            const bool queryOk = fetchIdentityTree(sock, operatorSubseed, operatorPublicKey, computorPublicKey, entries);

            if (queryOk)
            {
                listing = entries;
                unsigned int resolved = 0;

                // A listing entry carries no nonce, so a submission is matched on
                // (score, anchorTick, depth, parentRef). Two of our hits can share that tuple, so
                // claim each entry as it binds - a plain first-match would give both the same
                // selfRef. A swap between equal entries is caught by the LUT check above.
                std::vector<bool> claimed(entries.size(), false);
                for (const OwnNode& node : ownNodes)
                {
                    if (!node.refKnown)
                    {
                        continue;
                    }
                    for (size_t e = 0; e < entries.size(); e++)
                    {
                        if (entries[e].selfTick == node.selfTick
                            && entries[e].selfSolutionIndexInTick == node.selfSolutionIndexInTick)
                        {
                            claimed[e] = true;
                            break;
                        }
                    }
                }

                for (OwnNode& node : ownNodes)
                {
                    if (node.refKnown)
                    {
                        resolved++;
                        continue;
                    }
                    for (size_t e = 0; e < entries.size(); e++)
                    {
                        if (claimed[e])
                        {
                            continue;
                        }
                        const AntIdentityTreeNode& entry = entries[e];
                        if (entry.score == node.score
                            && entry.anchorTick == node.anchorTick
                            && entry.depth == node.depth
                            && entry.parentTick == node.parentTick
                            && entry.parentSolutionIndexInTick == node.parentSolutionIndexInTick)
                        {
                            claimed[e] = true;
                            node.refKnown = true;
                            node.selfTick = entry.selfTick;
                            node.selfSolutionIndexInTick = entry.selfSolutionIndexInTick;
                            resolved++;
                            break;
                        }
                    }
                    if (!node.refKnown)
                    {
                        node.resolveAttempts++;
                        if (node.resolveAttempts == 3U)
                        {
                            // The node recomputes the score from (pubkey, parentRef, anchorTick,
                            // nonce); if our score never appears on-chain, the recomputation
                            // disagreed with ours - the anchor digest is the prime suspect.
                            printf("WARNING: solution (score %u, anchor %u) not accepted after %u resolve cycles - possible anchor digest mismatch, compare 'Anchor tick N digest=' with the node's F3 line\n",
                                node.score, node.anchorTick, node.resolveAttempts);
                        }
                    }
                }

                RespondAntEpochContext refreshed;
                if (queryEpochContext(sock, refreshed))
                {
                    epochContext = refreshed;
                }
                printf("| %llu iterations | %llu submitted | %llu stale-skipped | max-anchor-age %u/%u | %u/%zu accepted+resolved | tree size %u |\n",
                    gIterations.load(), submitted, staleSkipped, maxSubmitAge, epochContext.freshnessWindow, resolved, ownNodes.size(), epochContext.solutionCount);
            }
            else
            {
                printf("Resolve query failed, reconnecting...\n");
                sock.closeConnection();
                while (!state && !sock.establishConnection(nodeIp, nodePort))
                {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
            }
        }
        // Coordinator pace: the workers mine continuously; this loop only shuttles jobs,
        // results, and queries.
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    for (std::thread& worker : workers)
    {
        worker.join();
    }
    sock.closeConnection();
    printf("AntMiner is shut down. %llu iterations, %llu submitted, %llu stale-skipped (max anchor age %u/%u), %zu own nodes.\n",
        gIterations.load(), submitted, staleSkipped, maxSubmitAge, epochContext.freshnessWindow, ownNodes.size());
    return 0;
}
