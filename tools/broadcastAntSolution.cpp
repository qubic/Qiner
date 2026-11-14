// broadcastAntSolution: standalone ant-solution submitter for test networks. Sibling of
// tools/broadcastMessageSolution.cpp, but for the ant-colony pipeline.
//
// Random canonical nonces, no search: each one is walked once on the task data of its anchor tick
// (asked from the node, like AntMiner does) and sent with the score that walk gave, because the node
// drops a solution whose claimed score differs from its own. Every other gate still applies, so only
// genuinely valid solutions are accepted. Under a permissive epoch threshold most ROOT children are,
// so the tree grows - exercising accepts, deposits, ranking and multi-node consensus.
//
// Reports how many solutions the node committed, not only how many were sent. The counter is
// network-wide, so with other miners active it is a floor.
//
// Usage:
//   broadcastAntSolution <Node IP> <Node Port> <MiningID> <Signing Seed> [count=1] [intervalMs=0] [-operator <Operator Seed>] [-repeat <n>] [-claim <score>]
//     MiningID   : the computor the solution is FOR - tree owner, deposit payer, broadcast destination.
//     Signing Seed  : a computor/funded seed - the broadcast SOURCE that signs the solution. Its identity
//                    may be the MiningID (submit-for-self) or a different one (submit-for-other).
//     -operator S  : the node operator's seed. Signs the operator-only tree read so children can extend
//                    real nodes (depth). Omit for a ROOT-only depth-1 flood. Admin credential - pass it
//                    only when you want depth.
//     count        : number of solutions to send (default 1; use a large number to flood)
//     intervalMs   : delay between sends (default 0)
//     -repeat n    : send each solution n times back to back (default 1). Every send is a new packet
//                    (fresh gamming nonce), so the network's duplicate filter passes it, as it passes a
//                    miner's resend; the node sees the same solution n times.
//     -claim s     : send this claimed score and skip the local walk. The node drops the solution
//                    unless s happens to be right, so this exercises its claim check.
//
// One walk per solution is what the node spends to verify it, in this miner's plain C++ scorer: seconds
// at the production population. Use -claim, or AntMiner, when a higher send rate is needed.

#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <memory>
#include <vector>

#ifdef _MSC_VER
#include <intrin.h>
#else
#include <immintrin.h>
#endif

#include "score_bpp9000.h"
#include "keyUtils.h"
#include "network.h"
#include "ant_colony_message.h"

// bpp9000 canonical nonce (src/score_bpp9000.h): nonce[0] == 1 selects Bpp9000, nonce[1] = L in
// [1, 10] (bits 0-3) + the wiring mode (bits 4-5), nonce[2] = K, the explore-step count; every byte
// value is canonical for K.
static constexpr unsigned char ALGO_BPP9000 = 1;

// The scorer with the deployed parameters, as in src/ant/AntMiner.cpp.
using AntMinerT = score_bpp9000::Miner<
    score_bpp9000::NUMBER_OF_INPUT_NEURONS,
    score_bpp9000::NUMBER_OF_OUTPUT_NEURONS,
    score_bpp9000::SEQUENCE_LENGTH,
    score_bpp9000::WINDOW_WIDTH,
    score_bpp9000::MAX_NUMBER_OF_TICKS,
    score_bpp9000::NUMBER_OF_NEIGHBORS,
    score_bpp9000::POPULATION_THRESHOLD,
    score_bpp9000::NUMBER_OF_MUTATIONS,
    score_bpp9000::SOLUTION_THRESHOLD,
    score_bpp9000::SHIFT_CAP>;
static constexpr unsigned long long ANN_BYTES = sizeof(AntMinerT::ANN);

// --- request/response helpers (from src/AntMiner.cpp) ---
static int waitForResponse(ServerSocket& sock, unsigned char wantedType, char* payload, unsigned int payloadCapacity)
{
    static char scratch[1024 * 1024];
    // The reply can sit behind many tick and vote broadcasts: bound the skip by time, not by count.
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < deadline)
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

// Unsigned read of the epoch's mining parameters. A node on another threshold or freshness window
// rejects every solution, which the send loop cannot tell from a send that never landed.
static bool queryEpochContext(ServerSocket& sock, RespondAntEpochContext& out)
{
    if (!sendRequest(sock, REQUEST_ANT_EPOCH_CONTEXT, NULL, 0))
    {
        return false;
    }
    return waitForResponse(sock, RESPOND_ANT_EPOCH_CONTEXT, (char*)&out, sizeof(out)) == (int)sizeof(out);
}

// Task data of one anchor tick. Returns 1 when the node answered (out.valid tells whether it holds
// the data), -1 on network failure.
static int queryTaskData(ServerSocket& sock, unsigned int anchorTick, RespondBpp9000TaskData& out)
{
    RequestBpp9000TaskData request;
    request.anchorTick = anchorTick;
    if (!sendRequest(sock, REQUEST_BPP9000_TASK_DATA, &request, sizeof(request)))
    {
        return -1;
    }
    memset(&out, 0, sizeof(out));
    const int received = waitForResponse(sock, RESPOND_BPP9000_TASK_DATA, (char*)&out, sizeof(out));
    if (received < (int)RESPOND_BPP9000_TASK_DATA_HEADER_SIZE)
    {
        return -1;
    }
    if (out.valid
        && (out.anchorTick != anchorTick
            || out.shiftLimit > score_bpp9000::SHIFT_CAP
            || (unsigned int)received != RESPOND_BPP9000_TASK_DATA_HEADER_SIZE + (unsigned int)score_bpp9000::WINDOW_WIDTH + out.shiftLimit))
    {
        out.valid = 0;
    }
    return 1;
}

// The anchor digest of anchorTick as the node derives it: K12(anchorTick || K12(TickData)).
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
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < deadline)
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
            unsigned char transactionDigest[32];
            KangarooTwelve(tickData, TICK_DATA_SIZE, transactionDigest, 32);

            unsigned char input[36];
            memcpy(input, &anchorTick, 4);
            memcpy(input + 4, transactionDigest, 32);
            KangarooTwelve(input, 36, anchorDigest, 32);
            return 1;
        }
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

// Ticks the node holds no data for carry no anchor, so step back from fromTick until one does.
static constexpr unsigned int MAX_ANCHOR_WALK_BACK = 16U;

// Returns 1 with anchorTick/anchorDigest filled, 0 when nothing anchored was found above minTick
// within the walk-back bound, -1 on network failure.
static int resolveAnchorDigest(ServerSocket& sock, unsigned int fromTick, unsigned int minTick,
    unsigned int& anchorTick, unsigned char anchorDigest[32])
{
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

// One stored node's network, read back from the node (operator-signed).
// Returns 1 with outAnn filled, 0 when the node answered without a usable ANN, -1 on network failure.
static int queryParentAnn(ServerSocket& sock, const unsigned char* operatorSubseed, const unsigned char* operatorPublicKey,
    unsigned int refTick, unsigned int refSolutionIndexInTick, unsigned char* outAnn)
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
    static char buffer[sizeof(RespondAntParentAnnHeader) + ANN_BYTES];
    const int received = waitForResponse(sock, RESPOND_ANT_PARENT_ANN, buffer, sizeof(buffer));
    if (received < (int)sizeof(RespondAntParentAnnHeader))
    {
        return -1;
    }
    const RespondAntParentAnnHeader* respHeader = (const RespondAntParentAnnHeader*)buffer;
    if (respHeader->status != ANT_PARENT_ANN_STATUS_OK
        || respHeader->annSizeBytes != ANN_BYTES
        || received < (int)(sizeof(RespondAntParentAnnHeader) + ANN_BYTES))
    {
        return 0;
    }
    memcpy(outAnn, buffer + sizeof(RespondAntParentAnnHeader), ANN_BYTES);
    return 1;
}

static void printEpochContext(const char* label, const RespondAntEpochContext& ctx)
{
    printf("%s epoch %u, threshold %u, freshness window %u ticks, maxChildrenPerParent %u\n",
        label, (unsigned int)ctx.epoch, ctx.threshold, ctx.freshnessWindow, ctx.maxChildrenPerParent);
    printf("%s solutions accepted so far %u, free ANN slots %u\n",
        label, ctx.solutionCount, ctx.freeAnnSlotsCount);
}

// Operator-signed read of one identity's tree (one page; caller loops on nextIndex). Signed with the
// same seed used to broadcast; the node verifies it against operatorPublicKey.
static bool queryIdentityTree(ServerSocket& sock,
    const unsigned char* signingSubseed, const unsigned char* signingPublicKey,
    const unsigned char* pubkey, unsigned int fromIndex,
    std::vector<AntIdentityTreeNode>& outEntries, unsigned int& nextIndex)
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

// Page the whole tree into 'listing'. A failed or unsigned read leaves it empty -> ROOT-only.
static void refreshListing(ServerSocket& sock,
    const unsigned char* signingSubseed, const unsigned char* signingPublicKey,
    const unsigned char* pubkey, std::vector<AntIdentityTreeNode>& listing)
{
    listing.clear();
    unsigned int fromIndex = 0;
    for (int page = 0; page < 4096; page++)
    {
        unsigned int nextIndex = 0;
        if (!queryIdentityTree(sock, signingSubseed, signingPublicKey, pubkey, fromIndex, listing, nextIndex))
        {
            return;
        }
        if (nextIndex == 0)
        {
            return;
        }
        fromIndex = nextIndex;
    }
}

// --- ant broadcast (from src/AntMiner.cpp submitAntSolution) ---
// Submit one ant solution as a BroadcastMessage whose decrypted gammingKey[0] selects
// MESSAGE_TYPE_ANT_SOLUTION.
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

static void fillRandomNonce(unsigned char* nonce)
{
    _rdrand64_step((unsigned long long*)&nonce[0]);
    _rdrand64_step((unsigned long long*)&nonce[8]);
    _rdrand64_step((unsigned long long*)&nonce[16]);
    _rdrand64_step((unsigned long long*)&nonce[24]);
}

static void reconnect(ServerSocket& sock, const char* nodeIp, int nodePort)
{
    sock.closeConnection();
    while (!sock.establishConnection((char*)nodeIp, nodePort))
    {
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

int main(int argc, char* argv[])
{
    if (argc < 5)
    {
        printf("Usage: broadcastAntSolution <Node IP> <Node Port> <MiningID> <Signing Seed> [count=1] [intervalMs=0] [-operator <Operator Seed>] [-repeat <n>] [-claim <score>]\n");
        printf("  Signing Seed:  a computor/funded seed - the broadcast SOURCE that signs the solution.\n");
        printf("  MiningID:   the computor the solution is FOR (tree owner, deposit payer).\n");
        printf("  -operator S:  operator seed; signs the tree read so children extend real nodes (depth). Omit = ROOT-only.\n");
        printf("  -repeat n:    send each solution n times back to back, as a resending miner would.\n");
        printf("  -claim s:     claim score s and skip the local walk (the node drops a wrong claim).\n");
        return 1;
    }

    const char* nodeIp = argv[1];
    const int nodePort = std::atoi(argv[2]);
    const char* miningID = argv[3];
    const char* signingSeed = argv[4];

    int count = 1;
    int intervalMs = 0;
    int repeat = 1;
    bool claimForced = false;
    unsigned int forcedClaim = 0;
    bool countSet = false;
    const char* operatorSeed = nullptr;   // signs the tree read (depth); NULL -> ROOT-only
    for (int i = 5; i < argc; i++)
    {
        if (strcmp(argv[i], "-operator") == 0 && i + 1 < argc)
        {
            operatorSeed = argv[++i];
        }
        else if (strcmp(argv[i], "-repeat") == 0 && i + 1 < argc)
        {
            repeat = std::atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "-claim") == 0 && i + 1 < argc)
        {
            claimForced = true;
            forcedClaim = (unsigned int)std::strtoul(argv[++i], nullptr, 10);
        }
        else if (!countSet)
        {
            count = std::atoi(argv[i]);
            countSet = true;
        }
        else
        {
            intervalMs = std::atoi(argv[i]);
        }
    }
    if (count < 1)
    {
        count = 1;
    }
    if (repeat < 1)
    {
        repeat = 1;
    }

    // Signing seed = broadcast SOURCE (signs the solution). MiningID = destination (tree owner).
    unsigned char computorPublicKey[32];
    unsigned char signingSubseed[32];
    unsigned char signingPrivateKey[32];
    unsigned char signingPublicKey[32];
    getPublicKeyFromIdentity(miningID, computorPublicKey);
    getSubseedFromSeed((const unsigned char*)signingSeed, signingSubseed);
    getPrivateKeyFromSubSeed(signingSubseed, signingPrivateKey);
    getPublicKeyFromPrivateKey(signingPrivateKey, signingPublicKey);

    // Operator seed (optional) = signs the operator-only tree read; enables extending real nodes (depth).
    unsigned char operatorSubseed[32];
    unsigned char operatorPrivateKey[32];
    unsigned char operatorPublicKey[32];
    if (operatorSeed != nullptr)
    {
        getSubseedFromSeed((const unsigned char*)operatorSeed, operatorSubseed);
        getPrivateKeyFromSubSeed(operatorSubseed, operatorPrivateKey);
        getPublicKeyFromPrivateKey(operatorPrivateKey, operatorPublicKey);
    }

    printf("broadcastAntSolution -> %s:%d, computor %s, depth %s, count %d, interval %d ms, repeat %d, ",
           nodeIp, nodePort, miningID,
           operatorSeed ? "on" : "ROOT-only", count, intervalMs, repeat);
    if (claimForced)
    {
        printf("claimed score %u (no local walk)\n", forcedClaim);
    }
    else
    {
        printf("scored locally (one walk per solution)\n");
    }

    ServerSocket sock;
    if (!sock.establishConnection((char*)nodeIp, nodePort))
    {
        printf("Failed to connect to %s:%d\n", nodeIp, nodePort);
        return 1;
    }

    // Baseline for the committed count.
    RespondAntEpochContext ctxBefore;
    const bool haveCtxBefore = queryEpochContext(sock, ctxBefore);
    if (haveCtxBefore)
    {
        printEpochContext("  before:", ctxBefore);
    }
    else
    {
        printf("  before: epoch context unavailable - acceptance cannot be reported\n");
        if (!claimForced)
        {
            printf("The walk needs the epoch context (pool seed, threshold). Stopping.\n");
            return 1;
        }
    }

    // The scorer of this epoch and this identity's root network.
    std::vector<unsigned char> pool;
    std::unique_ptr<AntMinerT> miner;
    static AntMinerT::ANN rootAnn;
    static AntMinerT::ANN parentAnn;
    static RespondBpp9000TaskData taskData;
    unsigned int taskDataAnchorTick = 0xFFFFFFFFU;
    bool taskDataWarned = false;
    if (!claimForced)
    {
        pool.resize(POOL_VEC_PADDING_SIZE);
        generateRandom2Pool(ctxBefore.spectrumDigest, pool.data());
        miner = std::make_unique<AntMinerT>();
        miner->setPool(pool.data());
        miner->deriveControlOutput(ctxBefore.spectrumDigest);
        miner->deriveRootANN(computorPublicKey, rootAnn);
    }

    unsigned int sent = 0;
    unsigned int expectedAccepted = 0;
    unsigned int notScored = 0;
    std::vector<AntIdentityTreeNode> listing;   // this identity's accepted nodes (for deeper extension)
    for (int c = 0; c < count; c++)
    {
        RespondCurrentTickInfo tickInfo;
        if (!queryCurrentTickInfo(sock, tickInfo) || tickInfo.tick <= tickInfo.initialTick)
        {
            printf("[%d/%d] tick query failed, reconnecting...\n", c + 1, count);
            reconnect(sock, nodeIp, nodePort);
            c--;
            continue;
        }

        // The latest completed tick the node holds data for.
        unsigned int anchorTick = 0;
        unsigned char anchorDigest[32];
        const int resolved = resolveAnchorDigest(sock, tickInfo.tick - 1U, tickInfo.initialTick, anchorTick, anchorDigest);
        if (resolved < 0)
        {
            printf("[%d/%d] anchor fetch failed, reconnecting...\n", c + 1, count);
            reconnect(sock, nodeIp, nodePort);
            c--;
            continue;
        }
        if (resolved == 0)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            c--;
            continue;
        }

        // The targets the node grades this anchor tick on, and how far the frame may slide.
        if (!claimForced && anchorTick != taskDataAnchorTick)
        {
            if (queryTaskData(sock, anchorTick, taskData) < 0)
            {
                printf("[%d/%d] task data fetch failed, reconnecting...\n", c + 1, count);
                reconnect(sock, nodeIp, nodePort);
                c--;
                continue;
            }
            if (!taskData.valid)
            {
                if (!taskDataWarned)
                {
                    taskDataWarned = true;
                    printf("The node has no task data for anchor tick %u yet - waiting.\n", anchorTick);
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
                c--;
                continue;
            }
            taskDataWarned = false;
            miner->loadTaskTargets(taskData.targets, (unsigned int)score_bpp9000::WINDOW_WIDTH + taskData.shiftLimit);
            taskDataAnchorTick = anchorTick;
        }

        // Every 16 sends, refresh the tree so we can extend real nodes (deeper), not just ROOT.
        // Then extend a random listed node ~70% of the time, ROOT the rest (to seed new depth-1 nodes).
        // ROOT-only when the tree is empty or the signed read is not accepted.
        if (operatorSeed != nullptr && (c % 16 == 0))
        {
            refreshListing(sock, operatorSubseed, operatorPublicKey, computorPublicKey, listing);
        }
        unsigned int parentTick = ROOT_TICK;
        unsigned int parentIndex = ROOT_INDEX_IN_TICK;
        score_bpp9000::Rating parentRating = score_bpp9000::Rating::worst();
        const AntMinerT::ANN* parentAnnToUse = &rootAnn;
        if (!listing.empty())
        {
            unsigned long long roll = 0;
            _rdrand64_step(&roll);
            if ((roll % 100U) >= 30U)
            {
                const AntIdentityTreeNode& p = listing[(roll >> 8) % listing.size()];
                bool usable = true;
                if (!claimForced)
                {
                    // A parent beyond this anchor's data cannot be extended, and its network must come
                    // from the node.
                    usable = (p.shift <= taskData.shiftLimit);
                    if (usable)
                    {
                        const int fetched = queryParentAnn(sock, operatorSubseed, operatorPublicKey,
                            p.selfTick, p.selfSolutionIndexInTick, (unsigned char*)&parentAnn);
                        if (fetched < 0)
                        {
                            printf("[%d/%d] parent network fetch failed, reconnecting...\n", c + 1, count);
                            reconnect(sock, nodeIp, nodePort);
                            c--;
                            continue;
                        }
                        usable = (fetched == 1);
                    }
                }
                if (usable)
                {
                    parentTick = p.selfTick;
                    parentIndex = p.selfSolutionIndexInTick;
                    parentRating.error = p.score;
                    parentRating.shift = p.shift;
                    parentAnnToUse = &parentAnn;
                }
            }
        }

        unsigned char nonce[32];
        fillRandomNonce(nonce);
        nonce[0] = ALGO_BPP9000;
        const unsigned char L = (unsigned char)((nonce[1] % score_bpp9000::MAX_CHANGES_PER_STEP) + 1);
        nonce[1] = (unsigned char)(L | (score_bpp9000::BPP9000_MODE_WIRING << 4));

        char nonceHex[65];
        for (int i = 0; i < 32; i++)
        {
            snprintf(nonceHex + i * 2, 3, "%02x", nonce[i]);
        }

        // One walk, the same the node will do; its error is the claim.
        unsigned int claimedScore = forcedClaim;
        char verdict[96];
        snprintf(verdict, sizeof(verdict), "claim %u, not scored", forcedClaim);
        if (!claimForced)
        {
            const score_bpp9000::Rating rating = miner->computeScoreFromParent(*parentAnnToUse, parentRating.shift,
                computorPublicKey, nonce, anchorDigest, taskData.shiftLimit);
            if (!rating.isValid())
            {
                notScored++;
                printf("[%d/%d] not sent: the walk gave no score, nonce %s\n", c + 1, count, nonceHex);
                continue;
            }
            claimedScore = rating.error;
            const char* expectation = "expect accept";
            if (!rating.clearsFloor(ctxBefore.threshold))
            {
                expectation = "expect drop: above the threshold";
            }
            // A stored parent is judged by its rating on this anchor tick's data; ROOT has none to beat.
            else if (!rating.isBetterThan((parentTick == ROOT_TICK) ? parentRating : miner->parentRating))
            {
                expectation = "expect drop: does not beat the parent";
            }
            else
            {
                expectedAccepted++;
            }
            snprintf(verdict, sizeof(verdict), "shift %u of %u, score %u, %s", rating.shift, taskData.shiftLimit,
                rating.error, expectation);
        }

        if (submitAntSolution(sock, signingSubseed, signingPrivateKey, signingPublicKey, computorPublicKey,
                parentTick, parentIndex, anchorTick, claimedScore, nonce))
        {
            sent++;
            if (parentIndex == ROOT_INDEX_IN_TICK)
            {
                printf("[%d/%d] sent: parent ROOT, anchor %u, %s, nonce %s\n", c + 1, count, anchorTick, verdict, nonceHex);
            }
            else
            {
                printf("[%d/%d] sent: parent (%u,%u), anchor %u, %s, nonce %s\n",
                    c + 1, count, parentTick, parentIndex, anchorTick, verdict, nonceHex);
            }
            for (int r = 1; r < repeat; r++)
            {
                const bool resent = submitAntSolution(sock, signingSubseed, signingPrivateKey, signingPublicKey,
                    computorPublicKey, parentTick, parentIndex, anchorTick, claimedScore, nonce);
                printf("[%d/%d] resend %d/%d %s\n", c + 1, count, r, repeat - 1, resent ? "sent" : "failed");
            }
        }
        else
        {
            printf("[%d/%d] send failed, reconnecting...\n", c + 1, count);
            reconnect(sock, nodeIp, nodePort);
        }

        if (intervalMs > 0 && c + 1 < count)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        }
    }

    // How many commits to wait for: the ones the local walk says the node accepts. With a forced
    // claim nothing is known in advance, so every send counts.
    const unsigned int awaited = claimForced ? sent : expectedAccepted;

    // The node publishes at most one queued solution per tick per MiningID and commits it one
    // settlement window later, so the counter keeps rising after the run. Wait until every send is
    // counted or the counter has not moved for two windows. A stalled network does not advance the
    // tick, so the wait holds.
    static const unsigned int SETTLE_POLL_MS = 1000;
    static const unsigned int SETTLE_PROGRESS_MS = 10000;
    static const unsigned int SETTLE_IDLE_TICKS = 2U * ANT_SETTLEMENT_WINDOW_TICKS;
    RespondAntEpochContext ctxAfter;
    bool haveCtxAfter = false;
    unsigned int waitedMs = 0;
    bool tracking = false;
    bool settledIdle = false;
    unsigned int lastCount = 0;
    unsigned int lastGrowthTick = 0;
    while (true)
    {
        // A fresh connection has no broadcast backlog in front of the reply.
        sock.closeConnection();
        haveCtxAfter = sock.establishConnection((char*)nodeIp, nodePort)
            && queryEpochContext(sock, ctxAfter);
        RespondCurrentTickInfo tickInfo;
        const bool haveTick = haveCtxAfter && queryCurrentTickInfo(sock, tickInfo);
        if (!haveCtxAfter || !haveCtxBefore || awaited == 0 || !haveTick)
        {
            break;
        }
        if (ctxAfter.epoch != ctxBefore.epoch)
        {
            break;
        }
        const unsigned int grewSoFar = (ctxAfter.solutionCount >= ctxBefore.solutionCount)
            ? (ctxAfter.solutionCount - ctxBefore.solutionCount) : 0U;
        if (grewSoFar >= awaited)
        {
            break;
        }
        if (!tracking || ctxAfter.solutionCount != lastCount)
        {
            tracking = true;
            lastCount = ctxAfter.solutionCount;
            lastGrowthTick = tickInfo.tick;
        }
        else if (tickInfo.tick >= lastGrowthTick + SETTLE_IDLE_TICKS)
        {
            settledIdle = true;
            break;
        }
        if (waitedMs > 0 && waitedMs % SETTLE_PROGRESS_MS == 0)
        {
            printf("  waiting: %u of %u committed, tick %u, no change for %u ticks\n",
                grewSoFar, awaited, tickInfo.tick,
                (tickInfo.tick >= lastGrowthTick) ? (tickInfo.tick - lastGrowthTick) : 0U);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_POLL_MS));
        waitedMs += SETTLE_POLL_MS;
    }
    sock.closeConnection();

    printf("Done. %u/%d ant solutions sent.\n", sent, count);
    if (!claimForced)
    {
        printf("  local walk: %u should be accepted, %u should be dropped by the node's gates, %u gave no score and were not sent\n",
            expectedAccepted, sent - expectedAccepted, notScored);
    }
    if (haveCtxBefore && haveCtxAfter)
    {
        printEpochContext("  after: ", ctxAfter);
        if (ctxAfter.epoch != ctxBefore.epoch)
        {
            printf("  epoch changed %u -> %u during the run; the counter restarted, so this run"
                   " cannot be measured by it\n", ctxBefore.epoch, ctxAfter.epoch);
            return 0;
        }
        // Network-wide counter: other miners this epoch are counted too, so this is a floor.
        const unsigned int grew = (ctxAfter.solutionCount >= ctxBefore.solutionCount)
            ? (ctxAfter.solutionCount - ctxBefore.solutionCount) : 0U;
        printf("  tree grew by %u accepted solutions while %u were sent and %u awaited (waited %u ms for"
               " them to commit)\n", grew, sent, awaited, waitedMs);
        if (grew < awaited && !settledIdle)
        {
            printf("  %u of %u had not committed when the node stopped answering - read the counter"
                   " again with -getantepochcontext\n", awaited - grew, awaited);
        }
        else if (grew == 0 && awaited > 0)
        {
            printf("  nothing was accepted - the node's F3 'AntPool:' line counts the drops by reason\n");
        }
        else if (grew < awaited)
        {
            printf("  %u of %u did not commit: the counter stopped moving for %u ticks, so the node"
                   " dropped or rejected them - its F3 'AntPool:' line counts the drops by reason\n",
                awaited - grew, awaited, SETTLE_IDLE_TICKS);
        }
    }
    return 0;
}
