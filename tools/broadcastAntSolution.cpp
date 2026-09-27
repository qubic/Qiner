// broadcastAntSolution: standalone dummy ant-solution submitter (no scoring). Sibling of
// tools/broadcastMessageSolution.cpp, but for the ant-colony pipeline.
//
// Canonical-nonce depth-1 (ROOT) children. Under a permissive epoch threshold these are
// accepted, so the tree grows - exercising accepts, deposits, ranking and multi-node consensus.
//
// The node still recomputes the real score; the claimed score is 0 (the receive-side claim check is
// disabled for the testnet). Every gate other than the claim check still applies, so only genuinely
// valid solutions are accepted.
//
// The run reads the epoch context before and after, so it reports how many solutions the node
// accepted rather than only how many were sent. That counter is network-wide, so with other miners
// active it is a floor on this run's contribution.
//
// Usage:
//   broadcastAntSolution <Node IP> <Node Port> <MiningID> <Signing Seed> [count=1] [intervalMs=0] [-operator <Operator Seed>]
//     MiningID   : the computor the solution is FOR - tree owner, deposit payer, broadcast destination.
//     Signing Seed  : a computor/funded seed - the broadcast SOURCE that signs the solution. Its identity
//                    may be the MiningID (submit-for-self) or a different one (submit-for-other).
//     -operator S  : the node operator's seed. Signs the operator-only tree read so children can extend
//                    real nodes (depth). Omit for a ROOT-only depth-1 flood. Admin credential - pass it
//                    only when you want depth.
//     count        : number of solutions to send (default 1; use a large number to flood)
//     intervalMs   : delay between sends (default 0)

#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>

#ifdef _MSC_VER
#include <intrin.h>
#else
#include <immintrin.h>
#endif

#include "keyUtils.h"
#include "K12AndKeyUtil.h"
#include "network.h"
#include "ant_colony_message.h"

// bpp9000 canonical-nonce knobs (core src/mining/score_bpp9000.h):
// nonce[0] == 1 selects Bpp9000, nonce[1] = L in [1, 10] (bits 0-3) + mode in [1, 3] (bits 4-5),
// nonce[2] = K, the explore-step count. isCanonicalAntNonce accepts K <= BPP9000_NUMBER_OF_MUTATIONS,
// which is 1000, so every value a byte can hold is canonical and K needs no masking. K does not change
// what a solution costs to verify: the walk always runs all 1000 steps, K only decides how many of them
// use the explore rule instead of the exploit rule.
static constexpr unsigned char ALGO_BPP9000 = 1;
static constexpr unsigned int MAX_CHANGES_PER_STEP = 10;

// --- request/response helpers (from src/AntMiner.cpp) ---
static int waitForResponse(ServerSocket& sock, unsigned char wantedType, char* payload, unsigned int payloadCapacity)
{
    static char scratch[1024 * 1024];
    // A busy node streams ticks and votes to every connected peer, so the reply can sit behind a
    // long run of unrelated broadcasts. Bound the skip by wall clock, not by a message count.
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

// Public (unsigned) read of the epoch's mining parameters. Worth doing before a run: if the node is
// not on the expected threshold or freshness window, every solution below will be rejected and the
// send loop cannot tell the difference from a send that simply never landed.
static bool queryEpochContext(ServerSocket& sock, RespondAntEpochContext& out)
{
    if (!sendRequest(sock, REQUEST_ANT_EPOCH_CONTEXT, NULL, 0))
    {
        return false;
    }
    return waitForResponse(sock, RESPOND_ANT_EPOCH_CONTEXT, (char*)&out, sizeof(out)) == (int)sizeof(out);
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

int main(int argc, char* argv[])
{
    if (argc < 5)
    {
        printf("Usage: broadcastAntSolution <Node IP> <Node Port> <MiningID> <Signing Seed> [count=1] [intervalMs=0] [-operator <Operator Seed>]\n");
        printf("  Signing Seed:  a computor/funded seed - the broadcast SOURCE that signs the solution.\n");
        printf("  MiningID:   the computor the solution is FOR (tree owner, deposit payer).\n");
        printf("  -operator S:  operator seed; signs the tree read so children extend real nodes (depth). Omit = ROOT-only.\n");
        return 1;
    }

    const char* nodeIp = argv[1];
    const int nodePort = std::atoi(argv[2]);
    const char* miningID = argv[3];
    const char* signingSeed = argv[4];

    int count = 1;
    int intervalMs = 0;
    bool countSet = false;
    const char* operatorSeed = nullptr;   // signs the tree read (depth); NULL -> ROOT-only
    for (int i = 5; i < argc; i++)
    {
        if (strcmp(argv[i], "-operator") == 0 && i + 1 < argc)
        {
            operatorSeed = argv[++i];
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

    printf("broadcastAntSolution -> %s:%d, computor %s, canonical (accept), depth %s, count %d, interval %d ms\n",
           nodeIp, nodePort, miningID,
           operatorSeed ? "on" : "ROOT-only", count, intervalMs);

    ServerSocket sock;
    if (!sock.establishConnection((char*)nodeIp, nodePort))
    {
        printf("Failed to connect to %s:%d\n", nodeIp, nodePort);
        return 1;
    }

    // Baseline before sending, so the run can report how many of its solutions the node actually
    // accepted rather than only how many were put on the wire.
    RespondAntEpochContext ctxBefore;
    const bool haveCtxBefore = queryEpochContext(sock, ctxBefore);
    if (haveCtxBefore)
    {
        printEpochContext("  before:", ctxBefore);
    }
    else
    {
        printf("  before: epoch context unavailable - acceptance cannot be reported\n");
    }

    unsigned int sent = 0;
    std::vector<AntIdentityTreeNode> listing;   // this identity's accepted nodes (for deeper extension)
    for (int c = 0; c < count; c++)
    {
        RespondCurrentTickInfo tickInfo;
        if (!queryCurrentTickInfo(sock, tickInfo) || tickInfo.tick <= tickInfo.initialTick)
        {
            printf("[%d/%d] tick query failed, reconnecting...\n", c + 1, count);
            sock.closeConnection();
            while (!sock.establishConnection((char*)nodeIp, nodePort))
            {
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            c--;
            continue;
        }
        const unsigned int anchorTick = tickInfo.tick - 1U;

        // Every 16 sends, refresh the tree so we can extend real nodes (deeper), not just ROOT.
        // Then extend a random listed node ~70% of the time, ROOT the rest (to seed new depth-1 nodes).
        // ROOT-only when the tree is empty or the signed read is not accepted.
        if (operatorSeed != nullptr && (c % 16 == 0))
        {
            refreshListing(sock, operatorSubseed, operatorPublicKey, computorPublicKey, listing);
        }
        unsigned int parentTick = ROOT_TICK;
        unsigned int parentIndex = ROOT_INDEX_IN_TICK;
        if (!listing.empty())
        {
            unsigned long long roll = 0;
            _rdrand64_step(&roll);
            if ((roll % 100U) >= 30U)
            {
                const AntIdentityTreeNode& p = listing[(roll >> 8) % listing.size()];
                parentTick = p.selfTick;
                parentIndex = p.selfSolutionIndexInTick;
            }
        }

        unsigned char nonce[32];
        fillRandomNonce(nonce);
        nonce[0] = ALGO_BPP9000;
        const unsigned char L = (unsigned char)((nonce[1] % MAX_CHANGES_PER_STEP) + 1);
        const unsigned char mode = (unsigned char)(((nonce[1] >> 4) % 3) + 1);
        nonce[1] = (unsigned char)(L | (mode << 4));
        // nonce[2] is K and every byte is canonical, so the random byte stands.

        char nonceHex[65];
        for (int i = 0; i < 32; i++)
        {
            snprintf(nonceHex + i * 2, 3, "%02x", nonce[i]);
        }

        if (submitAntSolution(sock, signingSubseed, signingPrivateKey, signingPublicKey, computorPublicKey,
                parentTick, parentIndex, anchorTick, 0, nonce))
        {
            sent++;
            if (parentIndex == ROOT_INDEX_IN_TICK)
            {
                printf("[%d/%d] sent: parent ROOT, anchor %u, nonce %s\n", c + 1, count, anchorTick, nonceHex);
            }
            else
            {
                printf("[%d/%d] sent: parent (%u,%u), anchor %u, nonce %s\n",
                    c + 1, count, parentTick, parentIndex, anchorTick, nonceHex);
            }
        }
        else
        {
            printf("[%d/%d] send failed, reconnecting...\n", c + 1, count);
            sock.closeConnection();
            while (!sock.establishConnection((char*)nodeIp, nodePort))
            {
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }

        if (intervalMs > 0 && c + 1 < count)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        }
    }

    // A solution is not committed when it arrives: the node publishes it into a later tick and
    // commits it only once that tick is processed, so the counter lags the send by several ticks.
    static const unsigned int SETTLE_POLL_MS = 1000;
    static const unsigned int SETTLE_TIMEOUT_MS = 60000;
    RespondAntEpochContext ctxAfter;
    bool haveCtxAfter = false;
    unsigned int waitedMs = 0;
    while (true)
    {
        // A socket collects the node's tick and vote broadcasts for as long as it is open, so a
        // reply on it sits behind that backlog. Read the counter on one that has none.
        sock.closeConnection();
        haveCtxAfter = sock.establishConnection((char*)nodeIp, nodePort)
            && queryEpochContext(sock, ctxAfter);
        if (!haveCtxAfter || !haveCtxBefore || sent == 0)
        {
            break;
        }
        if (ctxAfter.epoch != ctxBefore.epoch)
        {
            break;
        }
        if (ctxAfter.solutionCount >= ctxBefore.solutionCount
            && (ctxAfter.solutionCount - ctxBefore.solutionCount) >= sent)
        {
            break;
        }
        if (waitedMs >= SETTLE_TIMEOUT_MS)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_POLL_MS));
        waitedMs += SETTLE_POLL_MS;
    }
    sock.closeConnection();

    printf("Done. %u/%d ant solutions sent.\n", sent, count);
    if (haveCtxBefore && haveCtxAfter)
    {
        printEpochContext("  after: ", ctxAfter);
        if (ctxAfter.epoch != ctxBefore.epoch)
        {
            printf("  epoch changed %u -> %u during the run; the counter restarted, so this run"
                   " cannot be measured by it\n", ctxBefore.epoch, ctxAfter.epoch);
            return 0;
        }
        // The counter is network-wide, so anything else mining this epoch is counted here too; it is a
        // floor on what this run achieved, not an exact attribution.
        const unsigned int grew = (ctxAfter.solutionCount >= ctxBefore.solutionCount)
            ? (ctxAfter.solutionCount - ctxBefore.solutionCount) : 0U;
        printf("  tree grew by %u accepted solutions while %u were sent (waited %u ms for them to"
               " commit)\n", grew, sent, waitedMs);
        if (grew == 0 && sent > 0)
        {
            printf("  nothing was accepted - the node log says why on its '[ant-colony] pool drop'"
                   " lines; 'unacceptable' just means the nonce missed the threshold above, which is"
                   " normal for a small batch\n");
        }
        else if (grew < sent)
        {
            printf("  %u of %u had not committed within %u ms - re-run to read the counter again,"
                   " or check the node log for '[ant-colony] pool drop'\n",
                sent - grew, sent, SETTLE_TIMEOUT_MS);
        }
    }
    return 0;
}
