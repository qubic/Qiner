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
// nonce[0] == 1 selects Bpp9000, nonce[1] = L in [1, 10], nonce[2] = K in [0, 100] for ant.
static constexpr unsigned char ALGO_BPP9000 = 1;
static constexpr unsigned int MAX_LUT_ENTRIES_PER_STEP = 10;
static constexpr unsigned int NUMBER_OF_MUTATIONS = 100;

// --- request/response helpers (from src/AntMiner.cpp) ---
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
        nonce[1] = (unsigned char)((nonce[1] % MAX_LUT_ENTRIES_PER_STEP) + 1);
        nonce[2] = (unsigned char)(nonce[2] % (NUMBER_OF_MUTATIONS + 1));

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

    sock.closeConnection();
    printf("Done. %u/%d ant solutions sent.\n", sent, count);
    return 0;
}
