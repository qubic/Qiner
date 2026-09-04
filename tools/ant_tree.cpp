// ant_tree - operator tool: query one identity's ant-colony tree from a node and print it.
//
// The identity-tree query (REQUEST_ANT_IDENTITY_TREE) is OPERATOR-SIGNED: the node verifies
// sign(K12(payload)) against its configured operator public key, so this tool needs the node
// operator's 55-char seed. That is by design - it keeps tree reads off the public attack surface
// (pools run the node + seed and route their sub-miners' solutions).
//
// Build: added to Qiner/tools/CMakeLists.txt; reuses src/network.h, src/keyUtils.*, src/K12AndKeyUtil.h.
//
// Usage:
//   ant_tree <IDENTITY|HEXPUBKEY> --seed <55-char operator seed> [--node HOST:PORT] [--dot] [--max-depth N]
//   ant_tree <IDENTITY> --seed-file <path> --node 10.0.0.5:31841
//
// The seed comes from --seed (or --seed-file). Same seed the miner takes on its command line.

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>
#include <functional>

#include "keyUtils.h"
#include "network.h"
#include "K12AndKeyUtil.h"
#include "ant_colony_message.h"

using Ref = std::pair<unsigned int, unsigned int>;

// Read framed messages, skipping unsolicited ones, until wantedType arrives. Mirrors AntMiner.cpp.
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

static bool sendUnsigned(ServerSocket& sock, unsigned char type, const void* payload, unsigned int payloadSize)
{
    struct
    {
        RequestResponseHeader header;
        char payload[64];
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

// Operator-signed identity-tree request: payload + sign(subseed, K12(payload)).
static bool sendSignedTreeRequest(ServerSocket& sock, const unsigned char* subseed, const unsigned char* operatorPublicKey,
    const unsigned char* queryPubkey, unsigned int fromIndex)
{
    struct
    {
        RequestResponseHeader header;
        unsigned char payload[sizeof(RequestAntIdentityTree)];
        unsigned char signature[SIGNATURE_SIZE];
    } packet;
    packet.header.setSize(sizeof(packet));
    packet.header.randomizeDejavu();
    packet.header.setType(REQUEST_ANT_IDENTITY_TREE);

    unsigned int padding = 0;
    memcpy(packet.payload, queryPubkey, 32);
    memcpy(packet.payload + 32, &fromIndex, sizeof(fromIndex));
    memcpy(packet.payload + 36, &padding, sizeof(padding));

    unsigned char digest[32];
    KangarooTwelve(packet.payload, sizeof(packet.payload), digest, sizeof(digest));
    // FourQ encode() writes the signature with an aligned 32-byte store; sign into a 32-byte-aligned
    // buffer, then copy into the (possibly unaligned) packet field.
    alignas(32) unsigned char sig[64];
    sign(subseed, operatorPublicKey, digest, sig);
    memcpy(packet.signature, sig, 64);

    return sock.sendData((char*)&packet, sizeof(packet));
}

static bool queryEpochContext(ServerSocket& sock, RespondAntEpochContext& out)
{
    if (!sendUnsigned(sock, REQUEST_ANT_EPOCH_CONTEXT, nullptr, 0))
    {
        return false;
    }
    return waitForResponse(sock, RESPOND_ANT_EPOCH_CONTEXT, (char*)&out, sizeof(out)) == (int)sizeof(out);
}

static bool queryTree(ServerSocket& sock, const unsigned char* subseed, const unsigned char* operatorPublicKey,
    const unsigned char* queryPubkey, std::vector<AntIdentityTreeNode>& out)
{
    static char buffer[sizeof(RespondAntIdentityTreeHeader) + 256 * sizeof(AntIdentityTreeNode)];
    unsigned int fromIndex = 0;
    for (;;)
    {
        if (!sendSignedTreeRequest(sock, subseed, operatorPublicKey, queryPubkey, fromIndex))
        {
            return false;
        }
        int received = waitForResponse(sock, RESPOND_ANT_IDENTITY_TREE, buffer, sizeof(buffer));
        if (received < (int)sizeof(RespondAntIdentityTreeHeader))
        {
            return false;
        }
        const RespondAntIdentityTreeHeader* header = (const RespondAntIdentityTreeHeader*)buffer;
        if (header->itemSize != sizeof(AntIdentityTreeNode))
        {
            printf("node itemSize %u != %u - wire structs out of sync\n", header->itemSize, (unsigned int)sizeof(AntIdentityTreeNode));
            return false;
        }
        const AntIdentityTreeNode* nodes = (const AntIdentityTreeNode*)(buffer + sizeof(RespondAntIdentityTreeHeader));
        for (unsigned int i = 0; i < header->count; i++)
        {
            out.push_back(nodes[i]);
        }
        if (header->nextIndex == 0)
        {
            return true;
        }
        fromIndex = header->nextIndex;
    }
}

static bool hexToPubkey(const char* text, unsigned char* pubkey)
{
    if (strlen(text) != 64)
    {
        return false;
    }
    for (int i = 0; i < 32; i++)
    {
        unsigned int byte = 0;
        if (sscanf(text + i * 2, "%2x", &byte) != 1)
        {
            return false;
        }
        pubkey[i] = (unsigned char)byte;
    }
    return true;
}

static int findBest(const std::vector<AntIdentityTreeNode>& nodes)
{
    int best = -1;
    for (int i = 0; i < (int)nodes.size(); i++)
    {
        if (best < 0 || nodes[i].score < nodes[best].score)
        {
            best = i;
        }
    }
    return best;
}

static std::map<Ref, std::vector<int>> buildChildren(const std::vector<AntIdentityTreeNode>& nodes)
{
    std::map<Ref, std::vector<int>> children;
    for (int i = 0; i < (int)nodes.size(); i++)
    {
        children[Ref(nodes[i].parentTick, nodes[i].parentSolutionIndexInTick)].push_back(i);
    }
    for (auto& entry : children)
    {
        std::sort(entry.second.begin(), entry.second.end(),
            [&](int a, int b) { return nodes[a].score < nodes[b].score; });  // best (lowest error) first
    }
    return children;
}

// First 8 bytes of a digest as hex - enough to eyeball agreement without flooding the line.
static std::string hexPrefix(const unsigned char* digest)
{
    char text[17];
    for (int i = 0; i < 8; i++)
    {
        snprintf(text + i * 2, 3, "%02x", digest[i]);
    }
    return std::string(text);
}

static void printAscii(const std::vector<AntIdentityTreeNode>& nodes, const RespondAntEpochContext* ctx,
    const char* ident, int maxDepth)
{
    const int best = findBest(nodes);
    const std::map<Ref, std::vector<int>> children = buildChildren(nodes);
    const unsigned int cap = ctx ? ctx->maxChildrenPerParent : 0;

    printf("identity %.8s...  nodes: %u", ident, (unsigned int)nodes.size());
    if (best >= 0)
    {
        printf("  best: %u (depth %u)", nodes[best].score, nodes[best].depth);
    }
    printf("\n");
    if (ctx)
    {
        printf("epoch %u  threshold=%u  freshness=%u  child-cap=%s  tree=%u  free-ann=%u\n",
            ctx->epoch, ctx->threshold, ctx->freshnessWindow,
            cap == 0 ? "unbound" : std::to_string(cap).c_str(),
            ctx->solutionCount, ctx->freeAnnSlotsCount);
        // Must match the miners' .task file, or their scores will not match the node's.
        printf("spectrum=%s  topology=%s  data=%s\n",
            hexPrefix(ctx->spectrumDigest).c_str(),
            hexPrefix(ctx->topologyHash).c_str(),
            hexPrefix(ctx->dataHash).c_str());
    }
    printf("root  depth 0\n");

    std::function<void(Ref, int)> walk = [&](Ref ref, int indent)
    {
        auto it = children.find(ref);
        if (it == children.end())
        {
            return;
        }
        for (int idx : it->second)
        {
            const AntIdentityTreeNode& n = nodes[idx];
            if (maxDepth && (int)n.depth > maxDepth)
            {
                continue;
            }
            for (int s = 0; s < indent; s++)
            {
                printf("  ");
            }
            // From the listing's own edges: n.childCount is counted only up to the cap, so it
            // reads 0 for every node while the cap is unbound.
            const auto kidsIt = children.find(Ref(n.selfTick, n.selfSolutionIndexInTick));
            const unsigned int kids = (kidsIt == children.end()) ? 0u : (unsigned int)kidsIt->second.size();
            printf("[%u] d%u kids=%u anchor=%u%s%s\n", n.score, n.depth, kids, n.anchorTick,
                (cap && kids >= cap) ? "  FULL" : "",
                (idx == best) ? "  * best" : "");
            walk(Ref(n.selfTick, n.selfSolutionIndexInTick), indent + 1);
        }
    };
    walk(Ref(ROOT_TICK, ROOT_INDEX_IN_TICK), 1);
}

static void printDot(const std::vector<AntIdentityTreeNode>& nodes, const char* ident)
{
    const int best = findBest(nodes);
    unsigned int lo = 0xFFFFFFFFu, hi = 0;
    for (const AntIdentityTreeNode& n : nodes)
    {
        lo = n.score < lo ? n.score : lo;
        hi = n.score > hi ? n.score : hi;
    }
    printf("digraph ant_tree {\n");
    printf("  rankdir=TB; node [shape=box, style=filled, fontname=\"monospace\"];\n");
    printf("  root [label=\"root\\n%.6s..\", shape=oval, style=dashed];\n", ident);
    for (int i = 0; i < (int)nodes.size(); i++)
    {
        const AntIdentityTreeNode& n = nodes[i];
        double t = (hi == lo) ? 0.0 : (double)(n.score - lo) / (double)(hi - lo);  // 0 best .. 1 worst
        int r = (int)(0x30 + t * (0xE0 - 0x30));
        int g = (int)(0x90 + t * (0xF0 - 0x90));
        int b = (int)(0x30 + t * (0xE0 - 0x30));
        printf("  \"%u_%u\" [label=\"%u%s\\nd%u k%u\", fillcolor=\"#%02x%02x%02x\"];\n",
            n.selfTick, n.selfSolutionIndexInTick, n.score, (i == best) ? " *" : "", n.depth, n.childCount, r, g, b);
    }
    for (const AntIdentityTreeNode& n : nodes)
    {
        if (n.parentTick == ROOT_TICK && n.parentSolutionIndexInTick == ROOT_INDEX_IN_TICK)
        {
            printf("  root -> \"%u_%u\";\n", n.selfTick, n.selfSolutionIndexInTick);
        }
        else
        {
            printf("  \"%u_%u\" -> \"%u_%u\";\n", n.parentTick, n.parentSolutionIndexInTick,
                n.selfTick, n.selfSolutionIndexInTick);
        }
    }
    printf("}\n");
}

static bool readSeed(const char* seedArg, const char* seedFile, char* seedOut /* >= 56 */)
{
    if (seedArg && strlen(seedArg) >= 55)
    {
        memcpy(seedOut, seedArg, 55);
        seedOut[55] = 0;
        return true;
    }
    if (seedFile)
    {
        FILE* f = fopen(seedFile, "rb");
        if (!f)
        {
            return false;
        }
        size_t got = fread(seedOut, 1, 55, f);
        fclose(f);
        if (got == 55)
        {
            seedOut[55] = 0;
            return true;
        }
    }
    return false;
}

int main(int argc, char* argv[])
{
    const char* identity = nullptr;
    const char* node = "127.0.0.1:21841";
    const char* seedArg = nullptr;
    const char* seedFile = nullptr;
    int maxDepth = 0;
    bool dot = false;

    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "--node") && i + 1 < argc) { node = argv[++i]; }
        else if (!strcmp(argv[i], "--seed") && i + 1 < argc) { seedArg = argv[++i]; }
        else if (!strcmp(argv[i], "--seed-file") && i + 1 < argc) { seedFile = argv[++i]; }
        else if (!strcmp(argv[i], "--max-depth") && i + 1 < argc) { maxDepth = atoi(argv[++i]); }
        else if (!strcmp(argv[i], "--dot")) { dot = true; }
        else if (argv[i][0] != '-') { identity = argv[i]; }
    }
    if (!identity)
    {
        fprintf(stderr, "usage: ant_tree <IDENTITY|HEXPUBKEY> --seed <55-char operator seed> [--node HOST:PORT] [--seed-file PATH] [--dot] [--max-depth N]\n");
        return 1;
    }

    char seed[56];
    if (!readSeed(seedArg, seedFile, seed))
    {
        fprintf(stderr, "error: need the node operator's 55-char seed via --seed or --seed-file (the tree query is operator-signed)\n");
        return 1;
    }

    unsigned char subseed[32], privateKey[32], operatorPublicKey[32];
    if (!getSubseedFromSeed((const unsigned char*)seed, subseed))
    {
        fprintf(stderr, "error: seed must be 55 lowercase letters a-z\n");
        return 1;
    }
    getPrivateKeyFromSubSeed(subseed, privateKey);
    getPublicKeyFromPrivateKey(privateKey, operatorPublicKey);

    unsigned char queryPubkey[32];
    if (!hexToPubkey(identity, queryPubkey))
    {
        getPublicKeyFromIdentity(identity, queryPubkey);  // 60-char A-Z identity
    }

    char host[128];
    int port = 21841;
    const char* colon = strchr(node, ':');
    if (colon)
    {
        size_t hostLen = (size_t)(colon - node);
        if (hostLen >= sizeof(host)) { hostLen = sizeof(host) - 1; }
        memcpy(host, node, hostLen);
        host[hostLen] = 0;
        port = atoi(colon + 1);
    }
    else
    {
        strncpy(host, node, sizeof(host) - 1);
        host[sizeof(host) - 1] = 0;
    }

    ServerSocket sock;
    if (!sock.establishConnection(host, port))
    {
        fprintf(stderr, "error: could not connect to %s:%d\n", host, port);
        return 1;
    }

    RespondAntEpochContext ctx;
    bool haveCtx = queryEpochContext(sock, ctx);

    std::vector<AntIdentityTreeNode> nodes;
    bool ok = queryTree(sock, subseed, operatorPublicKey, queryPubkey, nodes);
    sock.closeConnection();

    if (!ok)
    {
        fprintf(stderr, "error: tree query failed (rejected signature, wrong node, or no such identity)\n");
        return 1;
    }

    if (dot)
    {
        printDot(nodes, identity);
    }
    else
    {
        printAscii(nodes, haveCtx ? &ctx : nullptr, identity, maxDepth);
    }
    return 0;
}
