// Golden regression for the bpp9000 scorer. Pins computeScore against known-good vectors so the
// scorer's integer math cannot silently change - the guard for consolidating the two Qiner scorers
// (the ant-colony seam is merged into this scorer; identical numbers before and after prove the
// port preserved the math).
//
// Two pins, both threaded (the scalar production walk is ~138 s per sample):
//   1. Solo computeScore vs operator ground truth - data/gt_production.csv, task = bpp9000.task, the
//      canonical task pinned by BPP9000_TOPOLOGY_HASH / BPP9000_DATA_HASH in core/src/public_settings.h.
//   2. The ant-colony seam - deriveRootANN + computeScoreFromParent - vs goldens captured from the
//      reference scorer the live testnet proved node-exact (the guard for the ported seam).
//
// computeScore clamps L = nonce[1] to [1, MAX_LUT_ENTRIES_PER_STEP], forces K = 0, and ignores
// nonce[0]. That is exactly the "bound into range, no canonical check" path the ground-truth
// generator used, so the raw gt nonces feed in unchanged - no per-nonce adjustment.
//
// GT_MAX_ROWS_PER_SEED (top of file) caps rows scored per mining seed; 0 = all, lower it for a quick run.

#include <catch2/catch.hpp>

#include "score_bpp9000.h"

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifndef QINER_TEST_DATA_DIR
#define QINER_TEST_DATA_DIR "data"
#endif

// Hardware_concurrency and the sample count clamp it further.
constexpr unsigned int MAX_TEST_THREADS = 16;

// Ground-truth rows scored per mining seed (0 = all rows in the batch)
constexpr size_t GT_MAX_ROWS_PER_SEED = 0;

std::string dataPath(const char* name)
{
    return std::string(QINER_TEST_DATA_DIR) + "/" + name;
}

int hexVal(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }
    return -1;
}

// Parse exactly 32 bytes from a 64-hex-char field; surrounding whitespace is ignored.
bool parseHex32(const std::string& field, unsigned char out[32])
{
    std::string s;
    for (const char c : field)
    {
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n')
        {
            s.push_back(c);
        }
    }
    if (s.size() != 64)
    {
        return false;
    }
    for (int i = 0; i < 32; ++i)
    {
        const int hi = hexVal(s[2 * i]);
        const int lo = hexVal(s[2 * i + 1]);
        if (hi < 0 || lo < 0)
        {
            return false;
        }
        out[i] = (unsigned char)((hi << 4) | lo);
    }
    return true;
}

struct Sample
{
    unsigned char publicKey[32];
    unsigned char nonce[32];
    unsigned int score;
};

template <typename MinerT>
void runGolden(const char* label, const std::string& taskPath, const unsigned char seed[32],
               const std::vector<Sample>& samples)
{
    REQUIRE_FALSE(samples.empty());

    unsigned int numThreads = std::thread::hardware_concurrency();
    if (numThreads < 1)
    {
        numThreads = 1;
    }
    if (numThreads > MAX_TEST_THREADS)
    {
        numThreads = MAX_TEST_THREADS;
    }
    if (numThreads > samples.size())
    {
        numThreads = (unsigned int)samples.size();
    }

    std::mutex failMutex;
    std::vector<std::string> failures;
    std::atomic<bool> loadFailed(false);

    const auto worker = [&](unsigned int threadIdx)
    {
        std::unique_ptr<MinerT> miner(new MinerT());
        unsigned char localSeed[32];
        std::memcpy(localSeed, seed, 32);
        if (!miner->initialize(localSeed, taskPath.c_str()))
        {
            loadFailed.store(true);
            return;
        }
        for (size_t i = threadIdx; i < samples.size(); i += numThreads)
        {
            unsigned char publicKey[32];
            unsigned char nonce[32];
            std::memcpy(publicKey, samples[i].publicKey, 32);
            std::memcpy(nonce, samples[i].nonce, 32);
            const unsigned int score = miner->computeScore(publicKey, nonce);
            if (score != samples[i].score)
            {
                std::stringstream ss;
                ss << label << " sample " << i << ": got " << score << ", expected " << samples[i].score;
                const std::lock_guard<std::mutex> lock(failMutex);
                failures.push_back(ss.str());
            }
        }
    };

    std::vector<std::thread> pool;
    for (unsigned int t = 0; t < numThreads; ++t)
    {
        pool.emplace_back(worker, t);
    }
    for (std::thread& th : pool)
    {
        th.join();
    }

    REQUIRE_FALSE(loadFailed.load());

    std::string report;
    for (const std::string& f : failures)
    {
        report += f;
        report += "\n";
    }
    INFO(report);
    CHECK(failures.empty());
}

// A batch of ground-truth rows sharing one mining seed. one pool build covers the whole batch.
struct SeedGroup
{
    unsigned char seed[32];
    std::vector<Sample> samples;
};

// gt_production.csv: "pubkey, nonce, miningseed, score", rows batched by mining seed
// (each seed is one ground-truth run appended into the file). Returns one SeedGroup per distinct
// seed, in first-seen order. maxPerSeed == 0 means all rows in each batch.
std::vector<SeedGroup> loadGroundTruth(const std::string& path, size_t maxPerSeed)
{
    std::ifstream in(path);
    REQUIRE(in.good());

    std::vector<SeedGroup> groups;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line))
    {
        if (line.find_first_not_of(" \t\r\n") == std::string::npos)
        {
            continue;
        }
        std::stringstream ss(line);
        std::string pubField;
        std::string nonceField;
        std::string seedField;
        std::string scoreField;
        std::getline(ss, pubField, ',');
        std::getline(ss, nonceField, ',');
        std::getline(ss, seedField, ',');
        std::getline(ss, scoreField, ',');

        Sample sample;
        REQUIRE(parseHex32(pubField, sample.publicKey));
        REQUIRE(parseHex32(nonceField, sample.nonce));
        sample.score = (unsigned int)std::strtoul(scoreField.c_str(), nullptr, 10);

        unsigned char rowSeed[32];
        REQUIRE(parseHex32(seedField, rowSeed));

        SeedGroup* group = nullptr;
        for (SeedGroup& candidate : groups)
        {
            if (std::memcmp(candidate.seed, rowSeed, 32) == 0)
            {
                group = &candidate;
                break;
            }
        }
        if (group == nullptr)
        {
            groups.emplace_back();
            std::memcpy(groups.back().seed, rowSeed, 32);
            group = &groups.back();
        }

        if (maxPerSeed != 0 && group->samples.size() >= maxPerSeed)
        {
            continue;
        }
        group->samples.push_back(sample);
    }
    REQUIRE_FALSE(groups.empty());
    return groups;
}

// Production config
using ProdMiner = score_bpp9000::Miner<18, 1, 24 * 365, 24 * 28, 100000, 3, 64, 100, 5400>;


TEST_CASE("bpp9000 production stanalone ground truth", "[bpp9000AntColony]")
{
    const std::string taskPath = dataPath("bpp9000.task");
    const size_t maxPerSeed = GT_MAX_ROWS_PER_SEED;
    const std::vector<SeedGroup> groups = loadGroundTruth(dataPath("gt_production.csv"), maxPerSeed);
    for (size_t g = 0; g < groups.size(); ++g)
    {
        const std::string label = "gt_production seed[" + std::to_string(g) + "]";
        runGolden<ProdMiner>(label.c_str(), taskPath, groups[g].seed, groups[g].samples);
    }
}

// Test the ant-colony seam - deriveRootANN and computeScoreFromParent
TEST_CASE("bpp9000 ant-colony seam", "[bpp9000AntColony]")
{
    const char* SEED_HEX = "8b12add89bc264e01038b61b784e0778a03cd3025e1faeedda6598f197ceccc0";
    const char* PUB_HEX[3] = {
        "039cc1f1560aa96daa994a2b296f22d7f2fc9503ce95321d0b8193079e5f93dc",
        "d4aaeaf020007d1349590d77e2f779ad48f9a6cf720d04ca6a65fbbad81dc050",
        "2cfd43630593e07902ff52948ce387aaf0e2bb0e11952f8a0c91a33672203bcc"};
    const char* ANCHOR_HEX[2] = {
        "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        "fedcba9876543210fedcba9876543210fedcba9876543210fedcba9876543210"};
    const char* ROOT_DIGEST_HEX[3] = {
        "6f782f3a0b53902e8a9b8d7eaad3d134ab8ef4c0d9d88dfb820144c2ce230100",
        "a5a63aeca47c07a083759e5ffb98d10bddad204ee3292315b4ba772c0bb6df27",
        "371dc2af0094a0ea99984edec4ea65c4f743d3150243f4c0857c651e7e021a64"};

    // computeScoreFromParent cases: pubkey / L (nonce[1]) / K (nonce[2]) / anchor, and the golden
    // score. Captured from the merged scorer; cases 0-1 reproduce the reference-captured 4119 / 4503.
    struct FromParentCase
    {
        int pubIdx;
        unsigned char L;
        unsigned char K;
        int anchorIdx;
        unsigned int expected;
    };
    const FromParentCase FROM_PARENT_CASES[8] = {
        {0, 5,  7,   0, 4119},
        {1, 5,  7,   0, 4503},
        {2, 5,  7,   0, 5235},
        {0, 1,  0,   0, 4233},
        {1, 10, 100, 0, 5879},
        {2, 3,  50,  1, 5469},
        {0, 8,  20,  1, 4019},
        {1, 2,  99,  0, 5879}};

    unsigned char seed[32];
    REQUIRE(parseHex32(SEED_HEX, seed));
    const std::string taskPath = dataPath("bpp9000.task");

    {
        std::unique_ptr<ProdMiner> miner(new ProdMiner());
        REQUIRE(miner->initialize(seed, taskPath.c_str()));
        for (int i = 0; i < 3; ++i)
        {
            unsigned char pub[32];
            REQUIRE(parseHex32(PUB_HEX[i], pub));
            ProdMiner::ANN root;
            std::memset(&root, 0, sizeof(root));
            miner->deriveRootANN(pub, root);
            unsigned char digest[32];
            KangarooTwelve(root.lut, (unsigned int)sizeof(root.lut), digest, 32);
            unsigned char expected[32];
            REQUIRE(parseHex32(ROOT_DIGEST_HEX[i], expected));
            INFO("deriveRootANN pub[" << i << "]");
            CHECK(std::memcmp(digest, expected, 32) == 0);
        }
    }

    // computeScoreFromParent: full scoring walk (~138 s each). One case per thread (8 <=
    // hardware_concurrency, so a single wave). Each nonce is built from (L, K) with a 0xab tail.
    {
        unsigned int got[8] = {0};
        std::atomic<bool> loadFailed(false);
        const auto worker = [&](int i)
        {
            std::unique_ptr<ProdMiner> miner(new ProdMiner());
            unsigned char localSeed[32];
            std::memcpy(localSeed, seed, 32);
            if (!miner->initialize(localSeed, taskPath.c_str()))
            {
                loadFailed.store(true);
                return;
            }
            const FromParentCase& c = FROM_PARENT_CASES[i];
            unsigned char pub[32];
            unsigned char anchor[32];
            if (!parseHex32(PUB_HEX[c.pubIdx], pub) || !parseHex32(ANCHOR_HEX[c.anchorIdx], anchor))
            {
                loadFailed.store(true);
                return;
            }
            unsigned char nonce[32];
            std::memset(nonce, 0xab, 32);
            nonce[0] = 1;        // AlgoType::Bpp9000
            nonce[1] = c.L;
            nonce[2] = c.K;

            ProdMiner::ANN root;
            std::memset(&root, 0, sizeof(root));
            miner->deriveRootANN(pub, root);
            got[i] = miner->computeScoreFromParent(root.lut, pub, nonce, anchor);
        };

        std::vector<std::thread> pool;
        for (int i = 0; i < 8; ++i)
        {
            pool.emplace_back(worker, i);
        }
        for (std::thread& th : pool)
        {
            th.join();
        }

        REQUIRE_FALSE(loadFailed.load());
        for (int i = 0; i < 8; ++i)
        {
            const FromParentCase& c = FROM_PARENT_CASES[i];
            INFO("computeScoreFromParent case " << i << " (pub[" << c.pubIdx << "] L=" << (int)c.L
                 << " K=" << (int)c.K << " anchor[" << c.anchorIdx << "]): got " << got[i]
                 << ", expected " << c.expected);
            CHECK(got[i] == c.expected);
        }
    }
}

// --- ant-colony chain replay (gt_ant_production.csv) --------------------------------------------
// Data-driven depth-N seam pin, generated by tools/bpp9000_ground_truth.cpp --ant. Each chain is a
// lineage: level 0 extends the derived root, level i extends level i-1's bestANN. The test replays
// every chain and CHECKs each node's computeScoreFromParent against the recorded score.

struct AntNode
{
    unsigned char nonce[32];
    unsigned char anchor[32];
    unsigned int score;
};

// One lineage: pubkey + seed (constant per chain), nodes indexed by depth (0 = root-child).
struct AntChainData
{
    unsigned char pubkey[32];
    unsigned char seed[32];
    std::vector<AntNode> nodes;
};

// gt_ant_production.csv: "chain, depth, pubkey, nonce, anchor, seed, score". Groups rows by chain id
// (first-seen order) and places each node at its depth, so row order in the file does not matter.
std::vector<AntChainData> loadAntChains(const std::string& path)
{
    std::ifstream in(path);
    REQUIRE(in.good());

    std::vector<int> chainIds;
    std::vector<AntChainData> chains;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line))
    {
        if (line.find_first_not_of(" \t\r\n") == std::string::npos)
        {
            continue;
        }
        std::stringstream ss(line);
        std::string chainField;
        std::string depthField;
        std::string pubField;
        std::string nonceField;
        std::string anchorField;
        std::string seedField;
        std::string scoreField;
        std::getline(ss, chainField, ',');
        std::getline(ss, depthField, ',');
        std::getline(ss, pubField, ',');
        std::getline(ss, nonceField, ',');
        std::getline(ss, anchorField, ',');
        std::getline(ss, seedField, ',');
        std::getline(ss, scoreField, ',');

        const int chainId = std::atoi(chainField.c_str());
        const int depth = std::atoi(depthField.c_str());
        REQUIRE(depth >= 0);

        AntNode node;
        REQUIRE(parseHex32(nonceField, node.nonce));
        REQUIRE(parseHex32(anchorField, node.anchor));
        node.score = (unsigned int)std::strtoul(scoreField.c_str(), nullptr, 10);

        unsigned char pub[32];
        unsigned char seed[32];
        REQUIRE(parseHex32(pubField, pub));
        REQUIRE(parseHex32(seedField, seed));

        size_t idx = chains.size();
        for (size_t k = 0; k < chainIds.size(); ++k)
        {
            if (chainIds[k] == chainId)
            {
                idx = k;
                break;
            }
        }
        if (idx == chains.size())
        {
            chainIds.push_back(chainId);
            AntChainData created;
            std::memcpy(created.pubkey, pub, 32);
            std::memcpy(created.seed, seed, 32);
            chains.push_back(created);
        }

        AntChainData& chain = chains[idx];
        if ((size_t)depth >= chain.nodes.size())
        {
            chain.nodes.resize((size_t)depth + 1);
        }
        chain.nodes[(size_t)depth] = node;
    }
    REQUIRE_FALSE(chains.empty());
    return chains;
}

// Chains scored per run (0 = all). Depth is fixed by the file; this caps breadth for a quick run.
constexpr size_t ANT_MAX_CHAINS = 0;

TEST_CASE("bpp9000 ant-colony chain replay (gt_ant_production.csv)", "[bpp9000AntColony]")
{
    const std::string taskPath = dataPath("bpp9000.task");
    std::vector<AntChainData> chains = loadAntChains(dataPath("gt_ant_production.csv"));
    if (ANT_MAX_CHAINS != 0 && chains.size() > ANT_MAX_CHAINS)
    {
        chains.resize(ANT_MAX_CHAINS);
    }

    unsigned int numThreads = std::thread::hardware_concurrency();
    if (numThreads < 1)
    {
        numThreads = 1;
    }
    if (numThreads > MAX_TEST_THREADS)
    {
        numThreads = MAX_TEST_THREADS;
    }
    if (numThreads > chains.size())
    {
        numThreads = (unsigned int)chains.size();
    }

    std::mutex failMutex;
    std::vector<std::string> failures;
    std::atomic<bool> loadFailed(false);

    // One thread per chain; a chain replays sequentially (each node's bestANN feeds the next level).
    // The pool is rebuilt only when a chain's seed differs from the last (a single-seed file builds once).
    const auto worker = [&](unsigned int t)
    {
        std::unique_ptr<ProdMiner> miner(new ProdMiner());
        bool poolReady = false;
        unsigned char poolSeed[32];
        for (size_t ci = t; ci < chains.size(); ci += numThreads)
        {
            const AntChainData& chain = chains[ci];
            if (!poolReady || std::memcmp(chain.seed, poolSeed, 32) != 0)
            {
                unsigned char localSeed[32];
                std::memcpy(localSeed, chain.seed, 32);
                if (!miner->initialize(localSeed, taskPath.c_str()))
                {
                    loadFailed.store(true);
                    return;
                }
                std::memcpy(poolSeed, chain.seed, 32);
                poolReady = true;
            }

            unsigned char pub[32];
            std::memcpy(pub, chain.pubkey, 32);
            unsigned char rootSeed[32];
            std::memcpy(rootSeed, chain.seed, 32);
            ProdMiner::ANN parentAnn;
            std::memset(&parentAnn, 0, sizeof(parentAnn));
            // Depth 0's parent is the SHARED epoch root, seeded by the epoch's spectrum digest -
            // the same for every identity. Must match bpp9000_ground_truth.cpp, which wrote the file.
            miner->deriveRootANN(rootSeed, parentAnn);

            for (size_t d = 0; d < chain.nodes.size(); ++d)
            {
                const AntNode& node = chain.nodes[d];
                unsigned char nonce[32];
                unsigned char anchor[32];
                std::memcpy(nonce, node.nonce, 32);
                std::memcpy(anchor, node.anchor, 32);
                const unsigned int score = miner->computeScoreFromParent(parentAnn.lut, pub, nonce, anchor);
                if (score != node.score)
                {
                    std::stringstream ss;
                    ss << "chain " << ci << " depth " << d << ": got " << score << ", expected " << node.score;
                    const std::lock_guard<std::mutex> lock(failMutex);
                    failures.push_back(ss.str());
                }
                std::memcpy(parentAnn.lut, miner->bestANN.lut, sizeof(parentAnn.lut));   // this node -> next parent
            }
        }
    };

    std::vector<std::thread> pool;
    for (unsigned int t = 0; t < numThreads; ++t)
    {
        pool.emplace_back(worker, t);
    }
    for (std::thread& th : pool)
    {
        th.join();
    }

    REQUIRE_FALSE(loadFailed.load());
    std::string report;
    for (const std::string& f : failures)
    {
        report += f;
        report += "\n";
    }
    INFO(report);
    CHECK(failures.empty());
}
