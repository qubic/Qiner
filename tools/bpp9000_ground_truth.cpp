#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#include "score_bpp9000.h"
#include "bpp9000_params.h"
#include "bpp9000_synth.h"
#include "task_file.h"
#include "K12AndKeyUtil.h"

// Ground-truth reference-vector generator for the bpp9000 PRODUCTION config.
//
// Loads the REAL production task file (hash-verified), scores random (publicKey, nonce) samples under one
// shared mining seed with the reference scorer score_bpp9000::Miner<ProdConfig> (mutations=100), and writes a
// single CSV:
//     pubkey, nonce, miningseed, score
// score = reference failure count in [0, SEQUENCE_LENGTH - WINDOW_WIDTH] (0..8088), lower is better;
//     4294967295 (INFINITE_ERROR) = per-tick timeout sentinel. The node remaps that sentinel to
//     numberOfWindows in score_engine::ScoreEngine::computeBpp9000Score; this file keeps the raw canonical
//     value (only differs on pathological timeouts, which good solutions never hit).
//
// One shared mining seed per run so the ~512MB pool builds once per thread and scoring stays parallel; the
// seed is written into every row so each row fully specifies its score input. For multiple seeds, run once
// per seed and concatenate.
//
// Usage:
//   bpp9000_ground_truth [task=bpp9000.task] [numSamples=16] [out.csv] [miningSeedHex|random] [numThreads=hw]
//                        [--ant] [--depth N]
//
// Solo mode (default): writes "pubkey, nonce, miningseed, score" from computeScore.
// Ant mode (--ant): numSamples = number of chains; each chain is a lineage of --depth nodes (default 2)
//   under one random pubkey. Level 0 extends the derived root; level i extends level i-1's bestANN.
//   Writes "chain, depth, pubkey, nonce, anchor, seed, score" from computeScoreFromParent (canonical
//   nonces, random anchors). A chain is sequential (each node feeds the next), so threading is across chains.

// ProdConfig is hardcoded in the portable bpp9000_params.h; enforce that it matches the real production
// constants here (the one place that includes both), so the ground truth is built at true production dims.
static_assert(
    bpp9000_params::ProdConfig::numberOfInputNeurons == score_bpp9000::NUMBER_OF_INPUT_NEURONS &&
    bpp9000_params::ProdConfig::numberOfOutputNeurons == score_bpp9000::NUMBER_OF_OUTPUT_NEURONS &&
    bpp9000_params::ProdConfig::sequenceLength == score_bpp9000::SEQUENCE_LENGTH &&
    bpp9000_params::ProdConfig::windowWidth == score_bpp9000::WINDOW_WIDTH &&
    bpp9000_params::ProdConfig::maxNumberOfTicks == score_bpp9000::MAX_NUMBER_OF_TICKS &&
    bpp9000_params::ProdConfig::numberOfNeighbors == score_bpp9000::NUMBER_OF_NEIGHBORS &&
    bpp9000_params::ProdConfig::populationThreshold == score_bpp9000::POPULATION_THRESHOLD &&
    bpp9000_params::ProdConfig::numberOfMutations == score_bpp9000::NUMBER_OF_MUTATIONS &&
    bpp9000_params::ProdConfig::solutionThreshold == score_bpp9000::SOLUTION_THRESHOLD,
    "bpp9000_params::ProdConfig must match the production constants in src/score_bpp9000.h");

using Prod = bpp9000_params::ProdConfig;
using ProdMiner = score_bpp9000::Miner<
    Prod::numberOfInputNeurons, Prod::numberOfOutputNeurons, Prod::sequenceLength, Prod::windowWidth,
    Prod::maxNumberOfTicks, Prod::numberOfNeighbors, Prod::populationThreshold, Prod::numberOfMutations,
    Prod::solutionThreshold>;

static void toHex(const unsigned char* b, int n, char* out)
{
    static const char* h = "0123456789abcdef";
    for (int i = 0; i < n; ++i)
    {
        out[2 * i] = h[b[i] >> 4];
        out[2 * i + 1] = h[b[i] & 0xF];
    }
    out[2 * n] = 0;
}

// Parse exactly 64 hex chars into 32 bytes. Returns false on wrong length or a non-hex char.
static bool hexTo32(const char* hex, unsigned char* out)
{
    if (strlen(hex) != 64)
    {
        return false;
    }
    for (int i = 0; i < 32; ++i)
    {
        unsigned int byte = 0;
        if (sscanf(hex + i * 2, "%2x", &byte) != 1)
        {
            return false;
        }
        out[i] = (unsigned char)byte;
    }
    return true;
}

struct Sample
{
    unsigned char pub[32];
    unsigned char non[32];
};

int main(int argc, char** argv)
{
    // Positional args (task, numSamples, out, seed, threads) may be interleaved with the flags
    // --ant (ant-colony chain mode) and --depth N (chain length). In ant mode numSamples is the number
    // of chains and each chain emits `depth` rows.
    std::vector<const char*> pos;
    bool antMode = false;
    bool skipTimeouts = false;
    int depth = 2;
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--ant") == 0)
        {
            antMode = true;
        }
        else if (strcmp(argv[i], "--skip-timeouts") == 0)
        {
            skipTimeouts = true;
        }
        else if (strcmp(argv[i], "--depth") == 0 && i + 1 < argc)
        {
            depth = atoi(argv[++i]);
        }
        else
        {
            pos.push_back(argv[i]);
        }
    }
    const char* taskPath = (pos.size() > 0) ? pos[0] : "bpp9000.task";
    const int numSamples = (pos.size() > 1) ? atoi(pos[1]) : 16;
    const char* outPath = (pos.size() > 2) ? pos[2] : (antMode ? "bpp9000_ant_ground_truth.csv" : "bpp9000_ground_truth.csv");
    const char* seedArg = (pos.size() > 3) ? pos[3] : "random";
    int numThreads = (pos.size() > 4) ? atoi(pos[4]) : (int)std::thread::hardware_concurrency();
    if (numThreads < 1)
    {
        numThreads = 1;
    }

    // Read + print the task header hashes so the operator can confirm this is the pinned canonical task.
    task_file::TaskFileHeader hdr;
    if (!task_file::readTaskFileHeader(taskPath, &hdr))
    {
        printf("Cannot read task header from %s\n", taskPath);
        return 1;
    }
    char topoHex[65];
    char dataHex[65];
    toHex(hdr.topologyHash, 32, topoHex);
    toHex(hdr.dataHash, 32, dataHex);
    printf("Task %s: N=%u M=%u T=%llu P=%u K=%u\n", taskPath, hdr.numInputTrits, hdr.numOutputTrits,
           (unsigned long long)hdr.numPairs, hdr.population, hdr.numNeighbors);
    printf("  topologyHash = %s\n", topoHex);
    printf("  dataHash     = %s\n", dataHex);
    printf("  (must match BPP9000_TOPOLOGY_HASH / BPP9000_DATA_HASH in core/src/public_settings.h)\n");

    // Mining seed: 64-hex or "random".
    unsigned char seed[32];
    if (strcmp(seedArg, "random") == 0)
    {
        bpp9000_synth::fillRandom(seed, 32);
    }
    else if (!hexTo32(seedArg, seed))
    {
        printf("Invalid mining seed '%s' (need 64 hex chars or \"random\")\n", seedArg);
        return 1;
    }

    if (antMode)
    {
        if (depth < 1)
        {
            depth = 1;
        }
        const int numChains = (numSamples > 0) ? numSamples : 0;

        // Each chain: one identity (pubkey) and a lineage of `depth` nodes. Level 0 extends the derived
        // root; level i extends level i-1's bestANN. Per level: a canonical nonce + a random anchor.
        struct AntChain
        {
            unsigned char pub[32];
            std::vector<unsigned char> nonces;    // depth * 32
            std::vector<unsigned char> anchors;   // depth * 32
        };
        std::vector<AntChain> chains((size_t)numChains);
        for (int c = 0; c < numChains; ++c)
        {
            bpp9000_synth::fillRandom(chains[(size_t)c].pub, 32);
            chains[(size_t)c].nonces.resize((size_t)depth * 32);
            chains[(size_t)c].anchors.resize((size_t)depth * 32);
            for (int d = 0; d < depth; ++d)
            {
                unsigned char* nonce = &chains[(size_t)c].nonces[(size_t)d * 32];
                unsigned char* anchor = &chains[(size_t)c].anchors[(size_t)d * 32];
                bpp9000_synth::fillRandom(nonce, 32);
                nonce[0] = 1;                                                                          // AlgoType::Bpp9000
                nonce[1] = (unsigned char)((nonce[1] % score_bpp9000::MAX_LUT_ENTRIES_PER_STEP) + 1);  // L in [1, 10]
                nonce[2] = (unsigned char)(nonce[2] % (score_bpp9000::NUMBER_OF_MUTATIONS + 1));        // K in [0, 100]
                bpp9000_synth::fillRandom(anchor, 32);
            }
        }

        int antThreads = numThreads;
        if (numChains == 0)
        {
            antThreads = 1;
        }
        else if (antThreads > numChains)
        {
            antThreads = numChains;
        }

        FILE* f = fopen(outPath, "w");
        if (f == nullptr)
        {
            printf("Cannot open %s\n", outPath);
            return 1;
        }
        fprintf(f, "chain, depth, pubkey, nonce, anchor, seed, score\n");
        fflush(f);

        char seedHex[65];
        toHex(seed, 32, seedHex);

        std::mutex writeMutex;
        std::atomic<bool> ok{true};
        std::atomic<int> rowsWritten{0};

        // One thread per chain (chains are independent). Within a chain the levels are sequential -
        // each node's bestANN becomes the next level's parent - so a chain cannot be parallelized.
        auto antWorker = [&](int tid)
        {
            ProdMiner* miner = new ProdMiner();
            unsigned char localSeed[32];
            memcpy(localSeed, seed, 32);
            if (!miner->initialize(localSeed, taskPath))
            {
                ok = false;
                delete miner;
                return;
            }
            unsigned char pub[32];
            std::vector<unsigned char> nonces((size_t)depth * 32);
            std::vector<unsigned char> anchors((size_t)depth * 32);
            std::vector<unsigned int> scores((size_t)depth);
            for (int c = tid; c < numChains; c += antThreads)
            {
                for (;;)   // one pass, unless --skip-timeouts rejects a lineage that hit INFINITE_ERROR
                {
                    if (skipTimeouts)
                    {
                        // Fresh canonical inputs each attempt (rdrand is thread-safe).
                        bpp9000_synth::fillRandom(pub, 32);
                        for (int d = 0; d < depth; ++d)
                        {
                            unsigned char* nonce = &nonces[(size_t)d * 32];
                            unsigned char* anchor = &anchors[(size_t)d * 32];
                            bpp9000_synth::fillRandom(nonce, 32);
                            nonce[0] = 1;                                                                          // AlgoType::Bpp9000
                            nonce[1] = (unsigned char)((nonce[1] % score_bpp9000::MAX_LUT_ENTRIES_PER_STEP) + 1);  // L in [1, 10]
                            nonce[2] = (unsigned char)(nonce[2] % (score_bpp9000::NUMBER_OF_MUTATIONS + 1));        // K in [0, 100]
                            bpp9000_synth::fillRandom(anchor, 32);
                        }
                    }
                    else
                    {
                        const AntChain& ch = chains[(size_t)c];
                        memcpy(pub, ch.pub, 32);
                        memcpy(nonces.data(), ch.nonces.data(), (size_t)depth * 32);
                        memcpy(anchors.data(), ch.anchors.data(), (size_t)depth * 32);
                    }

                    ProdMiner::ANN parentAnn;
                    memset(&parentAnn, 0, sizeof(parentAnn));
                    miner->deriveRootANN(localSeed, parentAnn);   // level 0's parent = the shared epoch root
                    bool anyInfinite = false;
                    for (int d = 0; d < depth; ++d)
                    {
                        scores[(size_t)d] = miner->computeScoreFromParent(parentAnn.lut, pub, &nonces[(size_t)d * 32], &anchors[(size_t)d * 32]);
                        if (scores[(size_t)d] == ProdMiner::INFINITE_ERROR)
                        {
                            anyInfinite = true;
                            if (skipTimeouts)
                            {
                                break;   // doomed lineage - stop scoring the rest, draw a fresh one
                            }
                        }
                        // This node becomes the next level's parent (its stored canonical LUT = bestANN).
                        memcpy(parentAnn.lut, miner->bestANN.lut, sizeof(parentAnn.lut));
                    }

                    if (skipTimeouts && anyInfinite)
                    {
                        continue;   // reject a timeout lineage, draw a fresh one
                    }

                    char pubHex[65];
                    toHex(pub, 32, pubHex);
                    for (int d = 0; d < depth; ++d)
                    {
                        char nonHex[65];
                        char anchorHex[65];
                        toHex(&nonces[(size_t)d * 32], 32, nonHex);
                        toHex(&anchors[(size_t)d * 32], 32, anchorHex);
                        {
                            std::lock_guard<std::mutex> lock(writeMutex);
                            fprintf(f, "%d, %d, %s, %s, %s, %s, %u\n", c, d, pubHex, nonHex, anchorHex, seedHex, scores[(size_t)d]);
                            fflush(f);
                        }
                        ++rowsWritten;
                    }
                    break;
                }
            }
            delete miner;
        };

        printf("Ant mode: %d chains x depth %d = %d rows under ProdConfig (mut=%llu), %d threads (~%d x 512MB):\n",
               numChains, depth, numChains * depth, (unsigned long long)Prod::numberOfMutations, antThreads, antThreads);
        printf("Appending each finished node to %s (mining seed %s)\n", outPath, seedHex);

        using clk = std::chrono::steady_clock;
        const auto t0 = clk::now();
        std::vector<std::thread> pool;
        pool.reserve((size_t)antThreads);
        for (int t = 0; t < antThreads; ++t)
        {
            pool.emplace_back(antWorker, t);
        }
        for (auto& th : pool)
        {
            th.join();
        }
        const auto t1 = clk::now();
        fclose(f);

        if (!ok)
        {
            printf("[FAIL] task load rejected: %s (%d rows written)\n", taskPath, rowsWritten.load());
            return 1;
        }
        const double totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        printf("  %.1f ms total, %.1f ms/row\n", totalMs, (rowsWritten.load() > 0) ? totalMs / rowsWritten.load() : 0.0);
        printf("Wrote %s (%d rows)\n", outPath, rowsWritten.load());
        return 0;
    }

    // Random samples.
    std::vector<Sample> samples((size_t)(numSamples > 0 ? numSamples : 0));
    for (int i = 0; i < numSamples; ++i)
    {
        bpp9000_synth::fillRandom(samples[i].pub, 32);
        bpp9000_synth::fillRandom(samples[i].non, 32);
    }

    const size_t n = samples.size();

    int threads = numThreads;
    if (n == 0)
    {
        threads = 1;
    }
    else if ((size_t)threads > n)
    {
        threads = (int)n;
    }

    // Open the CSV up front and write the header; each sample row is appended and flushed the moment that
    // sample finishes, so a long run or an interruption still leaves every completed sample on disk. Rows are
    // written in completion order (interleaved across threads), which is fine - each row is self-describing.
    FILE* f = fopen(outPath, "w");
    if (f == nullptr)
    {
        printf("Cannot open %s\n", outPath);
        return 1;
    }
    fprintf(f, "pubkey, nonce, miningseed, score\n");
    fflush(f);

    char seedHex[65];
    toHex(seed, 32, seedHex);

    std::mutex writeMutex;
    std::atomic<bool> ok{true};
    std::atomic<int> written{0};

    // Each thread owns its Miner (own ~512MB pool + full task load from the real file). initialize() is the
    // production-faithful path: exact-dim check + read each block + K12-verify both hashes against the header,
    // the same as the node's loadBpp9000Task (no subview reslicing).
    auto worker = [&](int tid)
    {
        ProdMiner* miner = new ProdMiner();
        unsigned char localSeed[32];
        memcpy(localSeed, seed, 32);
        if (!miner->initialize(localSeed, taskPath))
        {
            ok = false;
            delete miner;
            return;
        }
        for (size_t i = (size_t)tid; i < n; i += (size_t)threads)
        {
            unsigned char pub[32];
            unsigned char non[32];
            memcpy(pub, samples[i].pub, 32);
            memcpy(non, samples[i].non, 32);
            const unsigned int score = miner->computeScore(pub, non);

            char pubHex[65];
            char nonHex[65];
            toHex(samples[i].pub, 32, pubHex);
            toHex(samples[i].non, 32, nonHex);
            {
                std::lock_guard<std::mutex> lock(writeMutex);
                fprintf(f, "%s, %s, %s, %u\n", pubHex, nonHex, seedHex, score);
                fflush(f);
            }
            ++written;
        }
        delete miner;
    };

    printf("Scoring %d samples under ProdConfig (mut=%llu), up to %d threads (~%d x 512MB peak):\n",
           numSamples, (unsigned long long)Prod::numberOfMutations, threads, threads);
    printf("Appending each finished sample to %s (mining seed %s)\n", outPath, seedHex);

    using clk = std::chrono::steady_clock;
    const auto t0 = clk::now();
    std::vector<std::thread> pool;
    pool.reserve((size_t)threads);
    for (int t = 0; t < threads; ++t)
    {
        pool.emplace_back(worker, t);
    }
    for (auto& th : pool)
    {
        th.join();
    }
    const auto t1 = clk::now();

    fclose(f);

    if (!ok)
    {
        printf("[FAIL] task load rejected (hash mismatch or malformed): %s  (%d samples written)\n",
               taskPath, written.load());
        return 1;
    }
    const double totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    printf("  %.1f ms total, %.1f ms/sample\n", totalMs, (n > 0) ? totalMs / (double)n : 0.0);
    printf("Wrote %s (%d samples)\n", outPath, written.load());
    return 0;
}
