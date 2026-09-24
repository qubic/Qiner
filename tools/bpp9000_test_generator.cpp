#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <chrono>
#include <thread>
#include <atomic>

#include "score_bpp9000.h"
#include "bpp9000_params.h"
#include "bpp9000_synth.h"
#include "bpp9000_subview.h"
#include "task_file.h"
#include "K12AndKeyUtil.h"

// ProdConfig is hardcoded in the portable bpp9000_params.h; enforce that it matches the real production
// constants here (the one place that includes both), so the golden is built at true production dims.
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

// Ground-truth test generator
// Usage: bpp9000_test_generator [numSamples=16] [task.bin] [samples.csv] [scores.csv] [numThreads]
// Samples are scored in parallel (each thread its own Miner + ~512MB pool); configs run sequentially.
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

struct Sample
{
    unsigned char pub[32];
    unsigned char non[32];
};

template<typename C>
static std::string paramHeader()
{
    char buf[160];
    snprintf(buf, sizeof(buf), "%llu-%llu-%llu-%llu-%llu-%llu-%llu-%llu-%u",
             (unsigned long long)C::numberOfInputNeurons, (unsigned long long)C::numberOfOutputNeurons,
             (unsigned long long)C::sequenceLength, (unsigned long long)C::windowWidth,
             (unsigned long long)C::maxNumberOfTicks, (unsigned long long)C::numberOfNeighbors,
             (unsigned long long)C::populationThreshold, (unsigned long long)C::numberOfMutations,
             (unsigned int)C::solutionThreshold);
    return std::string(buf);
}

template<typename C>
static bool scoreColumn(const char* taskPath, const unsigned char* seed, const std::vector<Sample>& samples,
                        int numThreads, std::vector<std::string>& headers,
                        std::vector<std::vector<unsigned int>>& columns)
{
    using Miner = score_bpp9000::Miner<
        C::numberOfInputNeurons, C::numberOfOutputNeurons, C::sequenceLength, C::windowWidth,
        C::maxNumberOfTicks, C::numberOfNeighbors, C::populationThreshold, C::numberOfMutations,
        C::solutionThreshold,
        C::shiftCap>;

    const size_t n = samples.size();
    std::vector<unsigned int> col(n);

    int threads = numThreads;
    if (threads < 1)
    {
        threads = 1;
    }
    if (n == 0)
    {
        threads = 1;
    }
    else if ((size_t)threads > n)
    {
        threads = (int)n;
    }

    std::atomic<bool> ok{true};

    // Each thread owns its Miner (own ~512MB pool + subview task load)
    auto worker = [&](int tid)
    {
        Miner* miner = new Miner();
        unsigned char localSeed[32];
        memcpy(localSeed, seed, 32);
        std::vector<unsigned char> topo;
        std::vector<unsigned char> data;
        if (!bpp9000_subview::readSubviewBlocks<C>(taskPath, topo, data) ||
            !miner->initializeFromMemory(localSeed, topo.data(), data.data()))
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
            col[i] = miner->computeScore(pub, non).error;
        }
        delete miner;
    };

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

    if (!ok)
    {
        printf("[FAIL] subview load rejected for config %s\n", paramHeader<C>().c_str());
        return false;
    }
    const double totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    printf("  config %-40s : %.1f ms total, %.1f ms/sample (%d threads)\n",
           paramHeader<C>().c_str(), totalMs, (n > 0) ? totalMs / (double)n : 0.0, threads);

    headers.push_back(paramHeader<C>());
    columns.push_back(std::move(col));
    return true;
}

template<std::size_t... Is>
static bool scoreAll(std::index_sequence<Is...>, const char* taskPath, const unsigned char* seed,
                     const std::vector<Sample>& samples, int numThreads, std::vector<std::string>& headers,
                     std::vector<std::vector<unsigned int>>& columns)
{
    bool ok = true;
    ((ok = scoreColumn<std::tuple_element_t<Is, bpp9000_params::ConfigList>>(taskPath, seed, samples, numThreads, headers, columns) && ok), ...);
    return ok;
}

int main(int argc, char** argv)
{
    const int numSamples = (argc > 1) ? atoi(argv[1]) : 16;
    const char* taskPath = (argc > 2) ? argv[2] : "task_bpp9000.bin";
    const char* samplesPath = (argc > 3) ? argv[3] : "bpp9000_samples.csv";
    const char* scoresPath = (argc > 4) ? argv[4] : "bpp9000_scores.csv";
    int numThreads = (argc > 5) ? atoi(argv[5]) : (int)std::thread::hardware_concurrency();
    if (numThreads < 1)
    {
        numThreads = 1;
    }
    if (numThreads > numSamples && numSamples > 0)
    {
        numThreads = numSamples;
    }

    using Prod = bpp9000_params::ProdConfig;

    // Build + write the one full synthesic production task file.
    std::vector<unsigned char> topo;
    std::vector<unsigned char> data;
    bpp9000_synth::buildSyntheticTaskBlocks<Prod>(topo, data);

    task_file::TaskFileHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = task_file::MAGIC;
    header.version = task_file::VERSION;
    header.numInputTrits = (unsigned int)Prod::numberOfInputNeurons;
    header.numOutputTrits = (unsigned int)Prod::numberOfOutputNeurons;
    header.numPairs = Prod::sequenceLength;
    header.population = (unsigned int)Prod::populationThreshold;
    header.numNeighbors = (unsigned int)Prod::numberOfNeighbors;
    KangarooTwelve(topo.data(), (unsigned int)topo.size(), header.topologyHash, task_file::DATA_HASH_SIZE);
    KangarooTwelve(data.data(), (unsigned int)data.size(), header.dataHash, task_file::DATA_HASH_SIZE);
    if (!task_file::writeTaskFile(taskPath, header, topo.data(), topo.size(), data.data(), data.size()))
    {
        printf("Failed to write %s\n", taskPath);
        return 1;
    }
    printf("Wrote full task file %s (N=%u M=%u T=%llu P=%u K=%u)\n", taskPath,
           header.numInputTrits, header.numOutputTrits, (unsigned long long)header.numPairs,
           header.population, header.numNeighbors);

    // Random samples (one shared mining seed).
    unsigned char seed[32];
    bpp9000_synth::fillRandom(seed, 32);

    std::vector<Sample> samples((size_t)(numSamples > 0 ? numSamples : 0));
    for (int i = 0; i < numSamples; ++i)
    {
        bpp9000_synth::fillRandom(samples[i].pub, 32);
        bpp9000_synth::fillRandom(samples[i].non, 32);
        // Canonical standalone nonce: nonce[1] carries L (bits 0-3, in [1, 10]) and the mutation mode
        // (bits 4-5, in [1, 3]); nonce[2] = K = 0.
        samples[i].non[0] = 1;   // AlgoType::Bpp9000
        const unsigned char L = (unsigned char)((samples[i].non[1] % score_bpp9000::MAX_CHANGES_PER_STEP) + 1);
        const unsigned char mode = (unsigned char)(((samples[i].non[1] >> 4) % 3) + 1);
        samples[i].non[1] = (unsigned char)(L | (mode << 4));
        samples[i].non[2] = 0;
    }

    FILE* sf = fopen(samplesPath, "w");
    if (sf == nullptr)
    {
        printf("Cannot open %s\n", samplesPath);
        return 1;
    }
    fprintf(sf, "seed, publickey, nonce\n");
    char seedHex[65];
    toHex(seed, 32, seedHex);
    for (int i = 0; i < numSamples; ++i)
    {
        char pubHex[65];
        char nonHex[65];
        toHex(samples[i].pub, 32, pubHex);
        toHex(samples[i].non, 32, nonHex);
        fprintf(sf, "%s, %s, %s\n", seedHex, pubHex, nonHex);
    }
    fclose(sf);

    // Score every config (each subview-loads from the full file).
    std::vector<std::string> headers;
    std::vector<std::vector<unsigned int>> columns;
    constexpr std::size_t configCount = std::tuple_size_v<bpp9000_params::ConfigList>;
    printf("Scoring %zu configs x %d samples, up to %d threads/config (~%d x 512MB peak), subview from %s:\n",
           configCount, numSamples, numThreads, numThreads, taskPath);
    const bool ok = scoreAll(std::make_index_sequence<configCount>{}, taskPath, seed, samples, numThreads, headers, columns);

    // Write the multi-config scores CSV.
    FILE* cf = fopen(scoresPath, "w");
    if (cf == nullptr)
    {
        printf("Cannot open %s\n", scoresPath);
        return 1;
    }
    for (size_t c = 0; c < headers.size(); ++c)
    {
        fprintf(cf, "%s%s", headers[c].c_str(), (c + 1 < headers.size()) ? ", " : "");
    }
    fprintf(cf, "\n");
    for (int s = 0; s < numSamples; ++s)
    {
        for (size_t c = 0; c < columns.size(); ++c)
        {
            fprintf(cf, "%u%s", columns[c][s], (c + 1 < columns.size()) ? ", " : "");
        }
        fprintf(cf, "\n");
    }
    fclose(cf);

    printf("Wrote %s (%zu columns) and %s (%d samples)%s\n", scoresPath, columns.size(), samplesPath,
           numSamples, ok ? "" : "  [SOME CONFIGS FAILED]");
    return ok ? 0 : 1;
}
