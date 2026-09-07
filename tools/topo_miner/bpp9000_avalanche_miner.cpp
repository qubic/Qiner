// bpp9000_avalanche_miner: offline topology mining by the avalanche (sensitivity) metric.
// AVX-512 build (default) needs an AVX512-VBMI CPU; -DTOPO_MINER_AVX512=OFF builds a scalar binary.
//
// Usage: bpp9000_avalanche_miner <input.task> [out.task] [seed] [--K T] [--L N] [--steps N]
//                                [--patience M] [--restarts R] [--threads T] [--max-unreachable U]
//                                [--selfcheck-samples S]
//   --K T            ticks per propagation (default 10)
//   --L N            random trials per fitness evaluation, higher = better coverage (default 64)
//   --steps N        maximum mutation steps per restart (default 20000)
//   --patience M     stop after M consecutive steps without improvement (default 2000)
//   --restarts R     independent restarts (default: the --threads value)
//   --threads T      worker threads processing the restarts (default 1)
//   --max-unreachable U  allowed evolution neurons that reach neither/only one target (default 2);
//                        input/output/signal neurons must always reach both
//   --selfcheck-samples S  scalar-vs-AVX512 samples before mining, 0 disables (default 16)
//   --dump-all       also write each restart's topology to <out>_rNN (many candidates per run)
//   --eval-input     score the input task's own topology and exit (no search)

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include "topo_common.h"
#include "avalanche_kernel.h"
#include "task_file.h"
#include "K12AndKeyUtil.h"

using namespace topo_common;

// Evaluator for the search: parse the candidate wiring, run the avalanche metric (maximized).
struct AvalancheEval
{
    const avalanche::Trials* trials;
    unsigned int numberOfTicks;
    std::vector<uint32_t> inputIdx;
    std::vector<uint32_t> outputIdx;
    std::vector<uint32_t> neighborIdx;
    uint32_t signalIdx;

    AvalancheEval(const avalanche::Trials* tr, unsigned int k)
        : trials(tr), numberOfTicks(k), inputIdx(N), outputIdx(M), neighborIdx((size_t)P * K),
          signalIdx(0)
    {
    }

    uint64_t operator()(const unsigned char* topoBlock)
    {
        task_file::parseTopologyBlock(topoBlock, N, M, P, K, inputIdx.data(), outputIdx.data(),
                                      &signalIdx, neighborIdx.data());
        return avalanche::fitness(neighborIdx.data(), *trials, numberOfTicks);
    }

    static uint64_t worst() { return 0; }
    static bool isBetter(uint64_t candidate, uint64_t best) { return candidate > best; }
};

int main(int argc, char** argv)
{
    const char* inPath = nullptr;
    const char* outPath = "topo_avalanche.task";
    const char* seedArg = "random";
    unsigned int numberOfTicks = 10;
    unsigned int L = 64;
    uint64_t maxSteps = 20000;
    uint64_t patience = 2000;
    uint32_t maxUnreachable = 2;
    uint32_t restartCount = 0;
    uint32_t threadCount = 1;
    uint64_t selfcheckSamples = 16;
    bool evalInput = false;   // score the input task's topology and exit (no search)
    bool dumpAll = false;     // also write every restart's topology to its own <out>_rNN file

    std::vector<const char*> pos;
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--eval-input") == 0) { evalInput = true; }
        else if (strcmp(argv[i], "--dump-all") == 0) { dumpAll = true; }
        else if (strcmp(argv[i], "--K") == 0 && i + 1 < argc) { numberOfTicks = (unsigned int)strtoul(argv[++i], nullptr, 10); }
        else if (strcmp(argv[i], "--L") == 0 && i + 1 < argc) { L = (unsigned int)strtoul(argv[++i], nullptr, 10); }
        else if (strcmp(argv[i], "--steps") == 0 && i + 1 < argc) { maxSteps = strtoull(argv[++i], nullptr, 10); }
        else if (strcmp(argv[i], "--patience") == 0 && i + 1 < argc) { patience = strtoull(argv[++i], nullptr, 10); }
        else if (strcmp(argv[i], "--max-unreachable") == 0 && i + 1 < argc) { maxUnreachable = (uint32_t)strtoul(argv[++i], nullptr, 10); }
        else if (strcmp(argv[i], "--restarts") == 0 && i + 1 < argc) { restartCount = (uint32_t)strtoul(argv[++i], nullptr, 10); }
        else if (strcmp(argv[i], "--threads") == 0 && i + 1 < argc)
        {
            threadCount = (uint32_t)strtoul(argv[++i], nullptr, 10);
            if (threadCount == 0) { threadCount = 1; }
        }
        else if (strcmp(argv[i], "--selfcheck-samples") == 0 && i + 1 < argc) { selfcheckSamples = strtoull(argv[++i], nullptr, 10); }
        else { pos.push_back(argv[i]); }
    }
    if (pos.empty())
    {
        printf("Usage: bpp9000_avalanche_miner <input.task> [out.task] [seed] [--K T] [--L N] [--steps N] [--patience M] [--restarts R] [--threads T] [--max-unreachable U] [--selfcheck-samples S]\n");
        return 1;
    }
    inPath = pos[0];
    if (pos.size() > 1) { outPath = pos[1]; }
    if (pos.size() > 2) { seedArg = pos[2]; }
    if (numberOfTicks == 0 || L == 0)
    {
        printf("--K and --L must be positive\n");
        return 1;
    }

    uint64_t seed;
    if (strcmp(seedArg, "random") == 0)
    {
        std::random_device rd;
        seed = ((uint64_t)rd() << 32) ^ rd();
    }
    else
    {
        seed = strtoull(seedArg, nullptr, 10);
    }

    task_file::TaskFileHeader header;
    if (!task_file::readTaskFileHeader(inPath, &header))
    {
        printf("Cannot read task header from %s\n", inPath);
        return 1;
    }
    if (header.numInputTrits != N || header.numOutputTrits != M || header.numPairs < T
        || header.population != P || header.numNeighbors != K)
    {
        printf("Task dimensions do not match the production constants\n");
        return 1;
    }

    const uint64_t topoBytes = task_file::topologyBytes(N, M, P, K);
    const uint64_t dataSize = task_file::dataBytes(N, M, header.numPairs);
    const uint64_t scoredDataSize = task_file::dataBytes(N, M, T);
    std::vector<unsigned char> dataBlock(dataSize);
    if (!task_file::readTaskFileBlock(inPath, sizeof(header) + topoBytes, dataBlock.data(), dataSize))
    {
        printf("Cannot read the data block from %s\n", inPath);
        return 1;
    }

    // L trials drawn once and shared across restarts, so fitness is comparable across candidates.
    const avalanche::Trials trials = avalanche::makeTrials(seed, L);

    // Raw fitness sums Hamming over L*2*P perturbation runs; scale it to average affected neurons.
    const double scoreScale = 1.0 / (double)((uint64_t)L * 2 * (uint64_t)P);

    if (evalInput)
    {
        std::vector<unsigned char> topoBlock((size_t)topoBytes);
        if (!task_file::readTaskFileBlock(inPath, sizeof(header), topoBlock.data(), topoBytes))
        {
            printf("Cannot read the topology block from %s\n", inPath);
            return 1;
        }
        std::vector<uint32_t> inIdx(N);
        std::vector<uint32_t> outIdx(M);
        std::vector<uint32_t> nbr((size_t)P * K);
        uint32_t sig = 0;
        task_file::parseTopologyBlock(topoBlock.data(), N, M, P, K, inIdx.data(), outIdx.data(), &sig, nbr.data());
        const uint64_t f = avalanche::fitness(nbr.data(), trials, numberOfTicks);
        printf("eval fitness %llu | avg-affected %.2f\n", (unsigned long long)f, f * scoreScale);
        return 0;
    }

    if (restartCount == 0) { restartCount = threadCount; }
    if (threadCount > restartCount) { threadCount = restartCount; }
    printf("backend: %s | K %u | L %u | restarts %u | threads %u\n",
           avalanche::backendName(), numberOfTicks, L, restartCount, threadCount);

#ifdef TOPO_HAVE_AVALANCHE_AVX512
    if (selfcheckSamples > 0)
    {
        std::mt19937_64 checkRng(seed ^ 0x6176616c616e6368ULL);
        Topology t;
        for (uint64_t s = 0; s < selfcheckSamples; ++s)
        {
            if (!initTopology(t, checkRng, maxUnreachable))
            {
                printf("self-check: could not generate sample %llu\n", (unsigned long long)s);
                return 1;
            }
            const uint64_t sc = avalanche::fitnessScalar(t.neighborIdx.data(), trials, numberOfTicks);
            const uint64_t av = avalanche::fitnessAvx512(t.neighborIdx.data(), trials, numberOfTicks);
            if (sc != av)
            {
                printf("self-check FAILED at sample %llu: scalar %llu vs avx512 %llu\n",
                       (unsigned long long)s, (unsigned long long)sc, (unsigned long long)av);
                return 1;
            }
        }
        printf("self-check passed: %llu samples match\n", (unsigned long long)selfcheckSamples);
    }
#endif

    printf("seed %llu\n", (unsigned long long)seed);

    std::vector<RestartResult> results;
    const uint32_t bestRestart = runSearch<AvalancheEval>(
        threadCount, restartCount, seed, maxSteps, patience, maxUnreachable, "avg-affected", scoreScale,
        results, [&]() { return AvalancheEval(&trials, numberOfTicks); });

    if (bestRestart == restartCount)
    {
        printf("No restart produced a result\n");
        return 1;
    }
    const RestartResult& winner = results[bestRestart];
    const Topology& topo = winner.topo;

    // Write one topology to a task file (recomputes both hashes into header) and return its topo hash.
    std::vector<unsigned char> topoBlock;
    auto writeTopo = [&](const Topology& t, const char* path, char* hexTopoOut) -> bool
    {
        serializeTopo(t, topoBlock);
        KangarooTwelve(topoBlock.data(), (unsigned int)topoBlock.size(), header.topologyHash, task_file::DATA_HASH_SIZE);
        KangarooTwelve(dataBlock.data(), (unsigned int)scoredDataSize, header.dataHash, task_file::DATA_HASH_SIZE);
        if (!task_file::writeTaskFile(path, header, topoBlock.data(), topoBlock.size(), dataBlock.data(), dataBlock.size()))
        {
            return false;
        }
        toHexString(header.topologyHash, task_file::DATA_HASH_SIZE, hexTopoOut);
        return true;
    };

    // Insert _rNN before the output path's extension: "cand.task" -> "cand_r03.task".
    auto restartPath = [&](uint32_t r, char* buf, size_t n)
    {
        const char* dot = strrchr(outPath, '.');
        const char* slash = strrchr(outPath, '/');
        if (dot != nullptr && (slash == nullptr || dot > slash))
        {
            snprintf(buf, n, "%.*s_r%02u%s", (int)(dot - outPath), outPath, r, dot);
        }
        else
        {
            snprintf(buf, n, "%s_r%02u", outPath, r);
        }
    };

    char hexTopo[2 * task_file::DATA_HASH_SIZE + 1];
    if (!writeTopo(topo, outPath, hexTopo))
    {
        printf("Cannot write %s\n", outPath);
        return 1;
    }
    char hexData[2 * task_file::DATA_HASH_SIZE + 1];
    toHexString(header.dataHash, task_file::DATA_HASH_SIZE, hexData);
    printf("done | best restart %u | avg-affected %.2f -> %.2f | accepted %llu mutations | unreachable %u | wrote %s\n",
           bestRestart, winner.initial * scoreScale, winner.best * scoreScale,
           (unsigned long long)winner.accepted, reachabilityFailures(topo).evolutionFailures, outPath);
    printf("BPP9000_TOPOLOGY_HASH: %s\n", hexTopo);
    printf("BPP9000_DATA_HASH:     %s\n", hexData);

    if (dumpAll)
    {
        for (uint32_t r = 0; r < restartCount; ++r)
        {
            if (!results[r].valid)
            {
                continue;
            }
            char path[512];
            char hx[2 * task_file::DATA_HASH_SIZE + 1];
            restartPath(r, path, sizeof(path));
            if (writeTopo(results[r].topo, path, hx))
            {
                printf("dump r%u | avg-affected %.2f | unreachable %u | %s | %s\n",
                       r, results[r].best * scoreScale,
                       reachabilityFailures(results[r].topo).evolutionFailures, hx, path);
            }
        }
    }
    return 0;
}
