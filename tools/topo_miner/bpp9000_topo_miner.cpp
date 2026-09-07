// bpp9000_topo_miner: offline pre-epoch topology mining.
//
// Usage: bpp9000_topo_miner <input.task> [out.task] [seed] [--steps N] [--patience M] [--max-unreachable U] [--selfcheck-samples S] [--restarts R] [--threads T]
//   input.task       task file supplying the header dimensions and the data block
//   out.task         output path (default topo_mined.task)
//   seed             decimal 64-bit seed or "random" (default random)
//   --steps N        maximum mutation steps per restart (default 20000)
//   --patience M     stop after M consecutive steps without improvement (default 2000)
//   --max-unreachable U  allowed evolution neurons that reach neither/only one target (default 2);
//                        input/output/signal neurons must always reach both
//   --selfcheck-samples S  SIMD-vs-reference samples before mining, 0 disables (default 32)
//   --restarts R     independent hill climbs (default: the --threads value)
//   --threads T      worker threads processing the restarts (default 1)

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "bpp9000_params.h"
#include "score_bpp9000.h"
#include "task_file.h"
#include "K12AndKeyUtil.h"
#include "topo_scorer.h"

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

static constexpr uint32_t N = (uint32_t)Prod::numberOfInputNeurons;
static constexpr uint32_t M = (uint32_t)Prod::numberOfOutputNeurons;
static constexpr uint32_t K = (uint32_t)Prod::numberOfNeighbors;
static constexpr uint32_t P = (uint32_t)Prod::populationThreshold;
static constexpr uint64_t T = (uint64_t)Prod::sequenceLength;
static constexpr uint64_t LUT_SIZE = ProdMiner::lutSize;

struct Topology
{
    std::vector<uint32_t> inputIdx;
    std::vector<uint32_t> outputIdx;
    uint32_t signalIdx;
    std::vector<uint32_t> neighborIdx;
};

// Neurons that cannot reach BOTH the signal neuron and an output neuron, split into critical
// failures (input/output/signal neurons - never allowed) and evolution failures (bounded by
// --max-unreachable). Influence runs from a neuron to its readers, so the set that reaches
// target t = BFS from t over "neighbors of" edges.
struct Reachability
{
    uint32_t criticalFailures;
    uint32_t evolutionFailures;
};

static Reachability reachabilityFailures(const Topology& topo)
{
    // An input neuron's value comes from the feed, not from its neighbor entries, so influence
    // traversal must stop there: its LUT row is never evaluated.
    std::vector<bool> isInput(P, false);
    for (uint32_t i = 0; i < N; ++i)
    {
        isInput[topo.inputIdx[i]] = true;
    }

    auto ancestors = [&](uint32_t target, std::vector<bool>& mark)
    {
        std::vector<uint32_t> frontier;
        mark.assign(P, false);
        mark[target] = true;
        frontier.push_back(target);
        while (!frontier.empty())
        {
            const uint32_t y = frontier.back();
            frontier.pop_back();
            if (isInput[y])
            {
                continue;
            }
            for (uint32_t k = 0; k < K; ++k)
            {
                const uint32_t feeder = topo.neighborIdx[(size_t)y * K + k];
                if (!mark[feeder])
                {
                    mark[feeder] = true;
                    frontier.push_back(feeder);
                }
            }
        }
    };

    std::vector<bool> reachesSignal;
    ancestors(topo.signalIdx, reachesSignal);
    std::vector<bool> reachesOutput(P, false);
    for (uint32_t j = 0; j < M; ++j)
    {
        std::vector<bool> mark;
        ancestors(topo.outputIdx[j], mark);
        for (uint32_t n = 0; n < P; ++n)
        {
            if (mark[n])
            {
                reachesOutput[n] = true;
            }
        }
    }

    std::vector<bool> isCritical(P, false);
    for (uint32_t i = 0; i < N; ++i)
    {
        isCritical[topo.inputIdx[i]] = true;
    }
    for (uint32_t j = 0; j < M; ++j)
    {
        isCritical[topo.outputIdx[j]] = true;
    }
    isCritical[topo.signalIdx] = true;

    Reachability result;
    result.criticalFailures = 0;
    result.evolutionFailures = 0;
    for (uint32_t n = 0; n < P; ++n)
    {
        if (!reachesSignal[n] || !reachesOutput[n])
        {
            if (isCritical[n])
            {
                ++result.criticalFailures;
            }
            else
            {
                ++result.evolutionFailures;
            }
        }
    }
    return result;
}

static bool reachabilityOk(const Reachability& reach, uint32_t maxUnreachable)
{
    return reach.criticalFailures == 0 && reach.evolutionFailures <= maxUnreachable;
}

// One neighbor slot of one NON-INPUT neuron rewired to a random target, keeping no-self and
// no-duplicate by linear probing. Input neurons never evaluate their LUT row or read their
// neighbors, so mutating their rows would only waste a full evaluation. The new target also
// differs from the previous one, so every mutation is a real topology change.
// Returns the previous target so the caller can revert.
static uint32_t mutateOneNeighbor(Topology& topo, std::mt19937_64& rng, uint32_t& outNeuron, uint32_t& outSlot)
{
    std::vector<bool> isInput(P, false);
    for (uint32_t i = 0; i < N; ++i)
    {
        isInput[topo.inputIdx[i]] = true;
    }

    outNeuron = (uint32_t)(rng() % P);
    while (isInput[outNeuron])
    {
        outNeuron = (uint32_t)(rng() % P);
    }
    outSlot = (uint32_t)(rng() % K);
    const uint32_t previous = topo.neighborIdx[(size_t)outNeuron * K + outSlot];

    uint32_t nb = (uint32_t)(rng() % P);
    bool clash = true;
    while (clash)
    {
        clash = (nb == outNeuron) || (nb == previous);
        for (uint32_t p = 0; p < K && !clash; ++p)
        {
            if (p != outSlot && topo.neighborIdx[(size_t)outNeuron * K + p] == nb)
            {
                clash = true;
            }
        }
        if (clash)
        {
            nb = (nb + 1) % P;
        }
    }
    topo.neighborIdx[(size_t)outNeuron * K + outSlot] = nb;
    return previous;
}

// Random topology satisfying the properties: distinct input/output/signal placements, no
// self-reference, no duplicate neighbors, critical liveness, and the reachability bound.
// Critical liveness makes a uniformly random draw pass only rarely (well under 1%), so the
// attempt budget is large; one attempt is just two 64-node BFS passes, microseconds.
static bool initTopology(Topology& topo, std::mt19937_64& rng, uint32_t maxUnreachable)
{
    for (int attempt = 0; attempt < 1000000; ++attempt)
    {
        topo.inputIdx.assign(N, 0);
        topo.outputIdx.assign(M, 0);
        topo.neighborIdx.assign((size_t)P * K, 0);
        std::vector<bool> used(P, false);

        auto pickDistinct = [&]() -> uint32_t
        {
            uint32_t idx = (uint32_t)(rng() % P);
            while (used[idx])
            {
                idx = (idx + 1) % P;
            }
            used[idx] = true;
            return idx;
        };
        for (uint32_t i = 0; i < N; ++i)
        {
            topo.inputIdx[i] = pickDistinct();
        }
        for (uint32_t j = 0; j < M; ++j)
        {
            topo.outputIdx[j] = pickDistinct();
        }
        topo.signalIdx = pickDistinct();

        for (uint32_t n = 0; n < P; ++n)
        {
            for (uint32_t k = 0; k < K; ++k)
            {
                uint32_t nb = (uint32_t)(rng() % P);
                bool clash = true;
                while (clash)
                {
                    clash = (nb == n);
                    for (uint32_t p = 0; p < k && !clash; ++p)
                    {
                        if (topo.neighborIdx[(size_t)n * K + p] == nb)
                        {
                            clash = true;
                        }
                    }
                    if (clash)
                    {
                        nb = (nb + 1) % P;
                    }
                }
                topo.neighborIdx[(size_t)n * K + k] = nb;
            }
        }

        if (reachabilityOk(reachabilityFailures(topo), maxUnreachable))
        {
            return true;
        }
    }
    return false;
}

static void serializeTopo(const Topology& topo, std::vector<unsigned char>& block)
{
    block.resize((size_t)task_file::topologyBytes(N, M, P, K));
    task_file::serializeTopologyBlock(N, M, P, K, topo.inputIdx.data(), topo.outputIdx.data(),
                                      topo.signalIdx, topo.neighborIdx.data(), block.data());
}

// Error of the fixed LUT under the candidate topology. INFINITE_ERROR counts as worst.
static unsigned int evaluate(ProdMiner* miner, const unsigned char* topoBlock,
                             const unsigned char* dataBlock, const unsigned char* lut)
{
    if (!miner->loadTaskFromMemory(topoBlock, dataBlock))
    {
        return ProdMiner::INFINITE_ERROR;
    }
    for (uint32_t n = 0; n < P; ++n)
    {
        miner->currentANN.neurons[n].type = miner->neuronTypes[n];
        miner->currentANN.neurons[n].value = ProdMiner::TRIT_UNKNOWN;
    }
    memcpy(miner->currentANN.lut, lut, (size_t)P * LUT_SIZE);
    return miner->score();
}

#ifndef TOPO_HAVE_CORE_SIMD
// Reference backend: the scalar scorer above. topo_simd_scorer.cpp provides these instead when a
// core checkout was found at configure time.
struct TopoScorer
{
    std::unique_ptr<ProdMiner> miner;
    const unsigned char* dataBlock;
    std::vector<unsigned char> lut;
};

TopoScorer* topoScorerCreate(const unsigned char* dataBlock, const unsigned char* lutAbsolute)
{
    auto scorer = std::make_unique<TopoScorer>();
    scorer->miner = std::make_unique<ProdMiner>();
    scorer->dataBlock = dataBlock;
    scorer->lut.assign(lutAbsolute, lutAbsolute + (size_t)P * LUT_SIZE);
    return scorer.release();
}

unsigned int topoScorerScore(TopoScorer* scorer, const unsigned char* topoBlock)
{
    return evaluate(scorer->miner.get(), topoBlock, scorer->dataBlock, scorer->lut.data());
}

void topoScorerDestroy(TopoScorer* scorer)
{
    delete scorer;
}

const char* topoScorerBackendName()
{
    return "reference";
}
#endif

struct ClimbResult
{
    Topology topo;
    unsigned int initial = 0xFFFFFFFFU;
    unsigned int best = 0xFFFFFFFFU;
    uint64_t accepted = 0;
    uint64_t rejectedReach = 0;
    bool valid = false;
};

// Shared live counters the main thread renders as a rolling status line; workers never print.
struct Progress
{
    std::atomic<uint64_t> steps{0};
    std::atomic<unsigned int> globalBest{0xFFFFFFFFU};
    std::atomic<uint32_t> done{0};
};

static void updateGlobalBest(Progress* progress, unsigned int score)
{
    unsigned int current = progress->globalBest.load(std::memory_order_relaxed);
    while (score < current
           && !progress->globalBest.compare_exchange_weak(current, score, std::memory_order_relaxed))
    {
    }
}

// One independent restart: own rng and initial topology, the same shared LUT, a reused scorer.
static void runClimb(uint64_t restartSeed, TopoScorer* scorer,
                     uint64_t maxSteps, uint64_t patience, uint32_t maxUnreachable,
                     Progress* progress, ClimbResult& result)
{
    std::mt19937_64 rng(restartSeed);
    Topology topo;
    if (!initTopology(topo, rng, maxUnreachable))
    {
        return;
    }

    std::vector<unsigned char> topoBlock;
    serializeTopo(topo, topoBlock);
    unsigned int best = topoScorerScore(scorer, topoBlock.data());
    const unsigned int initial = best;
    updateGlobalBest(progress, best);

    uint64_t sinceImprovement = 0;
    uint64_t accepted = 0;
    uint64_t rejectedReach = 0;
    for (uint64_t step = 1; step <= maxSteps && sinceImprovement < patience; ++step)
    {
        progress->steps.fetch_add(1, std::memory_order_relaxed);
        uint32_t neuron;
        uint32_t slot;
        const uint32_t previous = mutateOneNeighbor(topo, rng, neuron, slot);

        if (!reachabilityOk(reachabilityFailures(topo), maxUnreachable))
        {
            topo.neighborIdx[(size_t)neuron * K + slot] = previous;
            ++rejectedReach;
            continue;
        }

        serializeTopo(topo, topoBlock);
        const unsigned int score = topoScorerScore(scorer, topoBlock.data());
        if (score < best)
        {
            best = score;
            ++accepted;
            sinceImprovement = 0;
            updateGlobalBest(progress, best);
        }
        else
        {
            topo.neighborIdx[(size_t)neuron * K + slot] = previous;
            ++sinceImprovement;
        }
    }

    result.topo = topo;
    result.initial = initial;
    result.best = best;
    result.accepted = accepted;
    result.rejectedReach = rejectedReach;
    result.valid = true;
}

// Worker: one scorer instance, restarts pulled from the shared counter until none remain. The
// per-restart done flag releases the result to the main thread's reporter.
static void runWorker(std::atomic<uint32_t>* nextRestart, uint32_t restartCount, uint64_t seed,
                      const unsigned char* dataBlock, const unsigned char* lut,
                      uint64_t maxSteps, uint64_t patience, uint32_t maxUnreachable,
                      Progress* progress, std::atomic<uint8_t>* doneFlags,
                      std::vector<ClimbResult>* results)
{
    TopoScorer* scorer = topoScorerCreate(dataBlock, lut);
    if (scorer == nullptr)
    {
        return;
    }
    for (;;)
    {
        const uint32_t r = nextRestart->fetch_add(1);
        if (r >= restartCount)
        {
            break;
        }
        // Golden-ratio stride decorrelates the sub-seeds; a restart's stream depends only on the
        // main seed and its own index, so neither the thread count nor the restart count changes
        // existing restarts' results.
        const uint64_t restartSeed = seed ^ (0x9e3779b97f4a7c15ULL * (uint64_t)(r + 1));
        runClimb(restartSeed, scorer, maxSteps, patience, maxUnreachable, progress, (*results)[r]);
        doneFlags[r].store(1, std::memory_order_release);
        progress->done.fetch_add(1);
    }
    topoScorerDestroy(scorer);
}

static void toHexString(const unsigned char* bytes, unsigned int count, char* out)
{
    static const char digits[] = "0123456789abcdef";
    for (unsigned int i = 0; i < count; ++i)
    {
        out[2 * i] = digits[bytes[i] >> 4];
        out[2 * i + 1] = digits[bytes[i] & 15];
    }
    out[2 * count] = 0;
}

int main(int argc, char** argv)
{
    const char* inPath = nullptr;
    const char* outPath = "topo_mined.task";
    const char* seedArg = "random";
    uint64_t maxSteps = 20000;
    uint64_t patience = 2000;
    uint32_t maxUnreachable = 2;
    uint64_t selfcheckSamples = 32;
    uint32_t restartCount = 0;
    uint32_t threadCount = 1;

    std::vector<const char*> pos;
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--steps") == 0 && i + 1 < argc)
        {
            maxSteps = strtoull(argv[++i], nullptr, 10);
        }
        else if (strcmp(argv[i], "--patience") == 0 && i + 1 < argc)
        {
            patience = strtoull(argv[++i], nullptr, 10);
        }
        else if (strcmp(argv[i], "--max-unreachable") == 0 && i + 1 < argc)
        {
            maxUnreachable = (uint32_t)strtoul(argv[++i], nullptr, 10);
        }
        else if (strcmp(argv[i], "--selfcheck-samples") == 0 && i + 1 < argc)
        {
            selfcheckSamples = strtoull(argv[++i], nullptr, 10);
        }
        else if (strcmp(argv[i], "--restarts") == 0 && i + 1 < argc)
        {
            restartCount = (uint32_t)strtoul(argv[++i], nullptr, 10);
        }
        else if (strcmp(argv[i], "--threads") == 0 && i + 1 < argc)
        {
            threadCount = (uint32_t)strtoul(argv[++i], nullptr, 10);
            if (threadCount == 0)
            {
                threadCount = 1;
            }
        }
        else
        {
            pos.push_back(argv[i]);
        }
    }
    if (pos.empty())
    {
        printf("Usage: bpp9000_topo_miner <input.task> [out.task] [seed] [--steps N] [--patience M] [--max-unreachable U] [--selfcheck-samples S] [--restarts R] [--threads T]\n");
        return 1;
    }
    inPath = pos[0];
    if (pos.size() > 1)
    {
        outPath = pos[1];
    }
    if (pos.size() > 2)
    {
        seedArg = pos[2];
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
    std::mt19937_64 rng(seed);

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
    // Read the FULL data block - pairs beyond T are the holdout tail, preserved on write but
    // excluded from scoring and from the data hash.
    const uint64_t dataSize = task_file::dataBytes(N, M, header.numPairs);
    const uint64_t scoredDataSize = task_file::dataBytes(N, M, T);
    std::vector<unsigned char> dataBlock(dataSize);
    if (!task_file::readTaskFileBlock(inPath, sizeof(header) + topoBytes, dataBlock.data(), dataSize))
    {
        printf("Cannot read the data block from %s\n", inPath);
        return 1;
    }

    std::vector<unsigned char> lut((size_t)P * LUT_SIZE);
    for (auto& trit : lut)
    {
        trit = (unsigned char)(rng() % 3);
    }

    if (restartCount == 0)
    {
        restartCount = threadCount;
    }
    if (threadCount > restartCount)
    {
        threadCount = restartCount;
    }
    printf("scorer backend: %s | restarts %u | threads %u\n",
           topoScorerBackendName(), restartCount, threadCount);

#ifdef TOPO_HAVE_CORE_SIMD
    if (selfcheckSamples > 0)
    {
        // Score random sample topologies with the scalar reference and the SIMD backend; a single
        // mismatch means the backends diverged and no mining result can be trusted.
        auto checkMiner = std::make_unique<ProdMiner>();
        TopoScorer* checkScorer = topoScorerCreate(dataBlock.data(), lut.data());
        if (checkScorer == nullptr)
        {
            printf("Scorer init failed (bad data block)\n");
            return 1;
        }
        std::mt19937_64 checkRng(seed ^ 0x73656c6663686bULL);
        Topology checkTopo;
        std::vector<unsigned char> checkBlock;
        for (uint64_t s = 0; s < selfcheckSamples; ++s)
        {
            if (!initTopology(checkTopo, checkRng, maxUnreachable))
            {
                printf("self-check: could not generate sample %llu\n", (unsigned long long)s);
                return 1;
            }
            serializeTopo(checkTopo, checkBlock);
            const unsigned int refScore = evaluate(checkMiner.get(), checkBlock.data(), dataBlock.data(), lut.data());
            const unsigned int simdScore = topoScorerScore(checkScorer, checkBlock.data());
            if (refScore != simdScore)
            {
                printf("self-check FAILED at sample %llu: reference %u vs %s %u\n",
                       (unsigned long long)s, refScore, topoScorerBackendName(), simdScore);
                return 1;
            }
        }
        topoScorerDestroy(checkScorer);
        printf("self-check passed: %llu samples match\n", (unsigned long long)selfcheckSamples);
    }
#endif

    printf("seed %llu\n", (unsigned long long)seed);
    std::vector<ClimbResult> results(restartCount);
    std::atomic<uint32_t> nextRestart(0);
    Progress progress;
    std::unique_ptr<std::atomic<uint8_t>[]> doneFlags(new std::atomic<uint8_t>[restartCount]);
    for (uint32_t r = 0; r < restartCount; ++r)
    {
        doneFlags[r].store(0);
    }
    std::vector<std::thread> workers;
    for (uint32_t t = 0; t < threadCount; ++t)
    {
        workers.emplace_back(runWorker, &nextRestart, restartCount, seed, dataBlock.data(),
                             lut.data(), maxSteps, patience, maxUnreachable,
                             &progress, doneFlags.get(), &results);
    }

    // Reporter: workers publish through Progress/doneFlags; this loop owns stdout - one rolling
    // status line, plus one fixed line per finished restart.
    const auto startTime = std::chrono::steady_clock::now();
    std::vector<bool> reported(restartCount, false);
    auto drainAndReport = [&]() -> uint32_t
    {
        uint32_t doneCount = 0;
        for (uint32_t r = 0; r < restartCount; ++r)
        {
            if (doneFlags[r].load(std::memory_order_acquire) == 0)
            {
                continue;
            }
            ++doneCount;
            if (!reported[r])
            {
                reported[r] = true;
                if (results[r].valid)
                {
                    printf("\r[r%u] done | error %u -> %u | accepted %llu | unreachable %u          \n",
                           r, results[r].initial, results[r].best,
                           (unsigned long long)results[r].accepted,
                           reachabilityFailures(results[r].topo).evolutionFailures);
                }
                else
                {
                    printf("\r[r%u] failed (no initial topology within the reachability bound)     \n", r);
                }
            }
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - startTime).count();
        const unsigned int globalBest = progress.globalBest.load();
        char bestText[16];
        if (globalBest == 0xFFFFFFFFU)
        {
            snprintf(bestText, sizeof(bestText), "-");
        }
        else
        {
            snprintf(bestText, sizeof(bestText), "%u", globalBest);
        }
        printf("\r%llus | %u/%u restarts done | global best %s | steps %llu   ",
               (unsigned long long)elapsed, doneCount, restartCount, bestText,
               (unsigned long long)progress.steps.load(std::memory_order_relaxed));
        fflush(stdout);
        return doneCount;
    };
    while (drainAndReport() < restartCount)
    {
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    printf("\n");

    for (auto& worker : workers)
    {
        worker.join();
    }

    uint32_t bestRestart = restartCount;
    for (uint32_t r = 0; r < restartCount; ++r)
    {
        if (results[r].valid && (bestRestart == restartCount || results[r].best < results[bestRestart].best))
        {
            bestRestart = r;
        }
    }
    if (bestRestart == restartCount)
    {
        printf("No restart produced a result\n");
        return 1;
    }
    const ClimbResult& winner = results[bestRestart];
    const Topology& topo = winner.topo;
    const unsigned int initial = winner.initial;
    const unsigned int best = winner.best;
    const uint64_t accepted = winner.accepted;

    std::vector<unsigned char> topoBlock;
    serializeTopo(topo, topoBlock);
    KangarooTwelve(topoBlock.data(), (unsigned int)topoBlock.size(), header.topologyHash, task_file::DATA_HASH_SIZE);
    KangarooTwelve(dataBlock.data(), (unsigned int)scoredDataSize, header.dataHash, task_file::DATA_HASH_SIZE);
    if (!task_file::writeTaskFile(outPath, header, topoBlock.data(), topoBlock.size(), dataBlock.data(), dataBlock.size()))
    {
        printf("Cannot write %s\n", outPath);
        return 1;
    }

    char hexTopo[2 * task_file::DATA_HASH_SIZE + 1];
    char hexData[2 * task_file::DATA_HASH_SIZE + 1];
    toHexString(header.topologyHash, task_file::DATA_HASH_SIZE, hexTopo);
    toHexString(header.dataHash, task_file::DATA_HASH_SIZE, hexData);
    printf("done | best restart %u | error %u -> %u | accepted %llu mutations | unreachable %u | wrote %s\n",
           bestRestart, initial, best, (unsigned long long)accepted,
           reachabilityFailures(topo).evolutionFailures, outPath);
    printf("BPP9000_TOPOLOGY_HASH: %s\n", hexTopo);
    printf("BPP9000_DATA_HASH:     %s\n", hexData);
    return 0;
}
