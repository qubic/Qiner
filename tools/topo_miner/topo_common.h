#pragma once

// Shared topology + templated search scaffolding. An Evaluator provides
// operator()(topoBlock)->fitness, static worst(), static isBetter(cand,best).
// Self-loops and duplicate neighbors are allowed; input/output/signal placements stay distinct.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <thread>
#include <vector>

#include "bpp9000_params.h"
#include "task_file.h"

namespace topo_common
{

using Cfg = bpp9000_params::ProdConfig;
static constexpr uint32_t N = (uint32_t)Cfg::numberOfInputNeurons;
static constexpr uint32_t M = (uint32_t)Cfg::numberOfOutputNeurons;
static constexpr uint32_t K = (uint32_t)Cfg::numberOfNeighbors;
static constexpr uint32_t P = (uint32_t)Cfg::populationThreshold;
static constexpr uint64_t T = (uint64_t)Cfg::sequenceLength;
static constexpr uint32_t LUT_SIZE = 27; // trits per neuron LUT (3^numberOfNeighbors)

struct Topology
{
    std::vector<uint32_t> inputIdx;
    std::vector<uint32_t> outputIdx;
    uint32_t signalIdx;
    std::vector<uint32_t> neighborIdx;
};

// Neurons failing to reach both signal and an output, split into critical (input/output/signal) and evolution failures.
struct Reachability
{
    uint32_t criticalFailures;
    uint32_t evolutionFailures;
};

inline Reachability reachabilityFailures(const Topology& topo)
{
    // Influence traversal stops at input neurons (their LUT row is not evaluated by the scorer).
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

inline bool reachabilityOk(const Reachability& reach, uint32_t maxUnreachable)
{
    return reach.criticalFailures == 0 && reach.evolutionFailures <= maxUnreachable;
}

// Rewire one neighbor slot of a random neuron to a different random target (self/duplicate allowed).
inline uint32_t mutateOneNeighbor(Topology& topo, std::mt19937_64& rng, uint32_t& outNeuron, uint32_t& outSlot)
{
    outNeuron = (uint32_t)(rng() % P);
    outSlot = (uint32_t)(rng() % K);
    const uint32_t previous = topo.neighborIdx[(size_t)outNeuron * K + outSlot];

    uint32_t nb = (uint32_t)(rng() % P);
    while (nb == previous)
    {
        nb = (nb + 1) % P;
    }
    topo.neighborIdx[(size_t)outNeuron * K + outSlot] = nb;
    return previous;
}

// Random valid topology: distinct input/output/signal placements, within the reachability bound.
inline bool initTopology(Topology& topo, std::mt19937_64& rng, uint32_t maxUnreachable)
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

        // Any target allowed, including self (autapse) and repeats of another neighbor.
        for (uint32_t n = 0; n < P; ++n)
        {
            for (uint32_t k = 0; k < K; ++k)
            {
                topo.neighborIdx[(size_t)n * K + k] = (uint32_t)(rng() % P);
            }
        }

        if (reachabilityOk(reachabilityFailures(topo), maxUnreachable))
        {
            return true;
        }
    }
    return false;
}

inline void serializeTopo(const Topology& topo, std::vector<unsigned char>& block)
{
    block.resize((size_t)task_file::topologyBytes(N, M, P, K));
    task_file::serializeTopologyBlock(N, M, P, K, topo.inputIdx.data(), topo.outputIdx.data(),
                                      topo.signalIdx, topo.neighborIdx.data(), block.data());
}

inline void toHexString(const unsigned char* bytes, unsigned int count, char* out)
{
    static const char digits[] = "0123456789abcdef";
    for (unsigned int i = 0; i < count; ++i)
    {
        out[2 * i] = digits[bytes[i] >> 4];
        out[2 * i + 1] = digits[bytes[i] & 15];
    }
    out[2 * count] = 0;
}

struct RestartResult
{
    Topology topo;
    uint64_t initial = 0;
    uint64_t best = 0;
    uint64_t accepted = 0;
    uint64_t rejectedReach = 0;
    bool valid = false;
};

// Shared live counters the main thread renders as a rolling status line; workers never print.
struct Progress
{
    std::atomic<uint64_t> steps{0};
    std::atomic<uint64_t> globalBest{0};
    std::atomic<uint32_t> done{0};
};

template<typename Eval>
inline void updateGlobalBest(Progress* progress, uint64_t score)
{
    uint64_t current = progress->globalBest.load(std::memory_order_relaxed);
    while (Eval::isBetter(score, current)
           && !progress->globalBest.compare_exchange_weak(current, score, std::memory_order_relaxed))
    {
    }
}

// One independent restart: own rng and initial topology, a reused evaluator.
template<typename Eval>
inline void runRestart(uint64_t restartSeed, Eval& eval, uint64_t maxSteps, uint64_t patience,
                     uint32_t maxUnreachable, Progress* progress, RestartResult& result)
{
    std::mt19937_64 rng(restartSeed);
    Topology topo;
    if (!initTopology(topo, rng, maxUnreachable))
    {
        return;
    }

    std::vector<unsigned char> topoBlock;
    serializeTopo(topo, topoBlock);
    uint64_t best = eval(topoBlock.data());
    const uint64_t initial = best;
    updateGlobalBest<Eval>(progress, best);

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
        const uint64_t score = eval(topoBlock.data());
        if (Eval::isBetter(score, best))
        {
            best = score;
            ++accepted;
            sinceImprovement = 0;
            updateGlobalBest<Eval>(progress, best);
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

// Worker: one evaluator (from makeEval), restarts pulled from the shared counter until none remain.
template<typename Eval, typename MakeEval>
inline void runWorker(std::atomic<uint32_t>* nextRestart, uint32_t restartCount, uint64_t seed,
                      uint64_t maxSteps, uint64_t patience, uint32_t maxUnreachable,
                      Progress* progress, std::atomic<uint8_t>* doneFlags,
                      std::vector<RestartResult>* results, MakeEval makeEval)
{
    Eval eval = makeEval();
    for (;;)
    {
        const uint32_t r = nextRestart->fetch_add(1);
        if (r >= restartCount)
        {
            break;
        }
        // Golden-ratio stride: a restart depends only on the main seed and its index, not thread count.
        const uint64_t restartSeed = seed ^ (0x9e3779b97f4a7c15ULL * (uint64_t)(r + 1));
        runRestart<Eval>(restartSeed, eval, maxSteps, patience, maxUnreachable, progress, (*results)[r]);
        doneFlags[r].store(1, std::memory_order_release);
        progress->done.fetch_add(1);
    }
}

// Run restartCount restarts on threadCount workers, render status, return the best restart index.
template<typename Eval, typename MakeEval>
inline uint32_t runSearch(uint32_t threadCount, uint32_t restartCount, uint64_t seed,
                             uint64_t maxSteps, uint64_t patience, uint32_t maxUnreachable,
                             const char* scoreLabel, double scoreScale,
                             std::vector<RestartResult>& results, MakeEval makeEval)
{
    results.assign(restartCount, RestartResult{});
    Progress progress;
    progress.globalBest.store(Eval::worst());
    std::atomic<uint32_t> nextRestart(0);
    std::unique_ptr<std::atomic<uint8_t>[]> doneFlags(new std::atomic<uint8_t>[restartCount]);
    for (uint32_t r = 0; r < restartCount; ++r)
    {
        doneFlags[r].store(0);
    }

    std::vector<std::thread> workers;
    for (uint32_t t = 0; t < threadCount; ++t)
    {
        workers.emplace_back(runWorker<Eval, MakeEval>, &nextRestart, restartCount, seed,
                             maxSteps, patience, maxUnreachable, &progress, doneFlags.get(),
                             &results, makeEval);
    }

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
                    printf("\r[r%u] done | %s %.2f -> %.2f | accepted %llu | unreachable %u          \n",
                           r, scoreLabel, results[r].initial * scoreScale,
                           results[r].best * scoreScale, (unsigned long long)results[r].accepted,
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
        const uint64_t globalBest = progress.globalBest.load();
        char bestText[24];
        if (globalBest == Eval::worst())
        {
            snprintf(bestText, sizeof(bestText), "-");
        }
        else
        {
            snprintf(bestText, sizeof(bestText), "%.2f", globalBest * scoreScale);
        }
        printf("\r%llds | %u/%u restarts done | global best %s | steps %llu   ",
               (long long)elapsed, doneCount, restartCount, bestText,
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
        if (results[r].valid
            && (bestRestart == restartCount || Eval::isBetter(results[r].best, results[bestRestart].best)))
        {
            bestRestart = r;
        }
    }
    return bestRestart;
}

}  // namespace topo_common
