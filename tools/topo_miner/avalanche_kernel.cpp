// Scalar avalanche kernel (always compiled), the reference for the AVX-512 path.

#include <cstring>
#include <random>

#include "bpp9000_params.h"
#include "avalanche_kernel.h"

namespace
{
using Cfg = bpp9000_params::ProdConfig;
constexpr unsigned int POP = (unsigned int)Cfg::populationThreshold;
constexpr unsigned int NEIGH = (unsigned int)Cfg::numberOfNeighbors;
constexpr unsigned int LUTSZ = 27;

// One synchronous tick: every neuron next = lut[n][t0 + 3*t1 + 9*t2].
inline void processTick(unsigned char* state, unsigned char* next, const unsigned char* lut,
                        const uint32_t* neighborIdx)
{
    for (unsigned int n = 0; n < POP; ++n)
    {
        const unsigned int t0 = state[neighborIdx[n * NEIGH + 0]];
        const unsigned int t1 = state[neighborIdx[n * NEIGH + 1]];
        const unsigned int t2 = state[neighborIdx[n * NEIGH + 2]];
        next[n] = lut[n * LUTSZ + t0 + 3 * t1 + 9 * t2];
    }
}

inline void runTickSimulation(unsigned char* state, const unsigned char* lut,
                              const uint32_t* neighborIdx, unsigned int numberOfTicks)
{
    unsigned char next[POP];
    for (unsigned int t = 0; t < numberOfTicks; ++t)
    {
        processTick(state, next, lut, neighborIdx);
        memcpy(state, next, POP);
    }
}
}  // namespace

namespace avalanche
{

Trials makeTrials(uint64_t seed, unsigned int L)
{
    Trials trials;
    trials.L = L;
    trials.lut.resize((size_t)L * POP * LUTSZ);
    trials.v0.resize((size_t)L * POP);
    std::mt19937_64 rng(seed);
    for (auto& t : trials.lut)
    {
        t = (unsigned char)(rng() % 3);
    }
    for (auto& t : trials.v0)
    {
        t = (unsigned char)(rng() % 3);
    }
    return trials;
}

uint64_t fitnessScalar(const uint32_t* neighborIdx, const Trials& trials, unsigned int numberOfTicks)
{
    uint64_t sum = 0;
    unsigned char base[POP];
    unsigned char baseline[POP];
    unsigned char st[POP];
    for (unsigned int l = 0; l < trials.L; ++l)
    {
        const unsigned char* lut = trials.lut.data() + (size_t)l * POP * LUTSZ;
        const unsigned char* v0 = trials.v0.data() + (size_t)l * POP;

        memcpy(base, v0, POP);
        memcpy(baseline, base, POP);
        runTickSimulation(baseline, lut, neighborIdx, numberOfTicks);

        for (unsigned int n = 0; n < POP; ++n)
        {
            for (unsigned char z = 1; z <= 2; ++z)  // cyclic +1 / +2 = the two other trit values
            {
                memcpy(st, base, POP);
                st[n] = (unsigned char)((base[n] + z) % 3);
                runTickSimulation(st, lut, neighborIdx, numberOfTicks);
                for (unsigned int m = 0; m < POP; ++m)
                {
                    if (st[m] != baseline[m])
                    {
                        ++sum;
                    }
                }
            }
        }
    }
    return sum;
}

#ifndef TOPO_HAVE_AVALANCHE_AVX512
uint64_t fitness(const uint32_t* neighborIdx, const Trials& trials, unsigned int numberOfTicks)
{
    return fitnessScalar(neighborIdx, trials, numberOfTicks);
}

const char* backendName()
{
    return "avalanche-scalar";
}
#endif

}  // namespace avalanche
