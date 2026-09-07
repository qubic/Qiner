#pragma once

// Avalanche fitness: perturbation spread over a fixed number of ticks, summed over neurons, both
// cyclic directions, and L random trials. Homogeneous graph, no roles, nothing held. Trits {0,1,2}.

#include <cstdint>
#include <vector>

namespace avalanche
{

// L random trials shared across all candidates. lut is L*P*LUT_SIZE trits; v0 is L*P trits.
struct Trials
{
    unsigned int L;
    std::vector<unsigned char> lut;
    std::vector<unsigned char> v0;
};

// Draw the L trials deterministically from seed.
Trials makeTrials(uint64_t seed, unsigned int L);

// Summed count of neurons whose final trit differs from baseline (reference backend).
uint64_t fitnessScalar(const uint32_t* neighborIdx, const Trials& trials, unsigned int numberOfTicks);

// The compiled backend (AVX-512 when built, else scalar); identical results.
uint64_t fitness(const uint32_t* neighborIdx, const Trials& trials, unsigned int numberOfTicks);

const char* backendName();

#ifdef TOPO_HAVE_AVALANCHE_AVX512
uint64_t fitnessAvx512(const uint32_t* neighborIdx, const Trials& trials, unsigned int numberOfTicks);
#endif

}  // namespace avalanche
