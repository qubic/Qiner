// AVX-512 avalanche kernel: 64 lanes = perturbation scenarios, one vpermb per neuron per tick.
// Requires AVX512-VBMI. Must match fitnessScalar (miner self-checks).

#include <cstring>
#include <immintrin.h>

#include "bpp9000_params.h"
#include "avalanche_kernel.h"

namespace
{
using Cfg = bpp9000_params::ProdConfig;
constexpr unsigned int POP = (unsigned int)Cfg::populationThreshold;
constexpr unsigned int NEIGH = (unsigned int)Cfg::numberOfNeighbors;
constexpr unsigned int LUTSZ = 27;
constexpr unsigned int LANES = 64;

// One unperturbed run, scalar, matching the reference tick.
inline void baselineRun(unsigned char* state, const unsigned char* lut, const uint32_t* neighborIdx,
                        unsigned int numberOfTicks)
{
    unsigned char next[POP];
    for (unsigned int t = 0; t < numberOfTicks; ++t)
    {
        for (unsigned int n = 0; n < POP; ++n)
        {
            const unsigned int t0 = state[neighborIdx[n * NEIGH + 0]];
            const unsigned int t1 = state[neighborIdx[n * NEIGH + 1]];
            const unsigned int t2 = state[neighborIdx[n * NEIGH + 2]];
            next[n] = lut[n * LUTSZ + t0 + 3 * t1 + 9 * t2];
        }
        memcpy(state, next, POP);
    }
}
}  // namespace

namespace avalanche
{

uint64_t fitnessAvx512(const uint32_t* neighborIdx, const Trials& trials, unsigned int numberOfTicks)
{
    alignas(64) unsigned char bufA[POP * LANES];
    alignas(64) unsigned char bufB[POP * LANES];
    alignas(64) unsigned char lutRow[POP * LANES];

    const unsigned int numScenarios = 2 * POP;
    const unsigned int numBatches = (numScenarios + LANES - 1) / LANES;

    uint64_t sum = 0;
    unsigned char base[POP];
    unsigned char baseline[POP];

    for (unsigned int l = 0; l < trials.L; ++l)
    {
        const unsigned char* lut = trials.lut.data() + (size_t)l * POP * LUTSZ;
        const unsigned char* v0 = trials.v0.data() + (size_t)l * POP;

        memcpy(base, v0, POP);
        memcpy(baseline, base, POP);
        baselineRun(baseline, lut, neighborIdx, numberOfTicks);

        // Pack each 27-entry LUT into a 64-byte vpermb table row.
        memset(lutRow, 0, sizeof(lutRow));
        for (unsigned int n = 0; n < POP; ++n)
        {
            memcpy(&lutRow[n * LANES], &lut[n * LUTSZ], LUTSZ);
        }

        for (unsigned int b = 0; b < numBatches; ++b)
        {
            unsigned char* cur = bufA;
            unsigned char* nxt = bufB;

            // Broadcast V0 to all lanes, then perturb one neuron per lane to (base + z) % 3.
            for (unsigned int n = 0; n < POP; ++n)
            {
                _mm512_store_si512((void*)&cur[n * LANES], _mm512_set1_epi8((char)base[n]));
            }
            for (unsigned int lane = 0; lane < LANES; ++lane)
            {
                const unsigned int s = b * LANES + lane;
                if (s >= numScenarios)
                {
                    break;
                }
                const unsigned int neuron = s >> 1;
                const unsigned int z = (s & 1) + 1;
                cur[neuron * LANES + lane] = (unsigned char)((base[neuron] + z) % 3);
            }

            for (unsigned int t = 0; t < numberOfTicks; ++t)
            {
                for (unsigned int n = 0; n < POP; ++n)
                {
                    const __m512i a = _mm512_load_si512((const void*)&cur[neighborIdx[n * NEIGH + 0] * LANES]);
                    const __m512i bb = _mm512_load_si512((const void*)&cur[neighborIdx[n * NEIGH + 1] * LANES]);
                    const __m512i c = _mm512_load_si512((const void*)&cur[neighborIdx[n * NEIGH + 2] * LANES]);
                    // idx = a + 3*bb + 9*c per lane (<= 26).
                    const __m512i b3 = _mm512_add_epi8(_mm512_add_epi8(bb, bb), bb);
                    const __m512i c2 = _mm512_add_epi8(c, c);
                    const __m512i c4 = _mm512_add_epi8(c2, c2);
                    const __m512i c8 = _mm512_add_epi8(c4, c4);
                    const __m512i c9 = _mm512_add_epi8(c8, c);
                    const __m512i idx = _mm512_add_epi8(a, _mm512_add_epi8(b3, c9));
                    const __m512i tbl = _mm512_load_si512((const void*)&lutRow[n * LANES]);
                    _mm512_store_si512((void*)&nxt[n * LANES], _mm512_permutexvar_epi8(idx, tbl));
                }
                unsigned char* tmp = cur;
                cur = nxt;
                nxt = tmp;
            }

            const unsigned int lanesThisBatch =
                (b * LANES + LANES <= numScenarios) ? LANES : (numScenarios - b * LANES);
            for (unsigned int lane = 0; lane < lanesThisBatch; ++lane)
            {
                for (unsigned int n = 0; n < POP; ++n)
                {
                    if (cur[n * LANES + lane] != baseline[n])
                    {
                        ++sum;
                    }
                }
            }
        }
    }
    return sum;
}

uint64_t fitness(const uint32_t* neighborIdx, const Trials& trials, unsigned int numberOfTicks)
{
    return fitnessAvx512(neighborIdx, trials, numberOfTicks);
}

const char* backendName()
{
    return "avalanche-avx512";
}

}  // namespace avalanche
