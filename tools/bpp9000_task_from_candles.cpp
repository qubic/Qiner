// Build the bpp9000 task file from BTC hourly candles.
//
// Reads the CSV produced by fetch_btc_candles.py
//   delta d[j] = round(close[j+1]) - round(close[j])   (unit = integer USDT), saturated to 18-bit signed
//   input  (N=18) = d[j] as an 18-bit two's-complement signed integer, MSB-first, one bit per input neuron
//   output (M=1)  = sign of d[j]  (d >= 0 -> 1, d < 0 -> 0)
// Topology (placement + wiring) is generated deterministically from --topo-seed 
// Prints the KangarooTwelve topo/data hashes to pin in
// core/src/public_settings.h (BPP9000_TOPOLOGY_HASH / BPP9000_DATA_HASH).
//
// Usage:
//   bpp9000_task_from_candles <candles.csv> [out.task] [start-openms] [topo-seed]

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <random>
#include <algorithm>

#include "bpp9000_params.h"
#include "task_file.h"
#include "K12AndKeyUtil.h"
#include "score_common.h"

using Prod = bpp9000_params::ProdConfig;

static constexpr long long INT18_MIN = -131072;   // 18-bit two's-complement signed range
static constexpr long long INT18_MAX = 131071;

// Read (openTime, integer-USDT close) columns from the cache CSV.
static bool readCandles(const char* path, std::vector<long long>& openMs, std::vector<long long>& closes)
{
    std::ifstream in(path);
    if (!in)
    {
        printf("Cannot open %s\n", path);
        return false;
    }
    std::string line;
    bool header = true;
    while (std::getline(in, line))
    {
        if (header)   // skip "openTime,open,high,low,close,volume"
        {
            header = false;
            continue;
        }
        if (line.empty())
        {
            continue;
        }
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> cols;
        while (std::getline(ss, cell, ','))
        {
            cols.push_back(cell);
        }
        if (cols.size() < 5)
        {
            continue;
        }
        openMs.push_back(std::stoll(cols[0]));
        closes.push_back((long long)std::llround(std::stod(cols[4])));   // close is column 4
    }
    return true;
}

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

// Validate the generated topology
static bool checkTopology(uint32_t N, uint32_t M, uint32_t K, uint32_t P,
                          const std::vector<uint32_t>& inputIdx, const std::vector<uint32_t>& outputIdx,
                          uint32_t signalIdx, const std::vector<uint32_t>& neighborIdx)
{
    std::vector<bool> seen(P, false);
    auto place = [&](uint32_t idx, const char* what) -> bool
    {
        if (idx >= P)
        {
            printf("topology check: %s index %u out of range (P=%u)\n", what, idx, P);
            return false;
        }
        if (seen[idx])
        {
            printf("topology check: %s index %u overlaps another placement neuron\n", what, idx);
            return false;
        }
        seen[idx] = true;
        return true;
    };
    for (uint32_t i = 0; i < N; ++i)
    {
        if (!place(inputIdx[i], "input"))
        {
            return false;
        }
    }
    for (uint32_t j = 0; j < M; ++j)
    {
        if (!place(outputIdx[j], "output"))
        {
            return false;
        }
    }
    if (!place(signalIdx, "signal"))
    {
        return false;
    }

    for (uint32_t n = 0; n < P; ++n)
    {
        for (uint32_t k = 0; k < K; ++k)
        {
            const uint32_t nb = neighborIdx[(size_t)n * K + k];
            if (nb >= P)
            {
                printf("topology check: neuron %u neighbour %u out of range (P=%u)\n", n, nb, P);
                return false;
            }
            if (nb == n)
            {
                printf("topology check: neuron %u has a self-reference neighbour\n", n);
                return false;
            }
            for (uint32_t p = 0; p < k; ++p)
            {
                if (neighborIdx[(size_t)n * K + p] == nb)
                {
                    printf("topology check: neuron %u has a duplicate neighbour %u\n", n, nb);
                    return false;
                }
            }
        }
    }
    return true;
}

// Topology from a fixed seed. Returns false if generation
static uint32_t countAttendingNeurons(uint32_t N, uint32_t M, uint32_t K, uint32_t P,
                                      const std::vector<uint32_t>& inputIdx,
                                      const std::vector<uint32_t>& outputIdx,
                                      uint32_t signalIdx,
                                      const std::vector<uint32_t>& neighborIdx)
{
    std::vector<bool> reached(P, false);
    std::vector<bool> isInput(P, false);
    for (uint32_t i = 0; i < N; ++i)
    {
        isInput[inputIdx[i]] = true;
    }
    std::vector<uint32_t> frontier;
    frontier.push_back(signalIdx);
    for (uint32_t j = 0; j < M; ++j)
    {
        frontier.push_back(outputIdx[j]);
    }
    while (!frontier.empty())
    {
        const uint32_t n = frontier.back();
        frontier.pop_back();
        if (reached[n])
        {
            continue;
        }
        reached[n] = true;
        if (isInput[n])
        {
            continue;
        }
        for (uint32_t k = 0; k < K; ++k)
        {
            frontier.push_back(neighborIdx[(size_t)n * K + k]);
        }
    }
    uint32_t count = 0;
    for (uint32_t n = 0; n < P; ++n)
    {
        if (reached[n])
        {
            ++count;
        }
    }
    return count;
}

static unsigned long long measureTicksPerScore(const std::vector<unsigned char>& topoBlock,
                                               const std::vector<unsigned char>& dataBlock,
                                               const unsigned char* pool)
{
    const uint32_t N = (uint32_t)Prod::numberOfInputNeurons;
    const uint32_t M = (uint32_t)Prod::numberOfOutputNeurons;
    const uint32_t K = (uint32_t)Prod::numberOfNeighbors;
    const uint32_t P = (uint32_t)Prod::populationThreshold;
    const unsigned long long W = Prod::windowWidth;
    const unsigned long long numberOfWindows = Prod::sequenceLength - Prod::windowWidth;
    const unsigned char TRIT_UNKNOWN = 2;

    std::vector<uint32_t> inputIdx(N), outputIdx(M), neighborIdx((size_t)P * K);
    uint32_t signalIdx = 0;
    task_file::parseTopologyBlock(topoBlock.data(), N, M, P, K,
                                  inputIdx.data(), outputIdx.data(), &signalIdx, neighborIdx.data());

    const unsigned long long inBytes = task_file::packedBytes(N);
    const unsigned long long outBytes = task_file::packedBytes(M);
    std::vector<unsigned char> inputs((size_t)Prod::sequenceLength * N);
    std::vector<unsigned char> outputs((size_t)Prod::sequenceLength * M);
    for (unsigned long long r = 0; r < Prod::sequenceLength; ++r)
    {
        const unsigned char* row = dataBlock.data() + r * (inBytes + outBytes);
        task_file::unpackTrits(row, N, &inputs[r * N]);
        task_file::unpackTrits(row + inBytes, M, &outputs[r * M]);
    }

    unsigned char probeKey[32];
    memset(probeKey, 0x51, sizeof(probeKey));
    unsigned char rootHash[32];
    KangarooTwelve(probeKey, 32, rootHash, 32);
    const unsigned long long lutSize = 27;
    std::vector<unsigned char> lutRaw(((size_t)P * lutSize + 63) / 64 * 64);
    random2(rootHash, (unsigned char*)pool, lutRaw.data(), lutRaw.size());
    std::vector<unsigned char> lut((size_t)P * lutSize);
    for (size_t i = 0; i < lut.size(); ++i)
    {
        lut[i] = (unsigned char)(lutRaw[i] % 3);
    }

    std::vector<bool> isInput(P, false);
    for (uint32_t i = 0; i < N; ++i)
    {
        isInput[inputIdx[i]] = true;
    }
    std::vector<unsigned char> value(P), next(P);
    unsigned long long totalTicks = 0;
    for (unsigned long long win = 0; win < numberOfWindows; ++win)
    {
        std::fill(value.begin(), value.end(), TRIT_UNKNOWN);
        unsigned long long feedCounter = 0;
        unsigned long long tick = 0;
        for (; tick < Prod::maxNumberOfTicks; ++tick)
        {
            if (value[signalIdx] == TRIT_UNKNOWN)
            {
                if (feedCounter >= W)
                {
                    break;
                }
                for (uint32_t i = 0; i < N; ++i)
                {
                    value[inputIdx[i]] = inputs[(win + feedCounter) * N + i];
                }
                feedCounter++;
            }
            else
            {
                for (uint32_t i = 0; i < N; ++i)
                {
                    value[inputIdx[i]] = TRIT_UNKNOWN;
                }
            }
            for (uint32_t n = 0; n < P; ++n)
            {
                if (isInput[n])
                {
                    next[n] = value[n];
                    continue;
                }
                const unsigned long long t0 = value[neighborIdx[(size_t)n * K + 0]];
                const unsigned long long t1 = value[neighborIdx[(size_t)n * K + 1]];
                const unsigned long long t2 = value[neighborIdx[(size_t)n * K + 2]];
                next[n] = lut[n * lutSize + (t0 + 3 * t1 + 9 * t2)];
            }
            value.swap(next);
        }
        if (tick == Prod::maxNumberOfTicks)
        {
            return 0;
        }
        totalTicks += tick;
    }
    return totalTicks / numberOfWindows;
}

static bool buildTopology(std::vector<unsigned char>& topoBlock, uint64_t seed)
{
    const uint32_t N = (uint32_t)Prod::numberOfInputNeurons;
    const uint32_t M = (uint32_t)Prod::numberOfOutputNeurons;
    const uint32_t K = (uint32_t)Prod::numberOfNeighbors;
    const uint32_t P = (uint32_t)Prod::populationThreshold;

    std::mt19937_64 rng(seed);

    std::vector<uint32_t> inputIdx(N);
    std::vector<uint32_t> outputIdx(M);
    std::vector<uint32_t> neighborIdx((size_t)P * K);
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
        inputIdx[i] = pickDistinct();
    }
    for (uint32_t j = 0; j < M; ++j)
    {
        outputIdx[j] = pickDistinct();
    }
    const uint32_t signalIdx = pickDistinct();

    // Each neuron's K neighbours are distinct from each other and never the neuron itself (P >> K, so there
    // is always room; linear-probe from a random start keeps it deterministic).
    for (uint32_t n = 0; n < P; ++n)
    {
        for (uint32_t k = 0; k < K; ++k)
        {
            uint32_t nb = (uint32_t)(rng() % P);
            bool clash = true;
            while (clash)
            {
                clash = (nb == n);   // no self-reference
                for (uint32_t p = 0; p < k && !clash; ++p)
                {
                    if (neighborIdx[(size_t)n * K + p] == nb)   // no duplicate neighbour
                    {
                        clash = true;
                    }
                }
                if (clash)
                {
                    nb = (nb + 1) % P;
                }
            }
            neighborIdx[(size_t)n * K + k] = nb;
        }
    }

    if (!checkTopology(N, M, K, P, inputIdx, outputIdx, signalIdx, neighborIdx))
    {
        return false;
    }

    topoBlock.resize((size_t)task_file::topologyBytes(N, M, P, K));
    task_file::serializeTopologyBlock(N, M, P, K, inputIdx.data(), outputIdx.data(),
                                      signalIdx, neighborIdx.data(), topoBlock.data());
    return true;
}

// Encode the selected 8761 closes into the packed data block, per the spec.
static bool buildDataBlock(const std::vector<long long>& closes, size_t start,
                           std::vector<unsigned char>& dataBlock, long long& maxAbsDelta, size_t& saturated)
{
    const uint32_t N = (uint32_t)Prod::numberOfInputNeurons;   // 18
    const uint32_t M = (uint32_t)Prod::numberOfOutputNeurons;  // 1
    const uint64_t T = (uint64_t)Prod::sequenceLength;         // 8760 deltas

    if (start + T + 1 > closes.size())
    {
        printf("Not enough candles: need %llu from index %zu, have %zu\n",
               (unsigned long long)(T + 1), start, closes.size());
        return false;
    }

    std::vector<unsigned char> inputsBits((size_t)T * N);
    std::vector<unsigned char> outputsBits((size_t)T * M);
    maxAbsDelta = 0;
    saturated = 0;

    for (uint64_t j = 0; j < T; ++j)
    {
        long long d = closes[start + j + 1] - closes[start + j];   // integer USDT delta
        if (d > INT18_MAX) { d = INT18_MAX; ++saturated; }
        else if (d < INT18_MIN) { d = INT18_MIN; ++saturated; }
        const long long a = d < 0 ? -d : d;
        if (a > maxAbsDelta)
        {
            maxAbsDelta = a;
        }

        // 18-bit two's complement, MSB-first (bit 0 = most significant of the 18).
        const uint32_t v = (uint32_t)(d & 0x3FFFF);
        for (uint32_t b = 0; b < N; ++b)
        {
            inputsBits[(size_t)j * N + b] = (unsigned char)((v >> (17 - b)) & 1u);
        }
        outputsBits[(size_t)j * M] = (unsigned char)(d >= 0 ? 1 : 0);   // sign = predicted direction
    }

    dataBlock.resize((size_t)task_file::dataBytes(N, M, T));
    task_file::packDataBlock(N, M, T, inputsBits.data(), outputsBits.data(), dataBlock.data());
    return true;
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        printf("Usage: bpp9000_task_from_candles <candles.csv> [out.task] [start-openms] [topo-seed] [seed-tries]\n");
        return 1;
    }
    const char* csvPath = argv[1];
    const char* outPath = (argc > 2) ? argv[2] : "bpp9000.task";
    const long long startOpenMs = (argc > 3) ? std::stoll(argv[3]) : -1;
    const uint64_t topoSeed = (argc > 4) ? (uint64_t)std::stoull(argv[4]) : 0x62707039303030ULL; // "bpp9000"
    const uint64_t seedTries = (argc > 5) ? (uint64_t)std::stoull(argv[5]) : 1;

    std::vector<long long> openMs;
    std::vector<long long> closes;
    if (!readCandles(csvPath, openMs, closes))
    {
        return 1;
    }

    // Select the 8761-candle window: by openTime if given, else from the start. Verify contiguity.
    size_t start = 0;
    if (startOpenMs >= 0)
    {
        bool found = false;
        for (size_t i = 0; i < openMs.size(); ++i)
        {
            if (openMs[i] == startOpenMs)
            {
                start = i;
                found = true;
                break;
            }
        }
        if (!found)
        {
            printf("start-openms %lld not found in %s\n", startOpenMs, csvPath);
            return 1;
        }
    }
    const uint64_t T = (uint64_t)Prod::sequenceLength;
    if (start + T + 1 > closes.size())
    {
        printf("Not enough candles from the chosen start: need %llu, have %zu\n",
               (unsigned long long)(T + 1), closes.size() - start);
        return 1;
    }
    for (size_t i = start + 1; i <= start + T; ++i)
    {
        if (openMs[i] - openMs[i - 1] != 3600000LL)
        {
            printf("Candles are not contiguous at index %zu - re-run fetch_btc_candles.py\n", i);
            return 1;
        }
    }

    std::vector<unsigned char> topo;
    std::vector<unsigned char> data;

    long long maxAbsDelta = 0;
    size_t saturated = 0;
    if (!buildDataBlock(closes, start, data, maxAbsDelta, saturated))
    {
        return 1;
    }

    const uint32_t Nn = (uint32_t)Prod::numberOfInputNeurons;
    const uint32_t Mn = (uint32_t)Prod::numberOfOutputNeurons;
    const uint32_t Kn = (uint32_t)Prod::numberOfNeighbors;
    const uint32_t Pn = (uint32_t)Prod::populationThreshold;
    printf("Generating random2 probe pool for the topology sweep...\n");
    std::vector<unsigned char> pool(POOL_VEC_PADDING_SIZE);
    {
        unsigned char probeSeed[32];
        memset(probeSeed, 0x51, sizeof(probeSeed));
        generateRandom2Pool(probeSeed, pool.data());
    }
    uint64_t chosenSeed = 0;
    uint32_t chosenAttending = 0;
    unsigned long long chosenTicks = ~0ULL;
    std::vector<unsigned char> candidate;
    for (uint64_t t = 0; t < seedTries; ++t)
    {
        const uint64_t seed = topoSeed + t;
        candidate.clear();
        if (!buildTopology(candidate, seed))
        {
            printf("seed 0x%llx: rejected (self-reference or duplicate neighbours)\n",
                   (unsigned long long)seed);
            continue;
        }
        std::vector<uint32_t> inputIdx(Nn), outputIdx(Mn), neighborIdx((size_t)Pn * Kn);
        uint32_t signalIdx = 0;
        task_file::parseTopologyBlock(candidate.data(), Nn, Mn, Pn, Kn,
                                      inputIdx.data(), outputIdx.data(), &signalIdx, neighborIdx.data());
        const uint32_t attending = countAttendingNeurons(Nn, Mn, Kn, Pn, inputIdx, outputIdx,
                                                         signalIdx, neighborIdx);
        const unsigned long long ticks = measureTicksPerScore(candidate, data, pool.data());
        if (ticks == 0)
        {
            printf("seed 0x%llx: rejected (window timeout during the probe score)\n",
                   (unsigned long long)seed);
            continue;
        }
        printf("seed 0x%llx: %u/%u neurons attend the inference, %llu ticks per window\n",
               (unsigned long long)seed, attending, Pn, ticks);
        if (attending > chosenAttending ||
            (attending == chosenAttending && ticks < chosenTicks))
        {
            chosenAttending = attending;
            chosenTicks = ticks;
            chosenSeed = seed;
            topo = candidate;
        }
    }
    if (chosenTicks == ~0ULL)
    {
        printf("No usable topology in [0x%llx, 0x%llx) - aborting.\n",
               (unsigned long long)topoSeed, (unsigned long long)(topoSeed + seedTries));
        return 1;
    }
    printf("Chosen topo-seed 0x%llx: %u/%u neurons attend, %llu ticks per window.\n",
           (unsigned long long)chosenSeed, chosenAttending, Pn, chosenTicks);
    printf("Verification cost scales with ticks per window; neurons outside the attending set never influence the score.\n");
    if (saturated > 0)
    {
        printf("WARNING: %zu deltas saturated to the 18-bit range (|delta| > %lld). Pick a flatter window.\n",
               saturated, INT18_MAX);
    }

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

    if (!task_file::writeTaskFile(outPath, header, topo.data(), topo.size(), data.data(), data.size()))
    {
        printf("Failed to write %s\n", outPath);
        return 1;
    }

    char topoHex[2 * task_file::DATA_HASH_SIZE + 1];
    char dataHex[2 * task_file::DATA_HASH_SIZE + 1];
    toHex(header.topologyHash, task_file::DATA_HASH_SIZE, topoHex);
    toHex(header.dataHash, task_file::DATA_HASH_SIZE, dataHex);

    printf("Wrote %s (N=%u M=%u T=%llu P=%u K=%u, window %u, max|delta|=%lld USDT, topo-seed=0x%llx)\n",
           outPath, header.numInputTrits, header.numOutputTrits, (unsigned long long)header.numPairs,
           header.population, header.numNeighbors, (unsigned int)Prod::windowWidth,
           maxAbsDelta, (unsigned long long)chosenSeed);
    printf("BPP9000_TOPOLOGY_HASH = %s\n", topoHex);
    printf("BPP9000_DATA_HASH     = %s\n", dataHex);
    return 0;
}
