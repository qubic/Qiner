#pragma once

#include "score_common.h"
#include "rating.h"
#include "K12AndKeyUtil.h"
#include "task_file.h"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

// bpp9000 scorer: autonomous recurrent ternary network (trits {0,1,2}, 2 = UNKNOWN)
namespace score_bpp9000
{

static constexpr unsigned long long NUMBER_OF_INPUT_NEURONS = 18;
static constexpr unsigned long long NUMBER_OF_OUTPUT_NEURONS = 1;
static constexpr unsigned long long POPULATION_THRESHOLD = 2048;
static constexpr unsigned long long NUMBER_OF_NEIGHBORS = 3;
static constexpr unsigned long long NUMBER_OF_MUTATIONS = 1000;
static constexpr unsigned long long MAX_NUMBER_OF_TICKS = 100000;

static constexpr unsigned int MAX_CHANGES_PER_STEP = 10;
static constexpr unsigned long long SEQUENCE_LENGTH = 24 * 365;
static constexpr unsigned long long WINDOW_WIDTH = 24 * 28;
static constexpr unsigned long long NUMBER_OF_WINDOWS = SEQUENCE_LENGTH - WINDOW_WIDTH;

// How far the frame may slide: production predicts one week ahead.
static constexpr unsigned long long SHIFT_CAP = 24 * 7;

// Frame-0 floor. The score is an error inside ONE frame, so it lands in [0, WINDOW_WIDTH].
static constexpr unsigned int SOLUTION_THRESHOLD = (unsigned int)(WINDOW_WIDTH * 45 / 100);

// Mutation modes, declared by the miner in nonce[1].
static constexpr unsigned char BPP9000_MODE_START = 1;    // mutate the start state
static constexpr unsigned char BPP9000_MODE_WIRING = 2;   // mutate the wiring
static constexpr unsigned char BPP9000_MODE_LUT = 3;      // mutate the LUTs

template <
    unsigned long long numberOfInputNeurons,
    unsigned long long numberOfOutputNeurons,
    unsigned long long sequenceLength,
    unsigned long long windowWidth,
    unsigned long long maxNumberOfTicks,
    unsigned long long numberOfNeighbors,
    unsigned long long populationThreshold,
    unsigned long long numberOfMutations,
    unsigned int solutionThreshold,
    unsigned long long shiftCapParam>
struct Miner
{
    static constexpr unsigned long long maxNumberOfNeurons = populationThreshold;
    // Number of graded emits the network must produce.
    static constexpr unsigned long long numberOfWindows = sequenceLength - windowWidth;

    static constexpr unsigned long long topoBlockSize =
        (numberOfInputNeurons + numberOfOutputNeurons + 1 + populationThreshold * numberOfNeighbors) * sizeof(uint32_t);
    static constexpr unsigned long long dataBlockSize =
        sequenceLength * (((numberOfInputNeurons + task_file::TRITS_PER_BYTE - 1) / task_file::TRITS_PER_BYTE)
                          + ((numberOfOutputNeurons + task_file::TRITS_PER_BYTE - 1) / task_file::TRITS_PER_BYTE));

    static constexpr unsigned char TRIT_UNKNOWN = 2;
    static constexpr unsigned int INFINITE_ERROR = 0xFFFFFFFFU;
    static constexpr unsigned int INVALID_SCORE_VALUE = 0xFFFFFFFFU;   // non-canonical ant nonce
    static constexpr unsigned long long lutSize = 27;   // 3^numberOfNeighbors, base-3 index t0 + 3*t1 + 9*t2

    // Rolling-frame scoring, derived from the frame width so they scale with any config.
    // advanceThreshold: shift advances when the frame error drops to <= 1/3 of the frame.
    static constexpr unsigned int advanceThreshold = (unsigned int)(windowWidth / 3);
    // shiftCap: how far the frame may slide. Production predicts one week ahead, so 24 * 7.
    static constexpr unsigned long long shiftCap = shiftCapParam;

    static constexpr unsigned long long numberOfLinks = populationThreshold * numberOfNeighbors;

    static_assert(numberOfNeighbors == 3, "the LUT index is hardcoded for 3 neighbors");
    static_assert(populationThreshold % 16 == 0, "populationThreshold must be a multiple of 16 so sizeof(RootMaterial) stays a multiple of 64 for the random2 draw");
    static_assert(numberOfOutputNeurons == 1, "score() grades only output neuron 0");
    static_assert(numberOfWindows >= 1 && numberOfWindows < sequenceLength, "the frame must leave targets after it");
    static_assert(shiftCap >= 1 && shiftCap <= numberOfWindows, "shiftCap must be positive and keep the last frame inside the data");
    static_assert(maxNumberOfTicks > shiftCap + windowWidth, "maxNumberOfTicks must exceed the deepest emit count so all emits can fit");
    static_assert(advanceThreshold < windowWidth, "the advance gate must be reachable inside one frame");
    // The frame-0 floor sits between the advance gate and the frame width.
    static_assert(solutionThreshold > advanceThreshold && solutionThreshold < windowWidth, "the frame-0 floor must admit some root and still reject the worst");
    static_assert(populationThreshold <= 65536, "ANN.neighbor is a 16-bit transfer index");

    // Per-identity root material, drawn from the pubkey seed. Trits are one byte each (read as bytes); each
    // wiring link is one unsigned long long, read back at the same width random2 writes and reduced % population.
    // control and output are NOT here - they are global (deriveControlOutput) and never mutate.
    struct RootMaterial
    {
        unsigned char lut[maxNumberOfNeurons * lutSize];
        unsigned char start[maxNumberOfNeurons];
        unsigned long long wire[numberOfLinks];
    };
    static_assert(sizeof(RootMaterial)
            == maxNumberOfNeurons * lutSize + maxNumberOfNeurons + numberOfLinks * sizeof(unsigned long long),
        "RootMaterial must be padding-free");
    static_assert(sizeof(RootMaterial) % 64 == 0, "root-material draw must be 64-byte aligned for random2");
    static constexpr unsigned long long mutationSeedCount = numberOfMutations * MAX_CHANGES_PER_STEP;
    // The draw is rounded up to a whole 64-byte block for random2; the walk reads only the first mutationSeedCount.
    static constexpr unsigned long long mutationSeedPaddedCount =
        ((mutationSeedCount * sizeof(unsigned long long) + 63) / 64) * 64 / sizeof(unsigned long long);

    // The exchanged form: byte-identical to core's ANN. Only the per-identity, mutable parts - wiring
    // (uint16 per link), the start state (one trit per neuron), the LUTs (one trit per byte). control and
    // output are global (same for every identity) and derived separately.
    struct ANN
    {
        unsigned short neighbor[numberOfLinks];
        unsigned char initialNeuronValues[maxNumberOfNeurons];
        unsigned char lut[maxNumberOfNeurons * lutSize];
    };
    static_assert(sizeof(ANN) == numberOfLinks * sizeof(unsigned short) + maxNumberOfNeurons + maxNumberOfNeurons * lutSize,
        "ANN must be padding-free");

    // The random2 pool the scorer reads.
    // initialize()/initializeFromMemory() point it at the owned poolVec; setPool() points it at a
    // caller-owned pool shared read-only across threads (the ant-colony path builds it once and shares it).
    std::vector<unsigned char> poolVec;
    const unsigned char* pRandom2Pool = nullptr;

    void setPool(const unsigned char* pool)
    {
        pRandom2Pool = pool;
    }

    // Task data.
    unsigned char topoBlockBuf[topoBlockSize];
    unsigned char dataBlockBuf[dataBlockSize];
    unsigned char inputs[sequenceLength][numberOfInputNeurons];   // filled by unpackDataBlock, unused
    unsigned char outputs[sequenceLength][numberOfOutputNeurons]; // the target output sequence
    // Self-clock and graded neuron: global for the epoch (deriveControlOutput), never mutated.
    uint32_t controlIndex;
    uint32_t outputIndex;

    // Working state the walk mutates and scores.
    unsigned char curInitial[maxNumberOfNeurons];
    uint32_t neighborIndices[numberOfLinks];
    unsigned char curLut[maxNumberOfNeurons * lutSize];

    unsigned char prevInitial[maxNumberOfNeurons];
    uint32_t prevNeighborIndices[numberOfLinks];
    unsigned char prevLut[maxNumberOfNeurons * lutSize];
    unsigned char bestInitial[maxNumberOfNeurons];
    uint32_t bestNeighborIndices[numberOfLinks];
    unsigned char bestLut[maxNumberOfNeurons * lutSize];

    // Rolling-frame position: score() grades [shift, shift+windowWidth). Holds the committed shift
    // once the walk returns.
    unsigned long long shift = 0;

    // Score scratch buffers.
    unsigned char neuronOut[maxNumberOfNeurons];
    unsigned char neuronPrev[maxNumberOfNeurons];

    // Per-identity root material (from the pubkey seed).
    RootMaterial rootMaterial;
    // The walk's mutation seeds (from pubkey + nonce + anchor).
    unsigned long long mutationSeed[mutationSeedPaddedCount];

    // Task loading.
    bool initialize(unsigned char* miningSeed, const char* taskFilePath)
    {
        poolVec.resize(POOL_VEC_PADDING_SIZE);
        generateRandom2Pool(miningSeed, poolVec.data());
        pRandom2Pool = poolVec.data();
        deriveControlOutput(miningSeed);
        return loadTaskData(taskFilePath);
    }

    bool initializeFromMemory(unsigned char* miningSeed, const unsigned char* topoBlock, const unsigned char* dataBlock)
    {
        poolVec.resize(POOL_VEC_PADDING_SIZE);
        generateRandom2Pool(miningSeed, poolVec.data());
        pRandom2Pool = poolVec.data();
        deriveControlOutput(miningSeed);
        return loadTaskFromMemory(topoBlock, dataBlock);
    }

    // Load + validate the unified task file: check dims, read each block, verify its hash, parse.
    bool loadTaskData(const char* taskFilePath)
    {
        task_file::TaskFileHeader header;
        if (!task_file::readTaskFileHeader(taskFilePath, &header))
        {
            return false;
        }
        if (header.magic != task_file::MAGIC ||
            header.version != task_file::VERSION ||
            header.numInputTrits != numberOfInputNeurons ||
            header.numOutputTrits != numberOfOutputNeurons ||
            header.numPairs < sequenceLength
#if BPP9000_TASK_HAS_TOPOLOGY
            || header.population != populationThreshold
            || header.numNeighbors != numberOfNeighbors
#endif
            )
        {
            return false;
        }

        unsigned char hash[task_file::DATA_HASH_SIZE];
        // Size the topology skip from the file's own header, so a task written at any population loads.
        const unsigned long long topoBytes = task_file::topologyBytes(
            header.numInputTrits, header.numOutputTrits, header.population, header.numNeighbors);

#if BPP9000_TASK_HAS_TOPOLOGY
        if (!task_file::readTaskFileBlock(taskFilePath, sizeof(task_file::TaskFileHeader), topoBlockBuf, topoBytes))
        {
            return false;
        }
        KangarooTwelve(topoBlockBuf, (unsigned int)topoBytes, hash, task_file::DATA_HASH_SIZE);
        if (memcmp(hash, header.topologyHash, task_file::DATA_HASH_SIZE) != 0)
        {
            return false;
        }
#endif

        if (!task_file::readTaskFileBlock(taskFilePath, sizeof(task_file::TaskFileHeader) + topoBytes, dataBlockBuf, dataBlockSize))
        {
            return false;
        }
        KangarooTwelve(dataBlockBuf, (unsigned int)dataBlockSize, hash, task_file::DATA_HASH_SIZE);
        if (memcmp(hash, header.dataHash, task_file::DATA_HASH_SIZE) != 0)
        {
            return false;
        }
        return loadTargetSequence(dataBlockBuf);
    }

    // In-memory task load: the network is derived per identity, so only the target output sequence is read
    // from the data block. The topology block is unused.
    bool loadTaskFromMemory(const unsigned char* topoBlock, const unsigned char* dataBlock)
    {
        (void)topoBlock;
        return loadTargetSequence(dataBlock);
    }

    // Keep only the output column of each row as the target sequence; the input column is skipped.
    bool loadTargetSequence(const unsigned char* dataBlock)
    {
        if (!task_file::unpackDataBlock(numberOfInputNeurons, numberOfOutputNeurons, sequenceLength, dataBlock, &inputs[0][0], &outputs[0][0]))
        {
            return false;
        }
        return true;
    }

    bool validateTopology()
    {
        if (controlIndex >= populationThreshold || outputIndex >= populationThreshold || controlIndex == outputIndex)
        {
            return false;
        }
        for (unsigned long long i = 0; i < numberOfLinks; ++i)
        {
            if (neighborIndices[i] >= populationThreshold)
            {
                return false;
            }
        }
        return true;
    }

    // nonce[1] layout: bits 0-3 = L in [1, MAX_CHANGES_PER_STEP], bits 4-5 = mode in [1, 3], bits 6-7 = 0.
    static unsigned int changesPerStep(const unsigned char* nonce)
    {
        return nonce[1] & 0x0F;
    }
    static unsigned char modeOf(const unsigned char* nonce)
    {
        return (unsigned char)((nonce[1] >> 4) & 0x03);
    }

    static bool isCanonicalNonceCommon(const unsigned char* nonce)
    {
        const unsigned int L = nonce[1] & 0x0F;
        const unsigned char mode = (unsigned char)((nonce[1] >> 4) & 0x03);
        return (nonce[0] == AlgoType::Bpp9000)
            && (L >= 1)
            && (L <= MAX_CHANGES_PER_STEP)
            && (mode >= BPP9000_MODE_START)
            && (mode <= BPP9000_MODE_LUT)
            && ((nonce[1] & 0xC0) == 0);
    }

    static bool isCanonicalStandaloneNonce(const unsigned char* nonce)
    {
        return isCanonicalNonceCommon(nonce) && (nonce[2] == 0);
    }

    static bool isCanonicalAntNonce(const unsigned char* nonce)
    {
        return isCanonicalNonceCommon(nonce) && (nonce[2] <= numberOfMutations);
    }

    // Emits shift+windowWidth outputs, grades only [shift, shift+windowWidth). Times out at maxNumberOfTicks.
    unsigned int score()
    {
        for (unsigned long long n = 0; n < populationThreshold; ++n)
        {
            neuronOut[n] = curInitial[n];
        }

        unsigned int failures = 0;
        unsigned long long counter = 0;
        unsigned long long ticks = 0;
        while (counter < shift + windowWidth)
        {
            if (++ticks >= maxNumberOfTicks)
            {
                return INFINITE_ERROR;
            }

            memcpy(neuronPrev, neuronOut, sizeof(neuronPrev));
            for (unsigned long long n = 0; n < populationThreshold; ++n)
            {
                const unsigned long long t0 = neuronPrev[neighborIndices[n * numberOfNeighbors + 0]];
                const unsigned long long t1 = neuronPrev[neighborIndices[n * numberOfNeighbors + 1]];
                const unsigned long long t2 = neuronPrev[neighborIndices[n * numberOfNeighbors + 2]];
                neuronOut[n] = curLut[n * lutSize + (t0 + 3 * t1 + 9 * t2)];
            }

            if (neuronOut[controlIndex] != TRIT_UNKNOWN)
            {
                if (counter >= shift && neuronOut[outputIndex] != outputs[counter][0])
                {
                    failures++;
                }
                counter++;
            }
        }
        return failures;
    }

    // Mutations, one seed per change.
    void mutateStartState(unsigned long long mutationSeedValue)
    {
        const unsigned long long delta = mutationSeedValue & 1ULL;
        const unsigned long long n = (mutationSeedValue >> 1) % populationThreshold;
        curInitial[n] = (unsigned char)((curInitial[n] + 1 + delta) % 3);
    }

    void mutateWiring(unsigned long long mutationSeedValue)
    {
        const unsigned long long flatSlot = mutationSeedValue % numberOfLinks;
        uint32_t target = (uint32_t)((mutationSeedValue / numberOfLinks) % populationThreshold);
        const uint32_t previous = neighborIndices[flatSlot];
        while (target == previous)
        {
            target = (uint32_t)((target + 1) % populationThreshold);
        }
        neighborIndices[flatSlot] = target;
    }

    void mutateLut(unsigned long long mutationSeedValue)
    {
        const unsigned long long delta = mutationSeedValue & 1ULL;
        const unsigned long long flatIdx = (mutationSeedValue >> 1) % (maxNumberOfNeurons * lutSize);
        curLut[flatIdx] = (unsigned char)((curLut[flatIdx] + 1 + delta) % 3);
    }

    // Apply one mutation of the declared mode. Mode is 1, 2 or 3 only; a non-canonical nonce (any other
    // mode) is rejected before scoring, so no other value reaches here and there is no fallback.
    void mutate(unsigned char mode, unsigned long long mutationSeedValue)
    {
        if (mode == BPP9000_MODE_START)
        {
            mutateStartState(mutationSeedValue);
        }
        else if (mode == BPP9000_MODE_WIRING)
        {
            mutateWiring(mutationSeedValue);
        }
        else if (mode == BPP9000_MODE_LUT)
        {
            mutateLut(mutationSeedValue);
        }
    }

    // Fill the root material (LUTs, start state, wiring) from the pubkey seed in one draw.
    void deriveRootMaterial(const unsigned char* seed)
    {
        unsigned char rootHash[32];
        KangarooTwelve(seed, 32, rootHash, 32);
        random2(rootHash, pRandom2Pool, (unsigned char*)&rootMaterial, sizeof(rootMaterial));
    }

    // Mutation-walk seeds. nonce[0..2] (algo/L/mode/K) are zeroed out of the hash so they do not reseed the
    // walk; anchorTickDigest is null for standalone, the anchor tick for an ant child.
    void deriveMutationSeeds(const unsigned char* publicKey, const unsigned char* nonce, const unsigned char* anchorTickDigest)
    {
        unsigned char searchHash[32];
        unsigned char combined[96];
        memcpy(combined, publicKey, 32);
        memcpy(combined + 32, nonce, 32);
        combined[32] = 0;
        combined[33] = 0;
        combined[34] = 0;
        unsigned int combinedSize = 64;
        if (anchorTickDigest != nullptr)
        {
            memcpy(combined + 64, anchorTickDigest, 32);
            combinedSize = 96;
        }
        KangarooTwelve(combined, combinedSize, searchHash, 32);
        random2(searchHash, pRandom2Pool, (unsigned char*)&mutationSeed, sizeof(mutationSeed));
    }

    // The control and output neurons: global for the epoch, drawn from the digest alone so every identity
    // shares them. Folded into initialize()/initializeFromMemory(); call once before any score.
    void deriveControlOutput(const unsigned char* digest)
    {
        unsigned char seedHash[32];
        KangarooTwelve(digest, 32, seedHash, 32);
        unsigned long long material[8];   // 64-byte minimum random2 draw; only [0] and [1] are used
        random2(seedHash, pRandom2Pool, (unsigned char*)material, sizeof(material));
        controlIndex = (uint32_t)(material[0] % populationThreshold);
        uint32_t output = (uint32_t)(material[1] % populationThreshold);
        if (output == controlIndex)
        {
            output = (uint32_t)((output + 1) % populationThreshold);
        }
        outputIndex = output;
    }

    // Load the root material into the working state (LUTs, start state, wiring), absolute by neuron index.
    void applyRootMaterial()
    {
        for (unsigned long long i = 0; i < maxNumberOfNeurons * lutSize; ++i)
        {
            curLut[i] = (unsigned char)(rootMaterial.lut[i] % 3);
        }
        for (unsigned long long n = 0; n < populationThreshold; ++n)
        {
            curInitial[n] = (unsigned char)(rootMaterial.start[n] % 3);
        }
        for (unsigned long long i = 0; i < numberOfLinks; ++i)
        {
            neighborIndices[i] = (uint32_t)(rootMaterial.wire[i] % populationThreshold);
        }
    }

    void compact(ANN& out) const
    {
        for (unsigned long long i = 0; i < numberOfLinks; ++i)
        {
            out.neighbor[i] = (unsigned short)neighborIndices[i];
        }
        memcpy(out.initialNeuronValues, curInitial, sizeof(out.initialNeuronValues));
        memcpy(out.lut, curLut, sizeof(out.lut));
    }

    void expand(const ANN& src)
    {
        for (unsigned long long i = 0; i < numberOfLinks; ++i)
        {
            neighborIndices[i] = src.neighbor[i];
        }
        memcpy(curInitial, src.initialNeuronValues, sizeof(curInitial));
        memcpy(curLut, src.lut, sizeof(curLut));
    }

    void getBestANN(ANN& out) const
    {
        for (unsigned long long i = 0; i < numberOfLinks; ++i)
        {
            out.neighbor[i] = (unsigned short)bestNeighborIndices[i];
        }
        memcpy(out.initialNeuronValues, bestInitial, sizeof(out.initialNeuronValues));
        memcpy(out.lut, bestLut, sizeof(out.lut));
    }

    void snapshotPrev()
    {
        memcpy(prevInitial, curInitial, sizeof(prevInitial));
        memcpy(prevNeighborIndices, neighborIndices, sizeof(prevNeighborIndices));
        memcpy(prevLut, curLut, sizeof(prevLut));
    }

    void rollbackPrev()
    {
        memcpy(curInitial, prevInitial, sizeof(curInitial));
        memcpy(neighborIndices, prevNeighborIndices, sizeof(neighborIndices));
        memcpy(curLut, prevLut, sizeof(curLut));
    }

    void snapshotBest()
    {
        memcpy(bestInitial, curInitial, sizeof(bestInitial));
        memcpy(bestNeighborIndices, neighborIndices, sizeof(bestNeighborIndices));
        memcpy(bestLut, curLut, sizeof(bestLut));
    }

    // Scores the frame at the current shift and advances while the network masters it. Stops when it
    // cannot master a frame, or at shiftCap. Returns the rating reached.
    Rating advanceShift()
    {
        for (;;)
        {
            const unsigned int frameError = score();
            if (frameError > advanceThreshold)
            {
                return Rating{ frameError, (unsigned int)shift };   // cannot master this frame
            }
            if (shift == shiftCap)
            {
                return Rating{ frameError, (unsigned int)shift };   // the cap
            }
            shift++;
        }
    }

    // L mutations/step, explore for K steps then exploit, one-step rollback of the network and shift.
    // Returns the best-ever rating, leaving that network in best*.
    Rating computeScoreFromCurrent(unsigned int L, unsigned long long K, unsigned char mode)
    {
        Rating cur = advanceShift();
        Rating best = cur;
        snapshotBest();

        for (unsigned long long s = 0; s < numberOfMutations; ++s)
        {
            snapshotPrev();
            const unsigned long long prevShift = shift;

            for (unsigned int i = 0; i < L; ++i)
            {
                mutate(mode, mutationSeed[s * MAX_CHANGES_PER_STEP + i]);
            }

            const Rating r = advanceShift();

            // A timed-out rollout is never accepted.
            if (s < K)
            {
                // Anti-attractor: worse-or-equal error only; records nothing.
                if (r.isValid() && r.errorWorseOrEqual(cur))
                {
                    cur = r;
                }
                else
                {
                    rollbackPrev();
                    shift = prevShift;
                }
            }
            else
            {
                // Takes anything no worse than where the walk stands.
                if (r.isValid() && r.isNotWorseThan(cur))
                {
                    cur = r;
                }
                else
                {
                    rollbackPrev();
                    shift = prevShift;
                }

                // Records on the same test as the accept above.
                if (cur.isValid() && cur.isNotWorseThan(best))
                {
                    best = cur;
                    snapshotBest();
                }
            }
        }

        shift = best.shift;
        return best;
    }

    // Standalone: root from the pubkey, walk with K = 0 (no explore).
    Rating computeScore(unsigned char* publicKey, unsigned char* nonce)
    {
        const unsigned int L = changesPerStep(nonce);
        const unsigned char mode = modeOf(nonce);

        deriveRootMaterial(publicKey);
        deriveMutationSeeds(publicKey, nonce, nullptr);
        applyRootMaterial();

        shift = 0;   // standalone has no parent, so the frame starts at the first window
        return computeScoreFromCurrent(L, 0, mode);
    }

    bool findSolution(unsigned char* publicKey, unsigned char* nonce, unsigned int& outScore)
    {
        const Rating rating = computeScore(publicKey, nonce);
        outScore = rating.error;
        return rating.isValid() && rating.clearsFloor(solutionThreshold);
    }

    // Ant colony: the identity's root, each identity's tree starts from its own root.
    void deriveRootANN(const unsigned char* rootSeed, ANN& out)
    {
        deriveRootMaterial(rootSeed);
        applyRootMaterial();
        compact(out);
    }

    // Ant colony: inherit the parent's network and shift, then walk with the child's own seeds.
    Rating computeScoreFromParent(const ANN& parentANN, unsigned long long parentShift,
                                        const unsigned char* publicKey,
                                        const unsigned char* nonce, const unsigned char* anchorTickDigest)
    {
        if (!isCanonicalAntNonce(nonce) || parentShift > shiftCap)
        {
            return Rating::worst();
        }

        expand(parentANN);
        if (!validateTopology())
        {
            return Rating::worst();
        }
        deriveMutationSeeds(publicKey, nonce, anchorTickDigest);

        const unsigned int L = changesPerStep(nonce);
        const unsigned long long K = nonce[2];
        const unsigned char mode = modeOf(nonce);

        shift = parentShift;   // inherit; the root's children start at 0
        return computeScoreFromCurrent(L, K, mode);
    }
};

}
