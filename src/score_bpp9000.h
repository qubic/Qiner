#pragma once

#include "score_common.h"
#include "K12AndKeyUtil.h"
#include "task_file.h"

#include <cassert>
#include <vector>

// bpp9000 scorer: a recurrent ternary ANN (trits {0,1,2}, 2 = UNKNOWN) predicting a windowed
// series; only the per-neuron LUTs change under mutation.
namespace score_bpp9000
{

static constexpr unsigned long long NUMBER_OF_INPUT_NEURONS = 18;
static constexpr unsigned long long NUMBER_OF_OUTPUT_NEURONS = 1;
static constexpr unsigned long long POPULATION_THRESHOLD = 64;
static constexpr unsigned long long NUMBER_OF_NEIGHBORS = 3;
static constexpr unsigned long long NUMBER_OF_MUTATIONS = 100;
static constexpr unsigned long long MAX_NUMBER_OF_TICKS = 100000;

static constexpr unsigned int MAX_LUT_ENTRIES_PER_STEP = 10;
static constexpr unsigned long long SEQUENCE_LENGTH = 24 * 365;
static constexpr unsigned long long WINDOW_WIDTH = 24 * 28;
static constexpr unsigned long long NUMBER_OF_WINDOWS = SEQUENCE_LENGTH - WINDOW_WIDTH;

// Placeholder,
// TODO: adjust this later
static constexpr unsigned int SOLUTION_THRESHOLD = (unsigned int)(((SEQUENCE_LENGTH - WINDOW_WIDTH) - 1) * 4 / 5);

template <
    unsigned long long numberOfInputNeurons,
    unsigned long long numberOfOutputNeurons,
    unsigned long long sequenceLength,
    unsigned long long windowWidth,
    unsigned long long maxNumberOfTicks,
    unsigned long long numberOfNeighbors,
    unsigned long long populationThreshold,
    unsigned long long numberOfMutations,
    unsigned int solutionThreshold>
struct Miner
{
    static constexpr unsigned long long maxNumberOfNeurons = populationThreshold;
    static constexpr unsigned long long numberOfWindows = sequenceLength - windowWidth;

    static constexpr unsigned long long topoBlockSize =
        (numberOfInputNeurons + numberOfOutputNeurons + 1 + populationThreshold * numberOfNeighbors) * sizeof(uint32_t);
    static constexpr unsigned long long dataBlockSize =
        sequenceLength * (((numberOfInputNeurons + task_file::TRITS_PER_BYTE - 1) / task_file::TRITS_PER_BYTE)
                          + ((numberOfOutputNeurons + task_file::TRITS_PER_BYTE - 1) / task_file::TRITS_PER_BYTE));

    static constexpr unsigned char TRIT_UNKNOWN = 2;

    static constexpr unsigned int INFINITE_ERROR = 0xFFFFFFFFU;
    static constexpr unsigned int INVALID_SCORE_VALUE = 0xFFFFFFFFU;   // non-canonical ant nonce

    static constexpr unsigned long long lutSize = 27;

    static_assert(
        numberOfNeighbors == 3,
        "the LUT index is hardcoded for 3 neighbours");
    static_assert(
        populationThreshold > numberOfInputNeurons + numberOfOutputNeurons + 1,
        "populationThreshold must leave room for the evolution neurons and the signal neuron");
    static_assert(
        (populationThreshold & (populationThreshold - 1)) == 0,
        "populationThreshold must be a power of 2");
    static_assert(
        windowWidth >= 2 && windowWidth < sequenceLength,
        "windowWidth must be at least 2 and leave room for the target after the window");
    static_assert(
        numberOfOutputNeurons == 1,
        "score() grades only output neuron 0");
    static_assert(
        maxNumberOfTicks > windowWidth,
        "maxNumberOfTicks must exceed windowWidth so a window can be fully fed before timing out");

    std::vector<unsigned char> poolVec;

    // The random2 pool the scorer reads. initialize()/initializeFromMemory() point it at the owned
    // poolVec; setPool() points it at a caller-owned pool shared read-only across threads (the
    // ant-colony path builds the ~512 MB pool once and shares it).
    const unsigned char* pRandom2Pool = nullptr;

    void setPool(const unsigned char* pool)
    {
        pRandom2Pool = pool;
    }

    bool initialize(unsigned char* miningSeed, const char* taskFilePath)
    {
        poolVec.resize(POOL_VEC_PADDING_SIZE);
        generateRandom2Pool(miningSeed, poolVec.data());
        pRandom2Pool = poolVec.data();

        return loadTaskData(taskFilePath);
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
            header.numPairs != sequenceLength ||
            header.population != populationThreshold ||
            header.numNeighbors != numberOfNeighbors)
        {
            return false;
        }

        unsigned char hash[task_file::DATA_HASH_SIZE];

        if (!task_file::readTaskFileBlock(taskFilePath, sizeof(task_file::TaskFileHeader), topoBlockBuf, topoBlockSize))
        {
            return false;
        }
        KangarooTwelve(topoBlockBuf, (unsigned int)topoBlockSize, hash, task_file::DATA_HASH_SIZE);
        if (memcmp(hash, header.topologyHash, task_file::DATA_HASH_SIZE) != 0)
        {
            return false;
        }
        task_file::parseTopologyBlock(topoBlockBuf, numberOfInputNeurons, numberOfOutputNeurons, populationThreshold, numberOfNeighbors,
                                      inputNeuronIndices, outputNeuronIndices, &signalNeuronIndex, neighborIndices);
        if (!validateTopology())
        {
            return false;
        }

        if (!task_file::readTaskFileBlock(taskFilePath, sizeof(task_file::TaskFileHeader) + topoBlockSize, dataBlockBuf, dataBlockSize))
        {
            return false;
        }
        KangarooTwelve(dataBlockBuf, (unsigned int)dataBlockSize, hash, task_file::DATA_HASH_SIZE);
        if (memcmp(hash, header.dataHash, task_file::DATA_HASH_SIZE) != 0)
        {
            return false;
        }
        if (!task_file::unpackDataBlock(numberOfInputNeurons, numberOfOutputNeurons, sequenceLength, dataBlockBuf, &inputs[0][0], &outputs[0][0]))
        {
            return false;
        }

        deriveNeuronRoles();
        return true;
    }

    // In-memory task load, skip file I/O + hash and parse the two
    // already-in-memory blocks directly, via the same parse/validate/unpack path as loadTaskData.
    bool loadTaskFromMemory(const unsigned char* topoBlock, const unsigned char* dataBlock)
    {
        task_file::parseTopologyBlock(topoBlock, numberOfInputNeurons, numberOfOutputNeurons, populationThreshold, numberOfNeighbors,
                                      inputNeuronIndices, outputNeuronIndices, &signalNeuronIndex, neighborIndices);
        if (!validateTopology())
        {
            return false;
        }
        if (!task_file::unpackDataBlock(numberOfInputNeurons, numberOfOutputNeurons, sequenceLength, dataBlock, &inputs[0][0], &outputs[0][0]))
        {
            return false;
        }
        deriveNeuronRoles();
        return true;
    }

    bool initializeFromMemory(unsigned char* miningSeed, const unsigned char* topoBlock, const unsigned char* dataBlock)
    {
        poolVec.resize(POOL_VEC_PADDING_SIZE);
        generateRandom2Pool(miningSeed, poolVec.data());
        pRandom2Pool = poolVec.data();
        return loadTaskFromMemory(topoBlock, dataBlock);
    }

    bool validateTopology()
    {
        for (unsigned long long i = 0; i < numberOfInputNeurons; ++i)
        {
            if (inputNeuronIndices[i] >= populationThreshold)
            {
                return false;
            }
        }
        for (unsigned long long i = 0; i < numberOfOutputNeurons; ++i)
        {
            if (outputNeuronIndices[i] >= populationThreshold)
            {
                return false;
            }
        }
        if (signalNeuronIndex >= populationThreshold)
        {
            return false;
        }

        bool seen[populationThreshold] = {};
        for (unsigned long long i = 0; i < numberOfInputNeurons; ++i)
        {
            if (seen[inputNeuronIndices[i]])
            {
                return false;
            }
            seen[inputNeuronIndices[i]] = true;
        }
        for (unsigned long long i = 0; i < numberOfOutputNeurons; ++i)
        {
            if (seen[outputNeuronIndices[i]])
            {
                return false;
            }
            seen[outputNeuronIndices[i]] = true;
        }
        if (seen[signalNeuronIndex])
        {
            return false;
        }

        for (unsigned long long i = 0; i < populationThreshold * numberOfNeighbors; ++i)
        {
            if (neighborIndices[i] >= populationThreshold)
            {
                return false;
            }
        }
        return true;
    }

    void deriveNeuronRoles()
    {
        for (unsigned long long i = 0; i < populationThreshold; ++i)
        {
            neuronTypes[i] = Neuron::kEvolution;
        }
        for (unsigned long long i = 0; i < numberOfInputNeurons; ++i)
        {
            neuronTypes[inputNeuronIndices[i]] = Neuron::kInput;
        }
        for (unsigned long long i = 0; i < numberOfOutputNeurons; ++i)
        {
            neuronTypes[outputNeuronIndices[i]] = Neuron::kOutput;
        }

        numberOfUpdatedNeurons = 0;
        for (unsigned long long i = 0; i < populationThreshold; ++i)
        {
            if (neuronTypes[i] != Neuron::kInput)
            {
                updatedNeuronIndices[numberOfUpdatedNeurons] = i;
                numberOfUpdatedNeurons++;
            }
        }
    }

    unsigned char inputs[sequenceLength][numberOfInputNeurons];
    unsigned char outputs[sequenceLength][numberOfOutputNeurons];

    // Scratch for the two file blocks: read, hash, then parse/unpack.
    unsigned char topoBlockBuf[topoBlockSize];
    unsigned char dataBlockBuf[dataBlockSize];

    struct Neuron
    {
        enum Type
        {
            kInput,
            kOutput,
            kEvolution,
        };
        Type type;
        unsigned char value;
    };

    struct ANN
    {
        Neuron neurons[maxNumberOfNeurons];
        unsigned char lut[maxNumberOfNeurons * lutSize];
    };
    ANN bestANN;
    ANN currentANN;
    ANN prevANN;

    struct InitValue
    {
        unsigned char lutInit[maxNumberOfNeurons * lutSize];
        unsigned long long mutationSeed[numberOfMutations * MAX_LUT_ENTRIES_PER_STEP];
    } initValue;

    unsigned char nextNeuronValue[maxNumberOfNeurons];

    uint32_t neighborIndices[populationThreshold * numberOfNeighbors];

    uint32_t inputNeuronIndices[numberOfInputNeurons];
    uint32_t outputNeuronIndices[numberOfOutputNeurons];

    uint32_t signalNeuronIndex;

    typename Neuron::Type neuronTypes[maxNumberOfNeurons];

    unsigned long long updatedNeuronIndices[maxNumberOfNeurons];
    unsigned long long numberOfUpdatedNeurons;

    // One inference tick: each non-input neuron looks up its next trit from its 3 neighbours.
    void processTick()
    {
        const unsigned long long population = populationThreshold;
        Neuron* neurons = currentANN.neurons;

        for (unsigned long long n = 0; n < population; ++n)
        {
            if (Neuron::kInput == neurons[n].type)
            {
                nextNeuronValue[n] = neurons[n].value;
                continue;
            }

            // Explicit per-neuron neighbours; base-3 LUT index = t0 + 3*t1 + 9*t2.
            const unsigned long long t0 = neurons[neighborIndices[n * numberOfNeighbors + 0]].value;
            const unsigned long long t1 = neurons[neighborIndices[n * numberOfNeighbors + 1]].value;
            const unsigned long long t2 = neurons[neighborIndices[n * numberOfNeighbors + 2]].value;
            nextNeuronValue[n] = currentANN.lut[n * lutSize + (t0 + 3 * t1 + 9 * t2)];
        }

        for (unsigned long long n = 0; n < population; ++n)
        {
            if (Neuron::kInput != neurons[n].type)
            {
                neurons[n].value = nextNeuronValue[n];
            }
        }
    }

    // Sliding-window self-clocked score: feed W samples (signal-paced), settle until the signal is
    // UNKNOWN again, then grade the output vs outputs[t+W]. A timed-out window fails the whole ANN.
    // Returns the failure count (lower is better), or INFINITE_ERROR on timeout.
    unsigned int score()
    {
        unsigned int numberOfFailures = 0;

        Neuron* neurons = currentANN.neurons;

        for (unsigned long long trainingEntryIndex = 0; trainingEntryIndex < numberOfWindows; ++trainingEntryIndex)
        {
            unsigned long long feedCounter = 0;

            for (unsigned long long n = 0; n < populationThreshold; ++n)
            {
                neurons[n].value = TRIT_UNKNOWN;
            }

            unsigned long long tick;
            for (tick = 0; tick < maxNumberOfTicks; tick++)
            {
                if (neurons[signalNeuronIndex].value == TRIT_UNKNOWN)
                {
                    // Whole window is in and the signal is ready -> output is settled, read it.
                    if (feedCounter >= windowWidth)
                    {
                        break;
                    }
                    for (unsigned long long i = 0; i < numberOfInputNeurons; ++i)
                    {
                        neurons[inputNeuronIndices[i]].value = inputs[trainingEntryIndex + feedCounter][i];
                    }
                    feedCounter++;
                }
                else
                {
                    for (unsigned long long i = 0; i < numberOfInputNeurons; ++i)
                    {
                        neurons[inputNeuronIndices[i]].value = TRIT_UNKNOWN;
                    }
                }

                processTick();
            }

            if (tick == maxNumberOfTicks)
            {
                return INFINITE_ERROR;
            }

            const unsigned char predicted = neurons[outputNeuronIndices[0]].value;
            const unsigned char expected = outputs[trainingEntryIndex + feedCounter][0];
            if (predicted != expected)
            {
                numberOfFailures++;
            }
        }

        return numberOfFailures;
    }

    // Flip one LUT entry of one non-input neuron (bit 0 picks the new trit, the rest picks the line).
    void mutate(unsigned long long mutationSeed)
    {
        const unsigned long long delta = mutationSeed & 1ULL;

        const unsigned long long totalLines = numberOfUpdatedNeurons * lutSize;
        const unsigned long long flatIdx = (mutationSeed >> 1) % totalLines;
        const unsigned long long neuronIdx = updatedNeuronIndices[flatIdx / lutSize];
        const unsigned long long line = flatIdx % lutSize;

        const unsigned char oldTrit = currentANN.lut[neuronIdx * lutSize + line];
        const unsigned char newTrit = (unsigned char)((oldTrit + 1 + delta) % 3);
        currentANN.lut[neuronIdx * lutSize + line] = newTrit;
    }

    // Standalone path only: root LUT from the pubkey alone; the ant path derives the shared epoch
    // root via deriveRootANN(rootSeed) instead. Mutation seeds from pubkey+nonce (nonce[0..2] are
    // the algo/L/K knobs, excluded from the RNG). Returns the start score.
    unsigned int initializeANN(unsigned char* publicKey, unsigned char* nonce)
    {
        const unsigned long long population = populationThreshold;
        Neuron* neurons = currentANN.neurons;

        unsigned char rootHash[32];
        KangarooTwelve(publicKey, 32, rootHash, 32);
        random2(rootHash, pRandom2Pool, (unsigned char*)&initValue.lutInit, sizeof(initValue.lutInit));

        unsigned char searchHash[32];
        unsigned char combined[64];
        memcpy(combined, publicKey, 32);
        memcpy(combined + 32, nonce, 32);
        combined[32] = 0;
        combined[33] = 0;
        combined[34] = 0;
        KangarooTwelve(combined, 64, searchHash, 32);
        random2(searchHash, pRandom2Pool, (unsigned char*)&initValue.mutationSeed, sizeof(initValue.mutationSeed));

        for (unsigned long long i = 0; i < population; ++i)
        {
            neurons[i].type = neuronTypes[i];
            neurons[i].value = TRIT_UNKNOWN;
        }

        for (unsigned long long n = 0; n < population; ++n)
        {
            for (unsigned long long line = 0; line < lutSize; ++line)
            {
                currentANN.lut[n * lutSize + line] = (unsigned char)(initValue.lutInit[n * lutSize + line] % 3);
            }
        }

        return score();
    }

    // Standalone solo score: root network from the pubkey, explore disabled (K = 0). L = nonce[1]
    // clamped to [1, MAX_LUT_ENTRIES_PER_STEP]; nonce[0] and nonce[2] do not affect this path.
    unsigned int computeScore(unsigned char* publicKey, unsigned char* nonce)
    {
        const unsigned int L = lutEntriesPerStep(nonce);
        const unsigned int cur = initializeANN(publicKey, nonce);
        return computeScoreFromCurrent(L, 0, cur);
    }

    bool findSolution(unsigned char* publicKey, unsigned char* nonce, unsigned int& outScore)
    {
        outScore = computeScore(publicKey, nonce);
        return outScore <= solutionThreshold;
    }

    // ---- Ant colony seam ---------------------------------------------------------------------------
    // Ported from core/src/mining/score_bpp9000.h. Scores here are bit-identical to the node's, which
    // is what makes ant-colony solutions acceptable. Shares pRandom2Pool, score(), mutate() and the
    // trit arithmetic with the solo path above.

    static unsigned int lutEntriesPerStep(const unsigned char* nonce)
    {
        unsigned int L = nonce[1];
        if (L < 1)
        {
            L = 1;
        }
        if (L > MAX_LUT_ENTRIES_PER_STEP)
        {
            L = MAX_LUT_ENTRIES_PER_STEP;
        }
        return L;
    }

    // Anti-attractor walk from the LUT already in currentANN: L mutations/step; accept worse-or-equal
    // for the first K steps (explore), then better-or-equal (exploit); one-step rollback. Returns the
    // best score found and leaves the LUT that produced it in bestANN.
    unsigned int computeScoreFromCurrent(unsigned int L, unsigned long long K, unsigned int startScore)
    {
        unsigned int cur = startScore;
        unsigned int best = cur;
        memcpy(&bestANN, &currentANN, sizeof(bestANN));

        for (unsigned long long s = 0; s < numberOfMutations; ++s)
        {
            memcpy(&prevANN, &currentANN, sizeof(prevANN));

            for (unsigned int i = 0; i < L; ++i)
            {
                mutate(initValue.mutationSeed[s * MAX_LUT_ENTRIES_PER_STEP + i]);
            }

            const unsigned int r = score();

            bool accept = false;
            if (s < K)
            {
                accept = (r >= cur);
            }
            else
            {
                accept = (r <= cur);
            }

            if (accept)
            {
                cur = r;
            }
            else
            {
                memcpy(&currentANN, &prevANN, sizeof(currentANN));
            }

            if (cur < best)
            {
                best = cur;
                memcpy(&bestANN, &currentANN, sizeof(bestANN));
            }
        }
        return best;
    }

    static bool isCanonicalAntNonce(const unsigned char* nonce)
    {
        return (nonce[0] == 1)                             // AlgoType::Bpp9000
            && (nonce[1] >= 1)
            && (nonce[1] <= MAX_LUT_ENTRIES_PER_STEP)
            && (nonce[2] <= numberOfMutations);
    }

    // Set every neuron's role/value and copy a LUT into currentANN, ready to score.
    void loadLutIntoCurrent(const unsigned char* lut)
    {
        for (unsigned long long i = 0; i < populationThreshold; ++i)
        {
            currentANN.neurons[i].type = neuronTypes[i];
            currentANN.neurons[i].value = TRIT_UNKNOWN;
        }
        memcpy(currentANN.lut, lut, maxNumberOfNeurons * lutSize);
    }

    // Mutation-walk seeds. When anchorTickDigest != nullptr the walk is bound to that anchor (the
    // ant-colony case); nonce[0..2] are the algo/L/K knobs and stay out of the RNG.
    void deriveAntMutationSeeds(const unsigned char* publicKey, const unsigned char* nonce,
        const unsigned char* anchorTickDigest)
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
        random2(searchHash, pRandom2Pool, (unsigned char*)&initValue.mutationSeed, sizeof(initValue.mutationSeed));
    }

    // The shared per-epoch root network: root LUT from the epoch-start spectrum digest, identical
    // for every identity, written into a caller-owned ANN.
    void deriveRootANN(const unsigned char* rootSeed, ANN& out)
    {
        unsigned char rootHash[32];
        KangarooTwelve(rootSeed, 32, rootHash, 32);
        random2(rootHash, pRandom2Pool, (unsigned char*)&initValue.lutInit, sizeof(initValue.lutInit));

        for (unsigned long long i = 0; i < populationThreshold; ++i)
        {
            out.neurons[i].type = neuronTypes[i];
            out.neurons[i].value = TRIT_UNKNOWN;
        }
        for (unsigned long long n = 0; n < populationThreshold; ++n)
        {
            for (unsigned long long line = 0; line < lutSize; ++line)
            {
                out.lut[n * lutSize + line] = (unsigned char)(initValue.lutInit[n * lutSize + line] % 3);
            }
        }
    }

    // Score a child by inheriting the parent's LUT and walking it with the child's own anchored seeds.
    // parentLut is the 1728-byte canonical LUT (from deriveRootANN().lut or a fetched parent node).
    unsigned int computeScoreFromParent(const unsigned char* parentLut, const unsigned char* publicKey,
        const unsigned char* nonce, const unsigned char* anchorTickDigest)
    {
        if (!isCanonicalAntNonce(nonce))
        {
            return INVALID_SCORE_VALUE;
        }
        loadLutIntoCurrent(parentLut);
        deriveAntMutationSeeds(publicKey, nonce, anchorTickDigest);

        const unsigned int L = lutEntriesPerStep(nonce);
        const unsigned long long K = nonce[2];
        const unsigned int cur = score();
        return computeScoreFromCurrent(L, K, cur);
    }
};

}
