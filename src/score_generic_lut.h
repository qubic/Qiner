#pragma once

#include "score_common.h"
#include "K12AndKeyUtil.h"
#include "task_file.h"

#include <cassert>
#include <vector>

// Generic LUT score engine.
// Every neuron holds a trit and computes its next value by looking up a per-neuron table indexed
// by the trits of its neighbour neurons; only the LUT contents change under mutation.
//
// Trit values are {0, 1, 2} where 0 and 1 are the two decided states and 2 means UNKNOWN.
//
// Scoring streams a sliding window of a time-ordered sequence into the input neurons at the
// network's own pace (gated by a signal neuron), then settles until the signal goes UNKNOWN
// (ready) before reading one output neuron. A window that cannot finish inside the tick budget
// times out and fails the whole ANN.
namespace score_generic_lut
{

static constexpr unsigned long long NUMBER_OF_INPUT_NEURONS = 14;
static constexpr unsigned long long NUMBER_OF_OUTPUT_NEURONS = 1;
static constexpr unsigned long long POPULATION_THRESHOLD = 256;
static constexpr unsigned long long NUMBER_OF_NEIGHBORS = 3;
static constexpr unsigned long long NUMBER_OF_MUTATIONS = 100;
static constexpr unsigned long long MAX_NUMBER_OF_TICKS = 256;

// Max LUT entries one mutation step may use (miner-chosen L from nonce[])
static constexpr unsigned int MAX_LUT_ENTRIES_PER_STEP = 10;

// Task relate params. 
// TODO: match this when the task file is released
static constexpr unsigned long long SEQUENCE_LENGTH = 128;              // T, total samples
static constexpr unsigned long long WINDOW_WIDTH = SEQUENCE_LENGTH / 2; // W

// Placeholder acceptance threshold; the real threshold is epoch config.
// TODO: remove this, we use best score
static constexpr unsigned int SOLUTION_THRESHOLD = (unsigned int)((WINDOW_WIDTH - 1) * 4 / 5);

template <
    unsigned long long numberOfInputNeurons,
    unsigned long long numberOfOutputNeurons,
    unsigned long long sequenceLength,
    unsigned long long windowWidth,
    unsigned long long maxNumberOfTicks,
    unsigned long long numberOfNeighbors,  // LUT fan-in (neighbours read per neuron)
    unsigned long long populationThreshold,   // P
    unsigned long long numberOfMutations,     // S
    unsigned int solutionThreshold>
struct Miner
{
    static constexpr unsigned long long maxNumberOfNeurons = populationThreshold;
    static constexpr unsigned long long feedCap = windowWidth;
    static constexpr unsigned long long numberOfWindows = sequenceLength - windowWidth;

    // Undecided trit (the third value); the two decided states are 0 and 1.
    static constexpr unsigned char TRIT_UNKNOWN = 2;

    // Any timed-out window fails the whole ANN; this stands in for its infinite score.
    static constexpr unsigned int INFINITE_ERROR = 0xFFFFFFFFU;

    // 3 trit inputs, 3^3 = 27 lines per LUT (one output trit per neighbour-trit combination).
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

    std::vector<unsigned char> poolVec;

    // Load the task data from taskFilePath, then fix the neuron placement and neighbour wiring from
    // the epoch-start spectrum digest. Returns false if the task file cannot be loaded or validated.
    bool initialize(unsigned char* miningSeed, const unsigned char* epochStartSpectrumDigest, const char* taskFilePath)
    {
        // Init random2 pool with mining seed
        poolVec.resize(POOL_VEC_PADDING_SIZE);
        generateRandom2Pool(miningSeed, poolVec.data());

        if (!loadTaskData(taskFilePath))
        {
            return false;
        }
        setEpochStartSpectrumDigest(epochStartSpectrumDigest);
        return true;
    }

    // Fix the neuron placement and neighbour wiring deterministically from the epoch-start digest.
    void setEpochStartSpectrumDigest(const unsigned char* epochStartSpectrumDigest)
    {
        random(epochStartSpectrumDigest, 32, (unsigned char*)&epochRandoms, sizeof(epochRandoms));

        computeNeuronPlacement();
        computeSourceNeurons();
    }

    // Read the task sequence from a task file into inputs/outputs. Validates the header against this
    // engine's compile-time dimensions and the stored data hash. Returns false on any mismatch.
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
            header.population != populationThreshold)
        {
            return false;
        }
        if (!task_file::readTaskFileData(taskFilePath, header, &inputs[0][0], &outputs[0][0]))
        {
            return false;
        }

        // Byte-level integrity: hash [all input trits || all output trits] and compare to the header.
        const unsigned long long inputTritCount = sequenceLength * numberOfInputNeurons;
        const unsigned long long outputTritCount = sequenceLength * numberOfOutputNeurons;
        std::vector<unsigned char> hashBuf;
        hashBuf.reserve((size_t)(inputTritCount + outputTritCount));
        hashBuf.insert(hashBuf.end(), &inputs[0][0], &inputs[0][0] + inputTritCount);
        hashBuf.insert(hashBuf.end(), &outputs[0][0], &outputs[0][0] + outputTritCount);
        unsigned char hash[task_file::DATA_HASH_SIZE];
        KangarooTwelve(hashBuf.data(), (unsigned int)hashBuf.size(), hash, task_file::DATA_HASH_SIZE);
        if (memcmp(hash, header.dataHash, task_file::DATA_HASH_SIZE) != 0)
        {
            return false;
        }
        return true;
    }

    // SequenceLength samples, each numberOfInputNeurons input trits and
    // numberOfOutputNeurons expected-output trits. 
    // Data is {0, 1}; trit 2 stays the UNKNOWN marker.
    unsigned char inputs[sequenceLength][numberOfInputNeurons];
    unsigned char outputs[sequenceLength][numberOfOutputNeurons];

    // Per-epoch placement draws from the spectrum digest.
    struct EpochRandoms
    {
        unsigned long long inputNeuronPositions[numberOfInputNeurons];
        unsigned long long outputNeuronPositions[numberOfOutputNeurons];
        unsigned long long signalNeuronPosition;
        unsigned long long neighborDraws[populationThreshold][numberOfNeighbors];
    } epochRandoms;

    // Data for running the ANN
    struct Neuron
    {
        enum Type
        {
            kInput,
            kOutput,
            kEvolution,
        };
        Type type;
        unsigned char value; // trit in {0, 1, 2}
    };

    // Data for roll back, mutation will change LUT output contents
    struct ANN
    {
        Neuron neurons[maxNumberOfNeurons];
        unsigned char lut[maxNumberOfNeurons][lutSize];
    };
    ANN bestANN;
    ANN currentANN;
    // Snapshot for the one-step rollback in the anti-attractor walk.
    ANN prevANN;

    struct InitValue
    {
        unsigned char lutInit[maxNumberOfNeurons * lutSize]; // one byte per LUT line, taken mod 3
        unsigned long long mutationSeed[numberOfMutations * MAX_LUT_ENTRIES_PER_STEP];
    } initValue;


    unsigned long long neuronIndices[maxNumberOfNeurons];
    unsigned char nextNeuronValue[maxNumberOfNeurons];

    // Fixed neighbour, source neuron index for each (neuron, slot)
    unsigned long long sourceNeuron[maxNumberOfNeurons][numberOfNeighbors];

    unsigned long long inputNeuronIndices[numberOfInputNeurons];
    unsigned long long outputNeuronIndices[numberOfOutputNeurons];

    // One evolution neuron drives the feed handshake; it is computed and mutated like any other
    // evolution neuron, its value is only additionally read for flow control.
    unsigned long long signalNeuronIndex;

    // Epoch-fixed neuron placement (input/output/evolution), computed once from the spectrum digest.
    Neuron::Type neuronTypes[maxNumberOfNeurons];

    // Indices of all non-input neurons (output + evolution), the only ones whose LUT is used
    // and the only ones a mutation may touch. Filled in computeNeuronPlacement().
    unsigned long long updatedNeuronIndices[maxNumberOfNeurons];
    unsigned long long numberOfUpdatedNeurons;

    // Each neuron draws numberOfNeighbors source neurons uniformly from all neurons, using the
    // epoch spectrum digest - so the wiring is global and identical on every node.
    // populationThreshold is a power of two, so the modulo is an unbiased mask.
    // Requires epochRandoms to be filled first (see setEpochStartSpectrumDigest).
    void computeSourceNeurons()
    {
        for (unsigned long long n = 0; n < populationThreshold; ++n)
        {
            for (unsigned long long k = 0; k < numberOfNeighbors; ++k)
            {
                sourceNeuron[n][k] = epochRandoms.neighborDraws[n][k] % populationThreshold;
            }
        }
    }

    // Neuron placement (input/output/evolution types plus the signal neuron) from the digest,
    // computed once per epoch.
    void computeNeuronPlacement()
    {
        for (unsigned long long i = 0; i < populationThreshold; ++i)
        {
            neuronIndices[i] = i;
            neuronTypes[i] = Neuron::kEvolution;
        }
        unsigned long long neuronCount = populationThreshold;

        // Input positions from the remaining pool
        for (unsigned long long i = 0; i < numberOfInputNeurons; ++i)
        {
            unsigned long long inputNeuronIdx = epochRandoms.inputNeuronPositions[i] % neuronCount;
            inputNeuronIndices[i] = neuronIndices[inputNeuronIdx];
            neuronTypes[neuronIndices[inputNeuronIdx]] = Neuron::kInput;
            neuronCount = neuronCount - 1;
            neuronIndices[inputNeuronIdx] = neuronIndices[neuronCount];
        }

        // Output positions from the remaining pool
        for (unsigned long long i = 0; i < numberOfOutputNeurons; ++i)
        {
            unsigned long long outputNeuronIdx = epochRandoms.outputNeuronPositions[i] % neuronCount;
            neuronTypes[neuronIndices[outputNeuronIdx]] = Neuron::kOutput;
            outputNeuronIndices[i] = neuronIndices[outputNeuronIdx];
            neuronCount = neuronCount - 1;
            neuronIndices[outputNeuronIdx] = neuronIndices[neuronCount];
        }

        // Signal neuron from the remaining pool. It stays kEvolution (computed and mutated like the
        // rest), only its index is remembered so the feed handshake can read it.
        unsigned long long signalIdx = epochRandoms.signalNeuronPosition % neuronCount;
        signalNeuronIndex = neuronIndices[signalIdx];
        neuronCount = neuronCount - 1;
        neuronIndices[signalIdx] = neuronIndices[neuronCount];

        // The remaining neurons stay kEvolution.

        // Cache the indices of all updated (non-input) neurons for mutation.
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

    // Inference step, every non-input neuron looks up its next trit from the trits of its neighbours
    void processTick()
    {
        const unsigned long long population = populationThreshold;
        Neuron* neurons = currentANN.neurons;

        for (unsigned long long n = 0; n < population; ++n)
        {
            if (Neuron::kInput == neurons[n].type)
            {
                nextNeuronValue[n] = neurons[n].value; // inputs are driven externally, not here
                continue;
            }

            // Base-3 index over the three neighbour trits, index = t0 + 3*t1 + 9*t2.
            const unsigned long long t0 = neurons[sourceNeuron[n][0]].value;
            const unsigned long long t1 = neurons[sourceNeuron[n][1]].value;
            const unsigned long long t2 = neurons[sourceNeuron[n][2]].value;
            nextNeuronValue[n] = currentANN.lut[n][t0 + 3 * t1 + 9 * t2];
        }

        // Commit the new values
        for (unsigned long long n = 0; n < population; ++n)
        {
            if (Neuron::kInput != neurons[n].type)
            {
                neurons[n].value = nextNeuronValue[n];
            }
        }
    }

    // Windowed self-clocked score matching the reference score(). Returns the total error count,
    // or INFINITE_ERROR if any window times out (an ANN has failed).
    unsigned int score()
    {
        unsigned int numberOfFalses = 0;
        unsigned int numberOfUnknowns = 0;

        Neuron* neurons = currentANN.neurons;

        for (unsigned long long t = 0; t < numberOfWindows; ++t)
        {
            unsigned long long feedCounter = 0;

            // Blank slate, then load the first sample of the window.
            for (unsigned long long n = 0; n < populationThreshold; ++n)
            {
                neurons[n].value = TRIT_UNKNOWN;
            }
            for (unsigned long long i = 0; i < numberOfInputNeurons; ++i)
            {
                neurons[inputNeuronIndices[i]].value = inputs[t + feedCounter][i];
            }
            feedCounter++;

            unsigned long long tick = 0;
            bool isFinalized = false;
            while ((feedCounter < feedCap || !isFinalized) && tick < maxNumberOfTicks)
            {
                processTick();
                tick++;

                // Drive the input neurons: when the signal is ready feed the next sample (or finalize
                // once the whole window is in), otherwise feed UNKNOWN and keep computing.
                const bool ready = (neurons[signalNeuronIndex].value == TRIT_UNKNOWN);
                for (unsigned long long i = 0; i < numberOfInputNeurons; ++i)
                {
                    neurons[inputNeuronIndices[i]].value = ready ? inputs[t + feedCounter][i] : TRIT_UNKNOWN;
                }
                if (ready)
                {
                    if (feedCounter < feedCap)
                    {
                        feedCounter++;
                    }
                    else
                    {
                        isFinalized = true;
                    }
                }
            }
            // A single timed-out window fails the whole ANN; the remaining windows cannot change
            // that, and grading here would tally a wrong-index result (feedCounter never reached
            // feedCap), so abandon this candidate immediately.
            if (tick == maxNumberOfTicks)
            {
                return INFINITE_ERROR;
            }

            const unsigned char predicted = neurons[outputNeuronIndices[0]].value;
            const unsigned char expected = outputs[t + feedCounter][0];
            if (predicted != expected)
            {
                if (predicted == TRIT_UNKNOWN)
                {
                    numberOfUnknowns++;
                }
                else
                {
                    numberOfFalses++;
                }
            }
        }

        return numberOfFalses + numberOfUnknowns;
    }

    // Rewrite a single LUT line of a single updated (non-input) neuron to a different trit.
    //  bit 0 selects the change, and the high bits select LUT-line to change
    void mutate(unsigned long long mutationSeed)
    {
        // bit 0: which of the two other trits to move to (always a change)
        const unsigned long long delta = mutationSeed & 1ULL;

        // bits 1..63: which LUT line
        const unsigned long long totalLines = numberOfUpdatedNeurons * lutSize;
        const unsigned long long flatIdx = (mutationSeed >> 1) % totalLines;
        const unsigned long long neuronIdx = updatedNeuronIndices[flatIdx / lutSize];
        const unsigned long long line = flatIdx % lutSize;

        const unsigned char oldTrit = currentANN.lut[neuronIdx][line];
        const unsigned char newTrit = (unsigned char)((oldTrit + 1 + delta) % 3);
        currentANN.lut[neuronIdx][line] = newTrit;
    }

    unsigned int initializeANN(unsigned char* publicKey, unsigned char* nonce)
    {
        unsigned char hash[32];
        unsigned char combined[64];
        memcpy(combined, publicKey, 32);
        memcpy(combined + 32, nonce, 32);
        // K, L and the algo bit live in nonce[0..2], exclude them from the RNG
        combined[32] = 0;
        combined[33] = 0;
        combined[34] = 0;
        KangarooTwelve(combined, 64, hash, 32);

        const unsigned long long population = populationThreshold;
        Neuron* neurons = currentANN.neurons;

        // LUT init and the mutation come from the nonce
        random2(hash, poolVec.data(), (unsigned char*)&initValue, sizeof(InitValue));

        // Apply the epoch-fixed neuron placement
        for (unsigned long long i = 0; i < population; ++i)
        {
            neurons[i].type = neuronTypes[i];
            neurons[i].value = TRIT_UNKNOWN;
        }

        // Seed every LUT line with a trit.
        for (unsigned long long n = 0; n < population; ++n)
        {
            for (unsigned long long line = 0; line < lutSize; ++line)
            {
                currentANN.lut[n][line] = (unsigned char)(initValue.lutInit[n * lutSize + line] % 3);
            }
        }

        // Error count of the starting ANN.
        return score();
    }

    // Main mining function: N mutation steps with the anti-attractor split
    unsigned int computeScore(unsigned char* publicKey, unsigned char* nonce)
    {
        // Miner knobs from nonce[1..2], do not affect the RNG.
        unsigned int L = nonce[1];
        if (L < 1)
        {
            L = 1;
        }
        if (L > MAX_LUT_ENTRIES_PER_STEP)
        {
            L = MAX_LUT_ENTRIES_PER_STEP;
        }
        unsigned long long K = nonce[2];
        if (K > numberOfMutations)
        {
            K = numberOfMutations;
        }

        unsigned int cur = initializeANN(publicKey, nonce);
        memcpy(&bestANN, &currentANN, sizeof(bestANN));
        unsigned int best = cur;

        for (unsigned long long s = 0; s < numberOfMutations; ++s)
        {
            // Snapshot for the one-step rollback.
            memcpy(&prevANN, &currentANN, sizeof(prevANN));

            // Apply L LUT-entry mutations from this step's fixed seed slot.
            for (unsigned int i = 0; i < L; ++i)
            {
                mutate(initValue.mutationSeed[s * MAX_LUT_ENTRIES_PER_STEP + i]);
            }

            const unsigned int r = score();

            bool accept = false;
            if (s < K)
            {
                // First K steps, keep the mutation if it made the score worse (or equal).
                accept = (r >= cur);
            }
            else
            {
                // Then, keep the mutation if it made the score better (or equal).
                accept = (r <= cur);
            }

            if (accept)
            {
                cur = r;
            }
            else
            {
                // Roll back one step (to the previous position, NOT to the best).
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

    bool findSolution(unsigned char* publicKey, unsigned char* nonce)
    {
        unsigned int totalErrors = computeScore(publicKey, nonce);
        if (totalErrors <= solutionThreshold)
        {
            return true;
        }

        return false;
    }
};

} // namespace score_generic_lut
