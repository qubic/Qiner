#pragma once

#include "score_common.h"
#include "K12AndKeyUtil.h"

#include <cassert>
#include <vector>

// LUT-based Addition.
// Every neuron holds a trit and computes its next value by looking up a per-neuron table indexed by the trits of
// a set of neighbours on the ring, only the LUT contents change under mutation.
//
// Trit values are {0, 1, 2} where 0 and 1 are the two decided states and 2 means UNKNOWN.
namespace score_addition
{

static constexpr unsigned long long NUMBER_OF_INPUT_NEURONS = 2 * 7; // K
static constexpr unsigned long long NUMBER_OF_OUTPUT_NEURONS = 8;    // L
static constexpr unsigned long long NUMBER_OF_TICKS = 120;
static constexpr unsigned long long NUMBER_OF_MUTATIONS = 100;
static constexpr unsigned long long POPULATION_THRESHOLD = 32;       // P
// The neighbour offsets that feed every neuron's LUT, in LUT-index order
static constexpr long long NEIGHBOR_OFFSETS[] = { 1, 4, 13 };
static constexpr unsigned long long MAX_NEIGHBOR_NEURONS =
    sizeof(NEIGHBOR_OFFSETS) / sizeof(NEIGHBOR_OFFSETS[0]);
static constexpr unsigned int SOLUTION_THRESHOLD = ((1ULL << NUMBER_OF_INPUT_NEURONS) * NUMBER_OF_OUTPUT_NEURONS * 4 / 5);
// Max LUT entries one mutation step may use (miner-chosen L)
static constexpr unsigned int MAX_LUT_ENTRIES_PER_STEP = 10;

template <
    unsigned long long numberOfInputNeurons,  // K
    unsigned long long numberOfOutputNeurons, // L
    unsigned long long numberOfTicks,         // N
    unsigned long long maxNumberOfNeighbors,  // LUT fan-in (neighbours read per neuron)
    unsigned long long populationThreshold,   // P
    unsigned long long numberOfMutations,     // S
    unsigned int solutionThreshold>
struct Miner
{
    static constexpr unsigned long long numberOfNeurons =
        numberOfInputNeurons + numberOfOutputNeurons;
    static constexpr unsigned long long maxNumberOfNeurons = populationThreshold;
    static constexpr unsigned long long numberOfEvolutionNeurons =
        populationThreshold - numberOfNeurons;   // P - K - L
    static constexpr unsigned long long trainingSetSize = 1ULL << numberOfInputNeurons; // 2^K

    // Undecided trit (the third value); the two decided states are 0 and 1.
    static constexpr unsigned char TRIT_UNKNOWN = 2;

    // 3^maxNumberOfNeighbors lines per LUT (one output trit per neighbour-trit combination).
    static constexpr unsigned long long ipow(unsigned long long base, unsigned long long exp)
    {
        unsigned long long result = 1;
        for (unsigned long long i = 0; i < exp; ++i)
        {
            result *= base;
        }
        return result;
    }
    static constexpr unsigned long long lutSize = ipow(3, maxNumberOfNeighbors);

    static_assert(
        populationThreshold > numberOfNeurons,
        "populationThreshold must be greater than numberOfNeurons");
    static_assert(
        (populationThreshold & (populationThreshold - 1)) == 0,
        "populationThreshold must be a power of 2");
    static_assert(
        maxNumberOfNeighbors == sizeof(NEIGHBOR_OFFSETS) / sizeof(NEIGHBOR_OFFSETS[0]),
        "maxNumberOfNeighbors must equal the NEIGHBOR_OFFSETS table length");

    std::vector<unsigned char> poolVec;

    void initialize(unsigned char miningSeed[32])
    {
        // Init random2 pool with mining seed
        poolVec.resize(POOL_VEC_PADDING_SIZE);
        generateRandom2Pool(miningSeed, poolVec.data());
    }

    // Training set
    struct TraningPair
    {
        char input[numberOfInputNeurons]; // numberOfInputNeurons / 2 bits of A , and B (values: -1 or +1)
        char output[numberOfOutputNeurons];  // numberOfOutputNeurons bits of C (values: -1 or +1)
    } trainingSet[trainingSetSize];       // training set size: 2^K

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
        unsigned long long population;
    };
    ANN bestANN;
    ANN currentANN;
    // Snapshot for the one-step rollback in the anti-attractor walk.
    ANN prevANN;

    struct InitValue
    {
        unsigned long long outputNeuronPositions[numberOfOutputNeurons];
        unsigned long long evolutionNeuronPositions[numberOfEvolutionNeurons];
        unsigned char lutInit[maxNumberOfNeurons * lutSize]; // one byte per LUT line, taken mod 3
        unsigned long long mutationSeed[numberOfMutations * MAX_LUT_ENTRIES_PER_STEP];
    } initValue;

    unsigned long long neuronIndices[maxNumberOfNeurons];
    unsigned char previousNeuronValue[maxNumberOfNeurons];
    unsigned char nextNeuronValue[maxNumberOfNeurons];

    unsigned long long outputNeuronIndices[numberOfOutputNeurons];
    unsigned char outputNeuronExpectedValue[numberOfOutputNeurons];

    // Indices of all non-input neurons (output + evolution), the only ones whose LUT is used
    // and the only ones a mutation may touch. Filled in initializeANN().
    unsigned long long updatedNeuronIndices[maxNumberOfNeurons];
    unsigned long long numberOfUpdatedNeurons;

    // Map a bipolar training bit to a trit. The two decided states map to 0 and, the neutral  maps to UNKNOWN.
    static unsigned char bipolarToTrit(char val)
    {
        if (val < 0)
        {
            return 0;
        }
        if (val > 0)
        {
            return 1;
        }
        return TRIT_UNKNOWN;
    }

    // Calculate the new neuron index reached by moving `value` neurons along the ring (wraps).
    unsigned long long clampNeuronIndex(long long neuronIdx, long long value)
    {
        unsigned long long population = currentANN.population;
        assert(value > -(long long)population && value < (long long)population
           && "clampNeuronIndex: |value| must be less than population");

        long long nnIndex = 0;
        if (value >= 0)
        {
            nnIndex = neuronIdx + value;
        }
        else
        {
            nnIndex = neuronIdx + population + value;
        }
        nnIndex = nnIndex % population;
        return (unsigned long long)nnIndex;
    }

    // Get neighbor index
    unsigned long long getSourceNeuron(unsigned long long neuronIdx, unsigned long long sourceSlot)
    {
        return clampNeuronIndex((long long)neuronIdx, NEIGHBOR_OFFSETS[sourceSlot]);
    }

    // Inference step, every non-input neuron looks up its next trit from the trits of its neighbours
    void processTick()
    {
        unsigned long long population = currentANN.population;
        Neuron* neurons = currentANN.neurons;

        for (unsigned long long n = 0; n < population; ++n)
        {
            if (Neuron::kInput == neurons[n].type)
            {
                nextNeuronValue[n] = neurons[n].value; // inputs are held
                continue;
            }

            // Base-3 index over the neighbours: index = sum(neighbourTrit_k * 3^k).
            unsigned long long index = 0;
            unsigned long long place = 1;
            for (unsigned long long k = 0; k < maxNumberOfNeighbors; ++k)
            {
                unsigned long long nnIndex = getSourceNeuron(n, k);
                index += (unsigned long long)neurons[nnIndex].value * place;
                place *= 3;
            }
            nextNeuronValue[n] = currentANN.lut[n][index];
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

    void loadTrainingData(unsigned long long trainingIndex)
    {
        unsigned long long population = currentANN.population;
        Neuron* neurons = currentANN.neurons;

        const auto& data = trainingSet[trainingIndex];
        unsigned long long inputIndex = 0;
        for (unsigned long long n = 0; n < population; ++n)
        {
            if (Neuron::kInput == neurons[n].type)
            {
                neurons[n].value = bipolarToTrit(data.input[inputIndex]);
                inputIndex++;
            }
            else
            {
                neurons[n].value = TRIT_UNKNOWN; // undecided before the dynamics run
            }
        }

        for (unsigned long long i = 0; i < numberOfOutputNeurons; ++i)
        {
            outputNeuronExpectedValue[i] = bipolarToTrit(data.output[i]);
        }
    }

    // Tick simulation only runs on one ANN
    void runTickSimulation(unsigned long long trainingIndex)
    {
        unsigned long long population = currentANN.population;
        Neuron* neurons = currentANN.neurons;

        loadTrainingData(trainingIndex);

        for (unsigned long long i = 0; i < population; ++i)
        {
            previousNeuronValue[i] = neurons[i].value;
        }

        for (unsigned long long tick = 0; tick < numberOfTicks; ++tick)
        {
            processTick();
            // Exit conditions:
            // - N ticks have passed (already in for loop)
            // - All neuron values are unchanged
            // - All output neurons are decided (left the UNKNOWN trit)
            bool allNeuronsUnchanged = true;
            bool allOutputsDecided = true;
            for (unsigned long long n = 0; n < population; ++n)
            {
                if (previousNeuronValue[n] != neurons[n].value)
                {
                    allNeuronsUnchanged = false;
                }
                if (neurons[n].type == Neuron::kOutput && neurons[n].value == TRIT_UNKNOWN)
                {
                    allOutputsDecided = false;
                }
            }

            if (allOutputsDecided || allNeuronsUnchanged)
            {
                break;
            }

            for (unsigned long long n = 0; n < population; ++n)
            {
                previousNeuronValue[n] = neurons[n].value;
            }
        }
    }

    unsigned int computeMatchingOutput()
    {
        unsigned long long population = currentANN.population;
        Neuron* neurons = currentANN.neurons;

        // Output neurons are matched in index-scan order against the expected trits.
        unsigned int R = 0;
        unsigned long long outputIdx = 0;
        for (unsigned long long i = 0; i < population; i++)
        {
            if (neurons[i].type == Neuron::kOutput)
            {
                if (neurons[i].value == outputNeuronExpectedValue[outputIdx])
                {
                    R++;
                }
                outputIdx++;
            }
        }
        return R;
    }

    // Generate all 2^K possible (A, B, C) pairs
    void generateTrainingSet()
    {
        static constexpr long long boundValue = (1LL << (numberOfInputNeurons / 2)) / 2;
        unsigned long long index = 0;
        for (long long A = -boundValue; A < boundValue; A++)
        {
            for (long long B = -boundValue; B < boundValue; B++)
            {
                long long C = A + B;

                toTenaryBits<numberOfInputNeurons / 2>(A, trainingSet[index].input);
                toTenaryBits<numberOfInputNeurons / 2>(
                    B, trainingSet[index].input + numberOfInputNeurons / 2);
                toTenaryBits<numberOfOutputNeurons>(C, trainingSet[index].output);
                index++;
            }
        }
    }

    unsigned int inferANN()
    {
        unsigned int score = 0;
        for (unsigned long long i = 0; i < trainingSetSize; ++i)
        {
            // Ticks simulation
            runTickSimulation(i);

            // Compute R
            unsigned int R = computeMatchingOutput();
            score += R;
        }
        return score;
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

        unsigned long long& population = currentANN.population;
        Neuron* neurons = currentANN.neurons;

        // Initialization fixed-topology: population is N total, set once.
        population = populationThreshold;

        // Generate all 2^K possible (A, B, C) pairs
        generateTrainingSet();

        // Initalize with nonce and public key
        random2(hash, poolVec.data(), (unsigned char*)&initValue, sizeof(InitValue));

        // Randomly choose the positions of neurons types. Default = Input.
        for (unsigned long long i = 0; i < population; ++i)
        {
            neuronIndices[i] = i;
            neurons[i].type = Neuron::kInput;
            neurons[i].value = TRIT_UNKNOWN;
        }
        unsigned long long neuronCount = population;

        // Output positions from the remaining pool
        for (unsigned long long i = 0; i < numberOfOutputNeurons; ++i)
        {
            unsigned long long outputNeuronIdx = initValue.outputNeuronPositions[i] % neuronCount;

            neurons[neuronIndices[outputNeuronIdx]].type = Neuron::kOutput;
            outputNeuronIndices[i] = neuronIndices[outputNeuronIdx];

            neuronCount = neuronCount - 1;
            neuronIndices[outputNeuronIdx] = neuronIndices[neuronCount];
        }

        // Evolution positions from the remaining pool
        for (unsigned long long i = 0; i < numberOfEvolutionNeurons; ++i)
        {
            unsigned long long evolutionNeuronIdx = initValue.evolutionNeuronPositions[i] % neuronCount;

            neurons[neuronIndices[evolutionNeuronIdx]].type = Neuron::kEvolution;

            neuronCount = neuronCount - 1;
            neuronIndices[evolutionNeuronIdx] = neuronIndices[neuronCount];
        }

        // Cache the indices of all updated (non-input) neurons for mutation.
        numberOfUpdatedNeurons = 0;
        for (unsigned long long i = 0; i < population; ++i)
        {
            if (neurons[i].type != Neuron::kInput)
            {
                updatedNeuronIndices[numberOfUpdatedNeurons] = i;
                numberOfUpdatedNeurons++;
            }
        }

        // Seed every LUT line with a trit.
        for (unsigned long long n = 0; n < population; ++n)
        {
            for (unsigned long long line = 0; line < lutSize; ++line)
            {
                currentANN.lut[n][line] = (unsigned char)(initValue.lutInit[n * lutSize + line] % 3);
            }
        }

        // Run the first inference to get starting point before mutation
        unsigned int score = inferANN();

        return score;
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

        unsigned int curR = initializeANN(publicKey, nonce);
        memcpy(&bestANN, &currentANN, sizeof(bestANN));
        unsigned int bestR = curR;

        for (unsigned long long s = 0; s < numberOfMutations; ++s)
        {
            // Snapshot for the one-step rollback.
            memcpy(&prevANN, &currentANN, sizeof(prevANN));

            // Apply L LUT-entry mutations from this step's fixed seed slot.
            for (unsigned int i = 0; i < L; ++i)
            {
                mutate(initValue.mutationSeed[s * MAX_LUT_ENTRIES_PER_STEP + i]);
            }

            const unsigned int r = inferANN();

            bool accept = false;
            if (s < K)
            {
                // First K steps, keep the mutation if it made the score worse.
                accept = (r <= curR);
            }
            else
            {
                // Then, keep the mutation if it made the score better.
                accept = (r >= curR);
            }

            if (accept)
            {
                curR = r;
            }
            else
            {
                // Roll back one step (to the previous position, NOT to the best).
                memcpy(&currentANN, &prevANN, sizeof(currentANN));
            }

            if (curR > bestR)
            {
                bestR = curR;
                memcpy(&bestANN, &currentANN, sizeof(bestANN));
            }
        }
        return bestR;
    }

    bool findSolution(unsigned char* publicKey, unsigned char* nonce)
    {
        unsigned int score = computeScore(publicKey, nonce);
        if (score >= solutionThreshold)
        {
            return true;
        }

        return false;
    }
};

} // namespace score_addition
