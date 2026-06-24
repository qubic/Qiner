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
static constexpr unsigned long long POPULATION_THRESHOLD = 256;      // P
// The neighbour offsets that feed every neuron's LUT, in LUT-index order
static constexpr long long NEIGHBOR_OFFSETS[] = { 1, 5, 47 };
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
    // Half of the 2^K possible pairs are graded each epoch (chosen from the epoch-start spectrum digest).
    static constexpr unsigned long long trainingSetSize = (1ULL << numberOfInputNeurons) / 2; // 2^(K-1)
    static constexpr unsigned long long fullTrainingSetSize = trainingSetSize * 2; // 2^K total possible pairs

    // Undecided trit (the third value); the two decided states are 0 and 1.
    static constexpr unsigned char TRIT_UNKNOWN = 2;

    // Error counts of one evaluation: FALSE = decided wrong, UNKNOWN = output left at trit 2.
    struct Score
    {
        unsigned int numberOfFalses;
        unsigned int numberOfUnknowns;
    };

    // Compare two error scores by total error count (fewer is better).
    // Returns 1 if (1) is worse, -1 if better, 0 if equal.
    static int compare(unsigned int numberOfFalses1, unsigned int numberOfUnknowns1, unsigned int numberOfFalses2, unsigned int numberOfUnknowns2)
    {
        const unsigned int total1 = numberOfFalses1 + numberOfUnknowns1;
        const unsigned int total2 = numberOfFalses2 + numberOfUnknowns2;
        if (total1 > total2)
        {
            return 1;
        }
        if (total1 < total2)
        {
            return -1;
        }
        return 0;
    }

    // 3 trit inputs, 3^3 = 27 lines per LUT (one output trit per neighbour-trit combination).
    static constexpr unsigned long long lutSize = 27;

    static_assert(
        maxNumberOfNeighbors == 3,
        "the LUT index is hardcoded for 3 neighbours");
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

    // The epoch-start Spectrum Digest is required; it selects the graded training subset.
    void initialize(unsigned char miningSeed[32], const unsigned char epochStartSpectrumDigest[32])
    {
        // Init random2 pool with mining seed
        poolVec.resize(POOL_VEC_PADDING_SIZE);
        generateRandom2Pool(miningSeed, poolVec.data());

        // Fixed neighbour wiring, computed once.
        computeSourceNeurons();

        // Generate the full sample, then select the subset from the Spectrum Digest.
        generateFullTrainingSet();
        setEpochStartSpectrumDigest(epochStartSpectrumDigest);
    }

    // Select the graded subset deterministically from the epoch-start Spectrum Digest
    void setEpochStartSpectrumDigest(const unsigned char epochStartSpectrumDigest[32])
    {
        // One random from the digest fills the subset selection draws and the neuron placement.
        random(epochStartSpectrumDigest, 32, (unsigned char*)&epochRandoms, sizeof(epochRandoms));

        // Select trainingSetSize distinct samples from the full set.
        for (unsigned long long i = 0; i < fullTrainingSetSize; ++i)
        {
            pairIndexPool[i] = (unsigned int)i;
        }
        for (unsigned long long k = 0; k < trainingSetSize; ++k)
        {
            const unsigned long long remaining = fullTrainingSetSize - k;
            const unsigned long long j = epochRandoms.selectionRandoms[k] % remaining;

            // Take pairIndexPool[j], remove it from the active range, copy that sample
            const unsigned int pickedTrainingIndex = pairIndexPool[j];
            // Swap the already pick to the tail to avoid duplicated selection
            pairIndexPool[j] = pairIndexPool[remaining - 1];

            trainingSet[k] = fullTrainingSet[pickedTrainingIndex];
        }

        // Neuron placement is fixed per epoch from the digest; compute it once here.
        computeNeuronPlacement();
    }

    struct TraningPair
    {
        char input[numberOfInputNeurons]; // numberOfInputNeurons / 2 bits of A , and B (values: -1 or +1)
        char output[numberOfOutputNeurons];  // numberOfOutputNeurons bits of C (values: -1 or +1)
    };
    // All 2^K possible samples (generated once); trainingSet is the 2^(K-1) graded subset chosen per epoch.
    TraningPair fullTrainingSet[fullTrainingSetSize];
    TraningPair trainingSet[trainingSetSize];

    // Per-epoch data from the spectrum digest
    struct EpochRandoms
    {
        unsigned int selectionRandoms[trainingSetSize];
        unsigned long long inputNeuronPositions[numberOfInputNeurons];
        unsigned long long outputNeuronPositions[numberOfOutputNeurons];
    } epochRandoms;
    unsigned int pairIndexPool[fullTrainingSetSize];

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
        unsigned char lutInit[maxNumberOfNeurons * lutSize]; // one byte per LUT line, taken mod 3
        unsigned long long mutationSeed[numberOfMutations * MAX_LUT_ENTRIES_PER_STEP];
    } initValue;


    unsigned long long neuronIndices[maxNumberOfNeurons];
    unsigned char nextNeuronValue[maxNumberOfNeurons];

    // Fixed neighbour, source neuron index for each (neuron, slot)
    unsigned long long sourceNeuron[maxNumberOfNeurons][maxNumberOfNeighbors];

    unsigned long long outputNeuronIndices[numberOfOutputNeurons];
    unsigned char outputNeuronExpectedValue[numberOfOutputNeurons];

    // Epoch-fixed neuron placement (input/output/evolution), computed once from the spectrum digest.
    Neuron::Type neuronTypes[maxNumberOfNeurons];

    // Indices of all non-input neurons (output + evolution), the only ones whose LUT is used
    // and the only ones a mutation may touch. Filled in computeNeuronPlacement().
    unsigned long long updatedNeuronIndices[maxNumberOfNeurons];
    unsigned long long numberOfUpdatedNeurons;

    // Map a bipolar training bit to a trit. The two decided states map to 0 and, the neutral maps to UNKNOWN.
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

    // Precompute the fixed neighbour wiring: source neuron index for each (neuron, slot)
    void computeSourceNeurons()
    {
        for (unsigned long long n = 0; n < populationThreshold; ++n)
        {
            for (unsigned long long k = 0; k < maxNumberOfNeighbors; ++k)
            {
                const long long value = NEIGHBOR_OFFSETS[k];
                long long nnIndex = 0;
                if (value >= 0)
                {
                    nnIndex = (long long)n + value;
                }
                else
                {
                    nnIndex = (long long)n + (long long)populationThreshold + value;
                }
                sourceNeuron[n][k] = (unsigned long long)(nnIndex % (long long)populationThreshold);
            }
        }
    }

    // Neuron placement (input/output/evolution types) from the digest, computed once per epoch.
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
        unsigned long long population = currentANN.population;
        Neuron* neurons = currentANN.neurons;

        for (unsigned long long n = 0; n < population; ++n)
        {
            if (Neuron::kInput == neurons[n].type)
            {
                nextNeuronValue[n] = neurons[n].value; // inputs are held
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

        for (unsigned long long tick = 0; tick < numberOfTicks; ++tick)
        {
            processTick();
            // Exit early once every output neuron is decided (left the UNKNOWN trit).
            bool allOutputsDecided = true;
            for (unsigned long long n = 0; n < population; ++n)
            {
                if (neurons[n].type == Neuron::kOutput && neurons[n].value == TRIT_UNKNOWN)
                {
                    allOutputsDecided = false;
                }
            }

            if (allOutputsDecided)
            {
                break;
            }
        }
    }

    // Count the output errors of the current ANN, FALSE (decided wrong) and UNKNOWN (undecided).
    void countOutputErrors(Score& score)
    {
        unsigned long long population = currentANN.population;
        Neuron* neurons = currentANN.neurons;

        unsigned long long outputIdx = 0;
        for (unsigned long long i = 0; i < population; i++)
        {
            if (neurons[i].type == Neuron::kOutput)
            {
                const unsigned char t = neurons[i].value;
                const unsigned char e = outputNeuronExpectedValue[outputIdx];
                if (t == TRIT_UNKNOWN)
                {
                    score.numberOfUnknowns++;
                }
                else if (t != e)
                {
                    score.numberOfFalses++;
                }
                outputIdx++;
            }
        }
    }

    // Generate all 2^K possible (A, B, C) pairs
    void generateFullTrainingSet()
    {
        static constexpr long long boundValue = (1LL << (numberOfInputNeurons / 2)) / 2;
        unsigned long long index = 0;
        for (long long A = -boundValue; A < boundValue; A++)
        {
            for (long long B = -boundValue; B < boundValue; B++)
            {
                long long C = A + B;

                toTenaryBits<numberOfInputNeurons / 2>(A, fullTrainingSet[index].input);
                toTenaryBits<numberOfInputNeurons / 2>(
                    B, fullTrainingSet[index].input + numberOfInputNeurons / 2);
                toTenaryBits<numberOfOutputNeurons>(C, fullTrainingSet[index].output);
                index++;
            }
        }
    }

    // Run the ANN over the selected training subset and return its error counts.
    Score inferANN()
    {
        Score score;
        score.numberOfFalses = 0;
        score.numberOfUnknowns = 0;
        for (unsigned long long i = 0; i < trainingSetSize; ++i)
        {
            runTickSimulation(i);
            countOutputErrors(score);
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

    Score initializeANN(unsigned char* publicKey, unsigned char* nonce)
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

        // Error counts of the starting ANN.
        return inferANN();
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

        Score cur = initializeANN(publicKey, nonce);
        memcpy(&bestANN, &currentANN, sizeof(bestANN));
        Score best = cur;

        for (unsigned long long s = 0; s < numberOfMutations; ++s)
        {
            // Snapshot for the one-step rollback.
            memcpy(&prevANN, &currentANN, sizeof(prevANN));

            // Apply L LUT-entry mutations from this step's fixed seed slot.
            for (unsigned int i = 0; i < L; ++i)
            {
                mutate(initValue.mutationSeed[s * MAX_LUT_ENTRIES_PER_STEP + i]);
            }

            const Score r = inferANN();
            const int c = compare(r.numberOfFalses, r.numberOfUnknowns, cur.numberOfFalses, cur.numberOfUnknowns);

            bool accept = false;
            if (s < K)
            {
                // First K steps, keep the mutation if it made the score worse.
                accept = (c >= 0);
            }
            else
            {
                // Then, keep the mutation if it made the score better.
                accept = (c <= 0);
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

            if (compare(cur.numberOfFalses, cur.numberOfUnknowns, best.numberOfFalses, best.numberOfUnknowns) < 0)
            {
                best = cur;
                memcpy(&bestANN, &currentANN, sizeof(bestANN));
            }
        }
        return best.numberOfFalses + best.numberOfUnknowns;
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

} // namespace score_addition
