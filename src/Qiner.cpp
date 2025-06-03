#include <chrono>
#include <thread>
#include <mutex>
#include <cstdio>
#include <cstring>
#include <array>
#include <queue>
#include <atomic>
#include <vector>
#include <assert.h>
#ifdef _MSC_VER
#include <intrin.h>
#include <winsock2.h>
#pragma comment (lib, "ws2_32.lib")

#else
#include <signal.h>
#include <immintrin.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#endif

#include "K12AndKeyUtil.h"
#include "keyUtils.h"


void random(unsigned char* publicKey, unsigned char* nonce, unsigned char* output, unsigned int outputSize)
{
    unsigned char state[200];
    memcpy(&state[0], publicKey, 32);
    memcpy(&state[32], nonce, 32);
    memset(&state[64], 0, sizeof(state) - 64);

    for (unsigned int i = 0; i < outputSize / sizeof(state); i++)
    {
        KeccakP1600_Permute_12rounds(state);
        memcpy(output, state, sizeof(state));
        output += sizeof(state);
    }
    if (outputSize % sizeof(state))
    {
        KeccakP1600_Permute_12rounds(state);
        memcpy(output, state, outputSize % sizeof(state));
    }
}

void random2(unsigned char* publicKey, unsigned char* nonce, unsigned char* output, unsigned int outputSize) // outputSize must be a multiple of 8
{
    unsigned char state[200];
    memcpy(&state[0], publicKey, 32);
    memcpy(&state[32], nonce, 32);
    memset(&state[64], 0, sizeof(state) - 64);

    // Data on heap to avoid stack overflow for some compiler
    std::vector<unsigned char> poolVec(1048576 + 24); // Need a multiple of 200
    unsigned char* pool = poolVec.data();

    for (unsigned int i = 0; i < poolVec.size(); i += sizeof(state))
    {
        KeccakP1600_Permute_12rounds(state);
        memcpy(&pool[i], state, sizeof(state));
    }

    unsigned int x = 0; // The same sequence is always used, exploit this for optimization
    for (unsigned long long i = 0; i < outputSize; i += 8)
    {
        *((unsigned long long*) & output[i]) = *((unsigned long long*) & pool[x & (1048576 - 1)]);
        x = x * 1664525 + 1013904223; // https://en.wikipedia.org/wiki/Linear_congruential_generator#Parameters_in_common_use
    }
}

void random2(unsigned char* miningSeed, unsigned char* output, unsigned int outputSize) // outputSize must be a multiple of 8
{
    unsigned char state[200];
    memcpy(&state[0], miningSeed, 32);
    memset(&state[32], 0, sizeof(state) - 32);

    // Data on heap to avoid stack overflow for some compiler
    std::vector<unsigned char> poolVec(1048576 + 24); // Need a multiple of 200
    unsigned char* pool = poolVec.data();

    for (unsigned int i = 0; i < poolVec.size(); i += sizeof(state))
    {
        KeccakP1600_Permute_12rounds(state);
        memcpy(&pool[i], state, sizeof(state));
    }

    unsigned int x = 0; // The same sequence is always used, exploit this for optimization
    for (unsigned long long i = 0; i < outputSize; i += 8)
    {
        *((unsigned long long*) & output[i]) = *((unsigned long long*) & pool[x & (1048576 - 1)]);
        x = x * 1664525 + 1013904223; // https://en.wikipedia.org/wiki/Linear_congruential_generator#Parameters_in_common_use
    }
}


struct RequestResponseHeader
{
private:
    unsigned char _size[3];
    unsigned char _type;
    unsigned int _dejavu;

public:
    inline unsigned int size()
    {
        return (*((unsigned int*)_size)) & 0xFFFFFF;
    }

    inline void setSize(unsigned int size)
    {
        _size[0] = (unsigned char)size;
        _size[1] = (unsigned char)(size >> 8);
        _size[2] = (unsigned char)(size >> 16);
    }

    inline bool isDejavuZero() const
    {
        return !_dejavu;
    }

    inline void zeroDejavu()
    {
        _dejavu = 0;
    }


    inline unsigned int dejavu() const
    {
        return _dejavu;
    }

    inline void setDejavu(unsigned int dejavu)
    {
        _dejavu = dejavu;
    }

    inline void randomizeDejavu()
    {
        _rdrand32_step(&_dejavu);
        if (!_dejavu)
        {
            _dejavu = 1;
        }
    }

    inline unsigned char type() const
    {
        return _type;
    }

    inline void setType(const unsigned char type)
    {
        _type = type;
    }
};

#define BROADCAST_MESSAGE 1

typedef struct
{
    unsigned char sourcePublicKey[32];
    unsigned char destinationPublicKey[32];
    unsigned char gammingNonce[32];
} Message;

char* nodeIp = NULL;
int nodePort = 0;
static constexpr unsigned long long NUMBER_OF_INPUT_NEURONS = 6000;     // K
static constexpr unsigned long long NUMBER_OF_OUTPUT_NEURONS = 256;    // L
static constexpr unsigned long long NUMBER_OF_TICKS = 1000;               // N
static constexpr unsigned long long MAX_NEIGHBOR_NEURONS = 1024; // 2M. Must divided by 2
static constexpr unsigned long long POPULATION_THRESHOLD = 8000; // P
static constexpr unsigned long long NUMBER_OF_MUTATIONS = 1024;
static constexpr unsigned int SOLUTION_THRESHOLD = NUMBER_OF_OUTPUT_NEURONS * 4 / 5;


// Clamp the neuron value
template <typename T>
T clampNeuron(T neuronValue)
{
    if (neuronValue > 1)
    {
        return 1;
    }

    if (neuronValue < -1)
    {
        return -1;
    }
    return neuronValue;
}

template <
unsigned long long numberOfInputNeurons, // K
unsigned long long numberOfOutputNeurons,// L
unsigned long long numberOfTicks,        // N
unsigned long long numberOfNeighbors,    // 2M
unsigned long long populationThreshold,  // P
unsigned long long numberOfMutations,    // S
unsigned int solutionThreshold
>
struct Miner
{
    static constexpr unsigned long long numberOfNeurons = numberOfInputNeurons + numberOfOutputNeurons;
    static constexpr unsigned long long maxNumberOfNeurons = populationThreshold;
    static constexpr unsigned long long maxNumberOfSynapses = populationThreshold * numberOfNeighbors;

    static_assert(maxNumberOfSynapses <= (0xFFFFFFFFFFFFFFFF << 1ULL), "maxNumberOfSynapses must less than or equal MAX_UINT64/2");
    static_assert(numberOfNeighbors % 2 == 0, "numberOfNeighbors must divided by 2");
    static_assert(populationThreshold > numberOfNeurons, "populationThreshold must be greater than numberOfNeurons");
    static_assert(numberOfNeurons > numberOfNeighbors, "Number of neurons must be greater than the number of neighbors");
    static_assert(numberOfNeurons % 8 == 0, "numberOfNeurons must divided by 8");

    unsigned char computorPublicKey[32];
    unsigned char currentRandomSeed[32];

    void initialize(unsigned char miningSeed[32])
    {
        memcpy(currentRandomSeed, miningSeed, sizeof(this->currentRandomSeed));
    }

    struct Synapse
    {
        char weight;
    };

    enum NeuronNeighborSide
    {
        kLeft,
        kRight,
        kMid,
        kOutOfBound
    };

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
        char value;
        bool markForRemoval;
    };

    // Data for roll back
    struct ANN
    {
        Neuron neurons[maxNumberOfNeurons];
        Synapse synapses[maxNumberOfSynapses];
        unsigned long long population;
    };
    ANN bestANN;
    ANN currentANN;

    // Intermediate data
    struct InitValue
    {
        unsigned long long outputNeuronPositions[numberOfOutputNeurons];
        unsigned long long synapseWeight[maxNumberOfSynapses];
        unsigned long long synpaseMutation[numberOfMutations];
    } initValue;

    struct MiningData
    {
        unsigned char inputNeuronRandomNumber[numberOfInputNeurons];
        unsigned char outputNeuronRandomNumber[numberOfOutputNeurons];
    } miningData;

    unsigned long long neuronIndices[numberOfNeurons];
    char previousNeuronValue[maxNumberOfNeurons];

    unsigned long long outputNeuronIndices[numberOfOutputNeurons];
    char outputNeuronExpectedValue[numberOfOutputNeurons];

    long long neuronValueBuffer[maxNumberOfNeurons];

    void mutate(unsigned char nonce[32], int mutateStep)
    {
        // Mutation
        unsigned long long population = currentANN.population;
        unsigned long long synapseCount = population * numberOfNeighbors;
        Synapse* synapses = currentANN.synapses;


        // Randomly pick a synapse, randomly increase or decrease its weight by 1 or -1
        unsigned long long synapseMutation = initValue.synpaseMutation[mutateStep];
        unsigned long long synapseIdx = (synapseMutation >> 1) % synapseCount;
        // Randomly increase or decrease its value
        char weightChange = 0;
        if ((synapseMutation & 1ULL) == 0)
        {
            weightChange = -1;
        }
        else
        {
            weightChange = 1;
        }

        char newWeight = synapses[synapseIdx].weight + weightChange;

        // Valid weight. Update it
        if (newWeight >= -1 && newWeight <=1 )
        {
            synapses[synapseIdx].weight = newWeight;
        }
        else // Invalid weight. Insert a neuron
        {
            // Insert the neuron
            insertNeuron(synapseIdx);
        }

        // Clean the ANN
        while (scanRedundantNeurons() > 0)
        {
            cleanANN();
        }
    }

    // Get the pointer to all outgoing synapse of a neurons
    Synapse* getSynapses(unsigned long long neuronIndex)
    {
        return &currentANN.synapses[neuronIndex * numberOfNeighbors];
    }

    // Circulate the neuron index
    unsigned long long clampNeuronIndex(long long neuronIdx, long long value)
    {
        unsigned long long population = currentANN.population;
        long long nnIndex = 0;
        // Calculate the neuron index (ring structure)
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


    // Remove a neuron and all synapses relate to it
    void removeNeuron(unsigned long long neuronIdx)
    {
        // Scan all its neigbor to remove their outgoing synapse point to the neuron
        for (long long i = 0; i < numberOfNeighbors; ++i)
        {
            long long delta = i - (long long)numberOfNeighbors / 2;
            unsigned long long neigborNeuronIdx = clampNeuronIndex(neuronIdx, delta);
            Synapse* pNNSynapses = getSynapses(neigborNeuronIdx);

            // Get the index of synapse point to current neuron and mark it as invalid synapse
            long long synapseIndexOfNN = numberOfNeighbors - i;

            // The synapse array need to be shifted regard to the remove neuron
            // Also neuron need to have 2M neighbors, the addtional synapse will be set as zero weight
            // Case1 [S0 S1 S2 - SR S5 S6]. SR is removed, [S0 S1 S2 S5 S6 0]
            // Case2 [S0 S1 SR - S3 S4 S5]. SR is removed, [0 S0 S1 S3 S4 S5]
            if (synapseIndexOfNN >= numberOfNeighbors / 2)
            {
                for (long long k = synapseIndexOfNN; k < numberOfNeighbors - 1; ++k)
                {
                    pNNSynapses[k] = pNNSynapses[k + 1];
                }
                pNNSynapses[numberOfNeighbors - 1].weight = 0;
            }
            else
            {
                for (long long k = synapseIndexOfNN; k > 0; --k)
                {
                    pNNSynapses[k] = pNNSynapses[k - 1];
                }
                pNNSynapses[0].weight = 0;
            }
        }

        // Shift the synapse array and the neuron array
        for (unsigned long long shiftIdx = neuronIdx; shiftIdx < currentANN.population; shiftIdx++)
        {
            currentANN.neurons[shiftIdx] = currentANN.neurons[shiftIdx + 1];

            // Also shift the synapses
            memcpy(getSynapses(shiftIdx), getSynapses(shiftIdx + 1), numberOfNeighbors * sizeof(Synapse));
        }
        currentANN.population--;
    }

    // Check if the target is on the left side of base
    int checkNeighborSide(unsigned long long base, unsigned long long target)
    {
        unsigned long long population = currentANN.population;

        if (base == target)
        {
            return kMid;
        }

        NeuronNeighborSide side = kMid;
        unsigned long long distance = 0;
        unsigned long long diff = 0;
        unsigned long long circularDiff = 0;
        if (target > base)
        {
            diff = target - base;
            circularDiff = population + base - target;
            if (diff < circularDiff)
            {
                distance = diff;
                side = kRight;
            }
            else
            {
                distance = circularDiff;
                side = kLeft;
            }
        }
        else
        {
            diff = base - target;
            circularDiff = population + target - base;
            if (diff < circularDiff)
            {
                distance = diff;
                side = kLeft;
            }
            else
            {
                distance = circularDiff;
                side = kRight;
            }
        }

        if (distance > numberOfNeighbors / 2)
        {
            return kOutOfBound;
        }
        return side;
    }

    void insertNeuron(unsigned long long synapseIdx)
    {
        // A synapse have incomingNeighbor and outgoingNeuron, direction incomingNeuron -> outgoingNeuron
        unsigned long long incomingNeighborSynapseIdx = synapseIdx % numberOfNeighbors;
        unsigned long long outgoingNeuron = synapseIdx / numberOfNeighbors;
        // The outgoing synapse of the newly added neuron points to the leftmost one.
        // The added neuron will be removed, so no further processing is needed.
        // For example: [incomingNeighbor N0 N1 outgoingNeuron ] with M = 3, [incomingNeighbor N0 N1 outgoingNeuron newNeuron]
        // new neuron will point to incomingNeighbor which is have distance > 3, it will be removed, so no need any further process
        if (incomingNeighborSynapseIdx == 0)
        {
            return;
        }

        Synapse* synapses = currentANN.synapses;
        Neuron* neurons = currentANN.neurons;
        unsigned long long& population = currentANN.population;

        // Copy original neuron to the inserted one and set it as  Neuron::kEvolution type
        Neuron insertNeuron;
        insertNeuron = neurons[outgoingNeuron];
        insertNeuron.type = Neuron::kEvolution;
        unsigned long long insertedNeuronIdx = outgoingNeuron + 1;

        char originalWeight = synapses[synapseIdx].weight;

        // Insert the neuron into array, population increased one, all neurons next to original one need to shift right
        for (unsigned long long i = population; i > outgoingNeuron; --i)
        {
            neurons[i] = neurons[i - 1];

            // Also shift the synapses to the right
            memcpy(getSynapses(i), getSynapses(i - 1), numberOfNeighbors * sizeof(Synapse));
        }
        neurons[insertedNeuronIdx] = insertNeuron;
        population++;

        // Try to update the synapse of inserted neuron. All outgoing synapse is init as zero weight
        Synapse* pInsertNeuronSynapse = getSynapses(insertedNeuronIdx);
        for (unsigned long long synIdx = 0; synIdx < numberOfNeighbors; ++synIdx)
        {
            pInsertNeuronSynapse[synIdx].weight = 0;
        }

        // Copy the outgoing synapse of original neuron
        // Outgoing points to the left
        if (incomingNeighborSynapseIdx < numberOfNeighbors / 2)
        {
            // Decrease by one because the new neuron is next to the original one
            pInsertNeuronSynapse[incomingNeighborSynapseIdx - 1].weight = originalWeight;
        }
        else
        {
            // No need to adjust the added neuron but need to remove the synapse of the original neuron
            pInsertNeuronSynapse[incomingNeighborSynapseIdx].weight = originalWeight;
        }

        // The change of synapse only impact neuron in [originalNeuronIdx - numberOfNeighbors / 2 + 1, originalNeuronIdx +  numberOfNeighbors / 2]
        // In the new index, it will be  [originalNeuronIdx + 1 - numberOfNeighbors / 2, originalNeuronIdx + 1 + numberOfNeighbors / 2]
        // [N0 N1 N2 original inserted N4 N5 N6], M = 2.
        for (long long delta = -numberOfNeighbors / 2; delta <= numberOfNeighbors / 2; ++delta)
        {
            unsigned long long updatedNeuronIdx = clampNeuronIndex(insertedNeuronIdx, delta);
            Synapse* pUpdatedSynapses = getSynapses(updatedNeuronIdx);

            if (delta < 0)
            {
                // [N1 N2 original inserted N4 N5 N6], M = 2.
                // [N1 N2 original]
                // Left side is kept as it is, only need to shift to the right side
                for (unsigned long long k = numberOfNeighbors / 2 ; k < numberOfNeighbors; k++)
                {
                    // Updated synapse
                    unsigned long long updatedNeuronNNIdx = clampNeuronIndex(updatedNeuronIdx, k);
                    if (kRight == checkNeighborSide(insertedNeuronIdx, updatedNeuronNNIdx))
                    {
                        pUpdatedSynapses[k] = pUpdatedSynapses[k - 1];
                    }
                }
            }
            else
            {
                // Right side is kept as it is, only need to shift to the left side
                for (unsigned long long k = 0; k < numberOfNeighbors / 2; k++)
                {
                    // Updated synapse
                    unsigned long long updatedNeuronNNIdx = clampNeuronIndex(updatedNeuronIdx, k);
                    if (updatedNeuronNNIdx == insertedNeuronIdx || kLeft == checkNeighborSide(insertedNeuronIdx, updatedNeuronNNIdx))
                    {
                        pUpdatedSynapses[k] = pUpdatedSynapses[k + 1];
                    }
                }
            }

        }
    }

    // Check which neurons/synapse need to be removed after mutation
    unsigned long long scanRedundantNeurons()
    {
        unsigned long long population = currentANN.population;
        Synapse* synapses = currentANN.synapses;
        Neuron* neurons = currentANN.neurons;

        unsigned long long numberOfRedundantNeurons = 0;
        // After each mutation, we must verify if there are neurons that do not affect the ANN output.
        // These are neurons that either have all incoming synapse weights as 0,
        // or all outgoing synapse weights as 0. Such neurons must be removed.
        for (unsigned long long i = 0; i < population; i++)
        {
            neurons[i].markForRemoval = false;
            if (neurons[i].type == Neuron::kEvolution)
            {
                bool allOutGoingZeros = true;
                bool allIncommingZeros = true;

                // Loop though its synapses for checkout outgoing synapses
                for (unsigned long long n = 0; n < numberOfNeighbors; n++)
                {
                    char synapseW = synapses[i * numberOfNeighbors + n].weight;
                    if (synapseW != 0)
                    {
                        allOutGoingZeros = false;
                        break;
                    }
                }

                // Loop through the neighbor neurons to check all incoming synapses
                for (unsigned long long n = 0; n <= numberOfNeighbors; n++)
                {
                    if (n == numberOfNeighbors / 2)
                    {
                        continue;
                    }
                    unsigned long long nnIdx = clampNeuronIndex(i + n, -numberOfNeighbors / 2);
                    char synapseW = synapses[nnIdx * numberOfNeighbors].weight;
                    if (synapseW != 0)
                    {
                        allIncommingZeros = false;
                        break;
                    }
                }
                if (allOutGoingZeros || allIncommingZeros)
                {
                    neurons[i].markForRemoval = true;
                    numberOfRedundantNeurons++;
                }
            }
        }
        return numberOfRedundantNeurons;
    }

    // Remove neurons and synapses that do not affect the ANN
    void cleanANN()
    {
        Synapse* synapses = currentANN.synapses;
        Neuron* neurons = currentANN.neurons;
        unsigned long long& population = currentANN.population;

        // Scan and remove neurons/synapses
        unsigned long long neuronIdx = 0;
        while (neuronIdx < population)
        {
            if (neurons[neuronIdx].markForRemoval)
            {
                // Remove it from the neuron list. Overwrite data
                // Remove its synapses in the synapses array
                removeNeuron(neuronIdx);
            }
            else
            {
                neuronIdx++;
            }
        }
    }

    void processTick()
    {
        unsigned long long population = currentANN.population;
        Synapse* synapses = currentANN.synapses;
        Neuron* neurons = currentANN.neurons;

        // Memset value of current one
        memset(neuronValueBuffer, 0, sizeof(neuronValueBuffer));

        // Loop though all neurons
        for (long long n = 0; n < population; ++n)
        {
            const Synapse* kSynapses = getSynapses(n);
            long long neuronValue = neurons[n].value;
            // Scan through all neighbor neurons and sum all connected neurons.
            // The synapses are arranged as neuronIndex * numberOfNeighbors
            for (long long m = 0; m < numberOfNeighbors; m++)
            {
                char synapseWeight = kSynapses[m].weight;
                unsigned long long nnIndex = 0 ;
                if ( m < numberOfNeighbors / 2)
                {
                    nnIndex = clampNeuronIndex(n + m, -numberOfNeighbors / 2);
                }
                else
                {
                    nnIndex = clampNeuronIndex(n + m + 1, -numberOfNeighbors / 2);
                }

                // Weight-sum
                neuronValueBuffer[nnIndex] += synapseWeight * neuronValue;
            }
        }

        // Clamp the neuron value
        for (long long n = 0; n < population; ++n)
        {
            long long neuronValue = clampNeuron(neuronValueBuffer[n]);
            neurons[n].value = neuronValue;
        }
    }

    void runTickSimulation()
    {
        unsigned long long population = currentANN.population;
        Synapse* synapses = currentANN.synapses;
        Neuron* neurons = currentANN.neurons;

        // Save the neuron value for comparison
        for (unsigned long long i = 0 ; i < population; ++i)
        {
            // Backup the neuron value
            previousNeuronValue[i] = neurons[i].value;
        }

        for(unsigned long long tick = 0; tick < numberOfTicks; ++tick)
        {
            processTick();
            // Check exit conditions:
            // - N ticks have passed (already in for loop)
            // - All neuron values are unchanged
            // - All output neurons have non-zero values
            bool shouldExit = true;
            bool allNeuronsUnchanged = true;
            bool allOutputNeuronsIsNonZeros = true;
            for (long long n = 0; n < population; ++n)
            {
                // Neuron unchanged check
                if (previousNeuronValue[n] != neurons[n].value)
                {
                    allNeuronsUnchanged = false;
                }

                // Ouput neuron value check
                if (neurons[n].type == Neuron::kOutput && neurons[n].value == 0)
                {
                    allOutputNeuronsIsNonZeros = false;
                }
            }

            if (allOutputNeuronsIsNonZeros || allNeuronsUnchanged)
            {
                break;
            }

            // Copy the neuron value
            for (long long n = 0; n < population; ++n)
            {
                previousNeuronValue[n] = neurons[n].value;
            }
        }
    }

    unsigned int computeNonMatchingOutput()
    {
        unsigned long long population = currentANN.population;
        Neuron* neurons = currentANN.neurons;

        // Compute the non-matching value R between output neuron value and initial value
        // Because the output neuron order never changes, the order is preserved
        unsigned int R = 0;
        unsigned long long outputIdx = 0;
        for (unsigned long long i = 0; i < population; i++)
        {
            if (neurons[i].type == Neuron::kOutput)
            {
                if (neurons[i].value != outputNeuronExpectedValue[outputIdx])
                {
                    R++;
                }
                outputIdx++;
            }
        }
        return R;
    }

    void generateMiningData(unsigned char miningSeed[32])
    {
        // Init the neuron input and expected output value
        random2(miningSeed, (unsigned char*)&miningData, sizeof(miningData));
    }

    void initInputNeuron()
    {
        unsigned long long population = currentANN.population;
        Neuron* neurons = currentANN.neurons;
        unsigned long long inputNeuronInitIndex = 0;
        for (unsigned long long i = 0 ; i < population; ++i)
        {
            // Input will use the init value
            if (neurons[i].type == Neuron::kInput)
            {
                char neuronValue = 0;
                unsigned char randomValue = miningData.inputNeuronRandomNumber[inputNeuronInitIndex];
                inputNeuronInitIndex++;
                if (randomValue % 3 == 0)
                {
                    neuronValue = 0;
                }
                else if (randomValue % 3 == 1)
                {
                    neuronValue = 1;
                }
                else
                {
                    neuronValue = -1;
                }

                // Convert value of neuron to trits (keeping 1 as 1, and changing 0 to -1.).
                neurons[i].value = (neuronValue == 0) ? -1 : neuronValue;
            }
        }
    }

    void initOutputNeuron()
    {
        unsigned long long population = currentANN.population;
        Neuron* neurons = currentANN.neurons;
        for (unsigned long long i = 0 ; i < population; ++i)
        {
            if (neurons[i].type == Neuron::kOutput)
            {
                neurons[i].value = 0;
            }
        }
    }

    void initExpectedOutputNeuron()
    {
        for (unsigned long long i = 0 ; i < numberOfOutputNeurons; ++i)
        {
            char neuronValue = 0;
            unsigned char randomNumber = miningData.outputNeuronRandomNumber[i];
            if (randomNumber % 3 == 0)
            {
                neuronValue = 0;
            }
            else if (randomNumber % 3 == 1)
            {
                neuronValue = 1;
            }
            else
            {
                neuronValue = -1;
            }

            outputNeuronExpectedValue[i] = neuronValue;
        }
    }

    unsigned int initializeANN(unsigned char nonce[32])
    {
        unsigned long long& population = currentANN.population;
        Synapse* synapses = currentANN.synapses;
        Neuron* neurons = currentANN.neurons;

        // Initialization
        population = numberOfNeurons;

        // Initalize with nonce and public key
        static_assert(sizeof(InitValue) % 8 == 0, "InitValue size must divided by 8");
        random2(computorPublicKey, nonce, (unsigned char*)&initValue, sizeof(InitValue));

        // Randomly choose the positions of neurons types
        for (unsigned long long i = 0 ; i < population; ++i)
        {
            neuronIndices[i] = i;
            neurons[i].type = Neuron::kInput;
        }
        unsigned long long neuronCount = population;
        for (unsigned long long i = 0 ; i < numberOfOutputNeurons; ++i)
        {
            unsigned long long outputNeuronIdx = initValue.outputNeuronPositions[i] % neuronCount;

            // Fill the neuron type
            neurons[neuronIndices[outputNeuronIdx]].type = Neuron::kOutput;
            outputNeuronIndices[i] = neuronIndices[outputNeuronIdx];

            // This index is used, copy the end of indices array to current position and decrease the number of picking neurons
            neuronCount = neuronCount - 1;
            neuronIndices[outputNeuronIdx] = neuronIndices[neuronCount];
        }

        // Synapse weight initialization
        const unsigned long long initNumberOfSynapses = population * numberOfNeighbors;
        for (unsigned long long i = 0 ; i < initNumberOfSynapses; ++i)
        {
            if (initValue.synapseWeight[i] % 3 == 0)
            {
                synapses[i].weight = 0;
            }
            else if (initValue.synapseWeight[i] % 3 == 1)
            {
                synapses[i].weight = 1;
            }
            else if (initValue.synapseWeight[i] % 3 == 2)
            {
                synapses[i].weight = -1;
            }
        }

        // Generate data using mining seed
        generateMiningData(currentRandomSeed);

        // Init input neuron value and output neuron
        initInputNeuron();
        initOutputNeuron();

        // Init expected output neuron
        initExpectedOutputNeuron();

        // Ticks simulation
        runTickSimulation();

        // Copy the state for rollback later
        memcpy(&bestANN, &currentANN, sizeof(ANN));

        // Compute R and roll back if neccessary
        unsigned int R = computeNonMatchingOutput();

        return R;
    }

    // Main function for mining
    bool findSolution(unsigned char* publicKey, unsigned char* nonce)
    {
        memcpy(computorPublicKey, publicKey, sizeof(computorPublicKey));

        // Initialize
        unsigned int bestR = initializeANN(nonce);

        for (unsigned long long s = 0; s < numberOfMutations; ++s)
        {

            // Do the mutation
            mutate(nonce, s);

            // Exit if the number of population reaches the maximum allowed
            if (currentANN.population >= populationThreshold)
            {
                break;
            }

            // Ticks simulation
            runTickSimulation();

            // Compute R and roll back if neccessary
            unsigned int R = computeNonMatchingOutput();
            if (R > bestR)
            {
                // Roll back
                memcpy(&currentANN, &bestANN, sizeof(bestANN));
            }
            else
            {
                bestR = R;

                // Better R. Save the state
                memcpy(&bestANN, &currentANN, sizeof(bestANN));
            }

            assert(bestANN.population <= populationThreshold);
        }

        // Check score
        unsigned int score = numberOfOutputNeurons - bestR;
        if (score >= solutionThreshold)
        {
            return true;
        }

        return false;
    }
};

static std::atomic<char> state(0);

static unsigned char computorPublicKey[32];
static unsigned char randomSeed[32];
static std::atomic<long long> numberOfMiningIterations(0);
static std::atomic<unsigned int> numberOfFoundSolutions(0);
static std::queue<std::array<unsigned char, 32>> foundNonce;
std::mutex foundNonceLock;


#ifdef _MSC_VER
static BOOL WINAPI ctrlCHandlerRoutine(DWORD dwCtrlType)
{
    if (!state)
    {
        state = 1;
    }
    else // User force exit quickly
    {
        std::exit(1);
    }
    return TRUE;
}
#else
void ctrlCHandlerRoutine(int signum)
{
    if (!state)
    {
        state = 1;
    }
    else // User force exit quickly
    {
        std::exit(1);
    }
}
#endif

void consoleCtrlHandler()
{
#ifdef _MSC_VER
    SetConsoleCtrlHandler(ctrlCHandlerRoutine, TRUE);
#else
    signal(SIGINT, ctrlCHandlerRoutine);
#endif
}

int getSystemProcs()
{
#ifdef _MSC_VER
#else
#endif
    return 0;
}

template<unsigned long long num>
bool isZeros(const unsigned char* value)
{
    bool allZeros = true;
    for (unsigned long long i = 0; i < num; ++i)
    {
        if (value[i] != 0)
        {
            return false;
        }
    }
    return true;
}
typedef Miner<NUMBER_OF_INPUT_NEURONS, NUMBER_OF_OUTPUT_NEURONS, NUMBER_OF_TICKS, MAX_NEIGHBOR_NEURONS, POPULATION_THRESHOLD, NUMBER_OF_MUTATIONS, SOLUTION_THRESHOLD> ActiveMiner;

int miningThreadProc()
{
    std::unique_ptr<ActiveMiner> miner(new ActiveMiner());
    miner->initialize(randomSeed);

    std::array<unsigned char, 32> nonce;
    while (!state)
    {
        _rdrand64_step((unsigned long long*)&nonce.data()[0]);
        _rdrand64_step((unsigned long long*)&nonce.data()[8]);
        _rdrand64_step((unsigned long long*)&nonce.data()[16]);
        _rdrand64_step((unsigned long long*)&nonce.data()[24]);

        if (miner->findSolution(computorPublicKey, nonce.data()))
        {
            {
                std::lock_guard<std::mutex> guard(foundNonceLock);
                foundNonce.push(nonce);
            }
            numberOfFoundSolutions++;
        }

        numberOfMiningIterations++;
    }
    return 0;
}

struct ServerSocket
{
#ifdef _MSC_VER
    ServerSocket()
    {
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
    }

    ~ServerSocket()
    {
        WSACleanup();
    }

    void closeConnection()
    {
        closesocket(serverSocket);
    }

    bool establishConnection(char* address)
    {
        serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (serverSocket == INVALID_SOCKET)
        {
            printf("Fail to create a socket (%d)!\n", WSAGetLastError());
            return false;
        }

        sockaddr_in addr;
        ZeroMemory(&addr, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(nodePort);
        sscanf_s(address, "%hhu.%hhu.%hhu.%hhu", &addr.sin_addr.S_un.S_un_b.s_b1, &addr.sin_addr.S_un.S_un_b.s_b2, &addr.sin_addr.S_un.S_un_b.s_b3, &addr.sin_addr.S_un.S_un_b.s_b4);
        if (connect(serverSocket, (const sockaddr*)&addr, sizeof(addr)))
        {
            printf("Fail to connect to %d.%d.%d.%d (%d)!\n", addr.sin_addr.S_un.S_un_b.s_b1, addr.sin_addr.S_un.S_un_b.s_b2, addr.sin_addr.S_un.S_un_b.s_b3, addr.sin_addr.S_un.S_un_b.s_b4, WSAGetLastError());
            closeConnection();
            return false;
        }

        return true;
    }

    SOCKET serverSocket;
#else
    void closeConnection()
    {
        close(serverSocket);
    }
    bool establishConnection(char* address)
    {
        serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket == -1)
        {
            printf("Fail to create a socket (%d)!\n", errno);
            return false;
        }

        sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(nodePort);
        if (inet_pton(AF_INET, address, &addr.sin_addr) <= 0)
        {
            printf("Invalid address/ Address not supported (%s)\n", address);
            return false;
        }

        if (connect(serverSocket, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            printf("Fail to connect to %s (%d)\n", address, errno);
            closeConnection();
            return false;
        }

        return true;
    }

    int serverSocket;
#endif

    bool sendData(char* buffer, unsigned int size)
    {
        while (size)
        {
            int numberOfBytes;
            if ((numberOfBytes = send(serverSocket, buffer, size, 0)) <= 0)
            {
                return false;
            }
            buffer += numberOfBytes;
            size -= numberOfBytes;
        }

        return true;
    }
    bool receiveData(char* buffer, unsigned int size)
    {
        const auto beginningTime = std::chrono::steady_clock::now();
        unsigned long long deltaTime = 0;
        while (size && deltaTime <= 2000)
        {
            int numberOfBytes;
            if ((numberOfBytes = recv(serverSocket, buffer, size, 0)) <= 0)
            {
                return false;
            }
            buffer += numberOfBytes;
            size -= numberOfBytes;
            deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - beginningTime).count();
        }

        return true;
    }
};

static void hexToByte(const char* hex, uint8_t* byte, const int sizeInByte)
{
    for (int i = 0; i < sizeInByte; i++){
        sscanf(hex+i*2, "%2hhx", &byte[i]);
    }
}

int main(int argc, char* argv[])
{
    std::vector<std::thread> miningThreads;
    if (argc != 7)
    {
        printf("Usage:   Qiner [Node IP] [Node Port] [MiningID] [Signing Seed] [Mining Seed] [Number of threads]\n");
    }
    else
    {
        nodeIp = argv[1];
        nodePort = std::atoi(argv[2]);
        char* miningID = argv[3];
        printf("Qiner is launched. Connecting to %s:%d\n", nodeIp, nodePort);

        consoleCtrlHandler();

        {
            getPublicKeyFromIdentity(miningID, computorPublicKey);

            // Data for signing the solution
            char* signingSeed = argv[4];
            unsigned char signingPrivateKey[32];
            unsigned char signingSubseed[32];
            unsigned char signingPublicKey[32];
            char privateKeyQubicFormat[128] = {0};
            char publicKeyQubicFormat[128] = {0};
            char publicIdentity[128] = {0};
            getSubseedFromSeed((unsigned char*)signingSeed, signingSubseed);
            getPrivateKeyFromSubSeed(signingSubseed, signingPrivateKey);
            getPublicKeyFromPrivateKey(signingPrivateKey, signingPublicKey);

            //getIdentityFromPublicKey(signingPublicKey, miningID, false);

            hexToByte(argv[5], randomSeed, 32);
            unsigned int numberOfThreads = atoi(argv[6]);
            printf("%d threads are used.\n", numberOfThreads);
            miningThreads.reserve(numberOfThreads);
            for (unsigned int i = numberOfThreads; i-- > 0; )
            {
                miningThreads.emplace_back(miningThreadProc);
            }
            ServerSocket serverSocket;

            auto timestamp = std::chrono::steady_clock::now();
            long long prevNumberOfMiningIterations = 0;
            while (!state)
            {
                bool haveNonceToSend = false;
                size_t itemToSend = 0;
                std::array<unsigned char, 32> sendNonce;
                {
                    std::lock_guard<std::mutex> guard(foundNonceLock);
                    haveNonceToSend = foundNonce.size() > 0;
                    if (haveNonceToSend)
                    {
                        sendNonce = foundNonce.front();
                    }
                    itemToSend = foundNonce.size();
                }
                if (haveNonceToSend)
                {
                    if (serverSocket.establishConnection(nodeIp))
                    {
                        struct
                        {
                            RequestResponseHeader header;
                            Message message;
                            unsigned char solutionMiningSeed[32];
                            unsigned char solutionNonce[32];
                            unsigned char signature[64];
                        } packet;

                        packet.header.setSize(sizeof(packet));
                        packet.header.zeroDejavu();
                        packet.header.setType(BROADCAST_MESSAGE);

                        memcpy(packet.message.sourcePublicKey, signingPublicKey, sizeof(packet.message.sourcePublicKey));
                        memcpy(packet.message.destinationPublicKey, computorPublicKey, sizeof(packet.message.destinationPublicKey));

                        unsigned char sharedKeyAndGammingNonce[64];
                        // Default behavior when provided seed is just a signing address
                        // first 32 bytes of sharedKeyAndGammingNonce is set as zeros
                        memset(sharedKeyAndGammingNonce, 0, 32);
                        // If provided seed is the for computor public key, generate sharedKey into first 32 bytes to encrypt message
                        if (memcmp(computorPublicKey, signingPublicKey, 32) == 0)
                        {
                            getSharedKey(signingPrivateKey, computorPublicKey, sharedKeyAndGammingNonce);
                        }
                        // Last 32 bytes of sharedKeyAndGammingNonce is randomly created so that gammingKey[0] = 0 (MESSAGE_TYPE_SOLUTION)
                        unsigned char gammingKey[32];
                        do
                        {
                            _rdrand64_step((unsigned long long*) & packet.message.gammingNonce[0]);
                            _rdrand64_step((unsigned long long*) & packet.message.gammingNonce[8]);
                            _rdrand64_step((unsigned long long*) & packet.message.gammingNonce[16]);
                            _rdrand64_step((unsigned long long*) & packet.message.gammingNonce[24]);
                            memcpy(&sharedKeyAndGammingNonce[32], packet.message.gammingNonce, 32);
                            KangarooTwelve(sharedKeyAndGammingNonce, 64, gammingKey, 32);
                        } while (gammingKey[0]);

                        // Encrypt the message payload
                        unsigned char gamma[32 + 32];
                        KangarooTwelve(gammingKey, sizeof(gammingKey), gamma, sizeof(gamma));
                        for (unsigned int i = 0; i < 32; i++)
                        {
                            packet.solutionMiningSeed[i] = randomSeed[i] ^ gamma[i];
                            packet.solutionNonce[i] = sendNonce[i] ^ gamma[i + 32];
                        }

                        // Sign the message
                        uint8_t digest[32] = {0};
                        uint8_t signature[64] = {0};
                        KangarooTwelve(
                            (unsigned char*)&packet + sizeof(RequestResponseHeader),
                            sizeof(packet) - sizeof(RequestResponseHeader) - 64,
                            digest,
                            32);
                        sign(signingSubseed, signingPublicKey, digest, signature);
                        memcpy(packet.signature, signature, 64);

                        // Send message
                        if (serverSocket.sendData((char*)&packet, packet.header.size()))
                        {
                            std::lock_guard<std::mutex> guard(foundNonceLock);
                            // Send data successfully. Remove it from the queue
                            foundNonce.pop();
                            itemToSend = foundNonce.size();
                        }
                        serverSocket.closeConnection();
                    }
                }

                std::this_thread::sleep_for(std::chrono::duration < double, std::milli>(1000));

                unsigned long long delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timestamp).count();
                if (delta >= 1000)
                {
                    // Get current time in UTC
                    std::time_t now_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                    std::tm* utc_time = std::gmtime(&now_time);
                    printf("|   %04d-%02d-%02d %02d:%02d:%02d   |   %llu it/s   |   %d solutions   |   %.10s...   |\n",
                        utc_time->tm_year + 1900, utc_time->tm_mon, utc_time->tm_mday, utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec,
                        (numberOfMiningIterations - prevNumberOfMiningIterations) * 1000 / delta, numberOfFoundSolutions.load(), miningID);
                    prevNumberOfMiningIterations = numberOfMiningIterations;
                    timestamp = std::chrono::steady_clock::now();
                }
            }
        }
        printf("Shutting down...Press Ctrl+C again to force stop.\n");

        // Wait for all threads to join
        for (auto& miningTh : miningThreads)
        {
            if (miningTh.joinable())
            {
                miningTh.join();
            }
        }
        printf("Qiner is shut down.\n");
    }

    return 0;
}