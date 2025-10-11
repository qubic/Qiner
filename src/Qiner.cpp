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

constexpr unsigned long long POOL_VEC_SIZE =  (((1ULL<<32) + 64)) >> 3; // 2^32+64 bits ~ 512MB
constexpr unsigned long long POOL_VEC_PADDING_SIZE = (POOL_VEC_SIZE + 200 - 1) / 200 * 200; // padding for multiple of 200

void generateRandom2Pool(unsigned char miningSeed[32], unsigned char* pool)
{
    unsigned char state[200];
    // same pool to be used by all computors/candidates and pool content changing each phase
    memcpy(&state[0], miningSeed, 32);
    memset(&state[32], 0, sizeof(state) - 32);

    for (unsigned int i = 0; i < POOL_VEC_PADDING_SIZE; i += sizeof(state))
    {
        KeccakP1600_Permute_12rounds(state);
        memcpy(&pool[i], state, sizeof(state));
    }
}

void random2(
    unsigned char seed[32],
    const unsigned char* pool,
    unsigned char* output,
    unsigned long long outputSizeInByte)
{
    unsigned long long paddingOutputSize = (outputSizeInByte + 64 - 1) / 64;
    paddingOutputSize = paddingOutputSize * 64;
    std::vector<unsigned char> paddingOutputVec(paddingOutputSize);
    unsigned char* paddingOutput = paddingOutputVec.data();

    unsigned long long segments = paddingOutputSize / 64;
    unsigned int x[8] = { 0 };
    for (int i = 0; i < 8; i++)
    {
        x[i] = ((unsigned int*)seed)[i];
    }

    for (int j = 0; j < segments; j++)
    {
        // Each segment will have 8 elements. Each element have 8 bytes
        for (int i = 0; i < 8; i++)
        {
            unsigned int base = (x[i] >> 3) >> 3;
            unsigned int m = x[i] & 63;

            unsigned long long u64_0 = ((unsigned long long*)pool)[base];
            unsigned long long u64_1 = ((unsigned long long*)pool)[base + 1];

            // Move 8 * 8 * j to the current segment. 8 * i to current 8 bytes element
            if (m == 0)
            {
                // some compiler doesn't work with bit shift 64
                *((unsigned long long*) & paddingOutput[j * 8 * 8 + i * 8]) = u64_0;
            }
            else
            {
                *((unsigned long long*) & paddingOutput[j * 8 * 8 + i * 8]) = (u64_0 >> m) | (u64_1 << (64 - m));
            }

            // Increase the positions in the pool for each element.
            x[i] = x[i] * 1664525 + 1013904223; // https://en.wikipedia.org/wiki/Linear_congruential_generator#Parameters_in_common_use
        }
    }

    memcpy(output, paddingOutput, outputSizeInByte);
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
static constexpr unsigned long long NUMBER_OF_INPUT_NEURONS = 256;     // K
static constexpr unsigned long long NUMBER_OF_OUTPUT_NEURONS = 256;    // L
static constexpr unsigned long long NUMBER_OF_TICKS = 120;               // N
static constexpr unsigned long long MAX_NEIGHBOR_NEURONS = 256; // 2M. Must divided by 2
static constexpr unsigned long long NUMBER_OF_MUTATIONS = 100;
static constexpr unsigned long long POPULATION_THRESHOLD = NUMBER_OF_INPUT_NEURONS + NUMBER_OF_OUTPUT_NEURONS + NUMBER_OF_MUTATIONS; // P
static constexpr unsigned int SOLUTION_THRESHOLD = NUMBER_OF_OUTPUT_NEURONS * 4 / 5;

// Compute threshold for clamp
template <typename T>
void computeNeuronValueThreshold(T N, T *L_out, T *R_out) 
{
    static_assert(std::is_integral_v<T>, "T must be an integer type");

    T numberOfValues = 2 * N + 1;
    T a = (numberOfValues + 2) / 3; // divup
    *L_out = -N + a;
    *R_out = N - a;
}

// Clamp the neuron value
template <typename T>
T clampNeuron(T neuronValue, T left, T right)
{
    static_assert(std::is_integral_v<T>, "T must be an integer type");

    if (neuronValue > right)
    {
        return 1;
    }

    if (neuronValue < left)
    {
        return -1;
    }
    return 0;
}

void extract64Bits(unsigned long long number, char* output)
{
    int count = 0;
    for (int i = 0; i < 64; ++i)
    {
        output[i] = ((number >> i) & 1);
    }
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
    static constexpr unsigned long long initNumberOfSynapses = numberOfNeurons * numberOfNeighbors;

    static_assert(numberOfInputNeurons % 64 == 0, "numberOfInputNeurons must be divided by 64");
    static_assert(numberOfOutputNeurons % 64 == 0, "numberOfOutputNeurons must be divided by 64");
    static_assert(maxNumberOfSynapses <= (0xFFFFFFFFFFFFFFFF << 1ULL), "maxNumberOfSynapses must less than or equal MAX_UINT64/2");
    static_assert(initNumberOfSynapses % 32 == 0, "initNumberOfSynapses must be divided by 32");
    static_assert(numberOfNeighbors % 2 == 0, "numberOfNeighbors must divided by 2");
    static_assert(populationThreshold > numberOfNeurons, "populationThreshold must be greater than numberOfNeurons");
    static_assert(numberOfNeurons > numberOfNeighbors, "Number of neurons must be greater than the number of neighbors");

    std::vector<unsigned char> poolVec;

    void initialize(unsigned char miningSeed[32])
    {
        // Init random2 pool with mining seed
        poolVec.resize(POOL_VEC_PADDING_SIZE);
        generateRandom2Pool(miningSeed, poolVec.data());
    }

    struct Synapse
    {
        char weight;
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
        long long clampLeftBoundary;
        long long clampRightBoundary;
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
        unsigned long long synapseWeight[initNumberOfSynapses / 32]; // each 64bits elements will decide value of 32 synapses
        unsigned long long synpaseMutation[numberOfMutations];
    } initValue;

    struct MiningData
    {
        unsigned long long inputNeuronRandomNumber[numberOfInputNeurons / 64 ];  // each bit will use for generate input neuron value
        unsigned long long outputNeuronRandomNumber[numberOfOutputNeurons / 64]; // each bit will use for generate expected output neuron value
    } miningData;

    unsigned long long neuronIndices[numberOfNeurons];
    char previousNeuronValue[maxNumberOfNeurons];

    unsigned long long outputNeuronIndices[numberOfOutputNeurons];
    char outputNeuronExpectedValue[numberOfOutputNeurons];

    long long neuronValueBuffer[maxNumberOfNeurons];
    long long numberOfNonZeroInputSynapse[maxNumberOfNeurons];

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
        for (long long neighborOffset = -(long long)numberOfNeighbors / 2; neighborOffset <= (long long)numberOfNeighbors / 2; neighborOffset++)
        {
            unsigned long long nnIdx = clampNeuronIndex(neuronIdx, neighborOffset);
            Synapse* pNNSynapses = getSynapses(nnIdx);

            long long synapseIndexOfNN = getIndexInSynapsesBuffer(nnIdx, -neighborOffset);
            if (synapseIndexOfNN < 0)
            {
                continue;
            }

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

    unsigned long long getNeighborNeuronIndex(unsigned long long neuronIndex, unsigned long long neighborOffset)
    {
        unsigned long long nnIndex = 0;
        if (neighborOffset < (numberOfNeighbors / 2))
        {
            nnIndex = clampNeuronIndex(neuronIndex + neighborOffset, -(long long)numberOfNeighbors / 2);
        }
        else
        {
            nnIndex = clampNeuronIndex(neuronIndex + neighborOffset + 1, -(long long)numberOfNeighbors / 2);
        }
        return nnIndex;
    }

    void insertNeuron(unsigned long long synapseIdx)
    {
        // A synapse have incomingNeighbor and outgoingNeuron, direction incomingNeuron -> outgoingNeuron
        unsigned long long incomingNeighborSynapseIdx = synapseIdx % numberOfNeighbors;
        unsigned long long outgoingNeuron = synapseIdx / numberOfNeighbors;

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
            if (incomingNeighborSynapseIdx > 0)
            {
                // Decrease by one because the new neuron is next to the original one
                pInsertNeuronSynapse[incomingNeighborSynapseIdx - 1].weight = originalWeight;
            }
            // Incase of the outgoing synapse point too far, don't add the synapse
        }
        else
        {
            // No need to adjust the added neuron but need to remove the synapse of the original neuron
            pInsertNeuronSynapse[incomingNeighborSynapseIdx].weight = originalWeight;
        }

        // The change of synapse only impact neuron in [originalNeuronIdx - numberOfNeighbors / 2 + 1, originalNeuronIdx +  numberOfNeighbors / 2]
        // In the new index, it will be  [originalNeuronIdx + 1 - numberOfNeighbors / 2, originalNeuronIdx + 1 + numberOfNeighbors / 2]
        // [N0 N1 N2 original inserted N4 N5 N6], M = 2.
        for (long long delta = -(long long)numberOfNeighbors / 2; delta <= (long long)numberOfNeighbors / 2; ++delta)
        {
            // Only process the neigbors
            if (delta == 0)
            {
                continue;
            }
            unsigned long long updatedNeuronIdx = clampNeuronIndex(insertedNeuronIdx, delta);

            // Generate a list of neighbor index of current updated neuron NN
            // Find the location of the inserted neuron in the list of neighbors
            long long insertedNeuronIdxInNeigborList = -1;
            for (long long k = 0 ; k < numberOfNeighbors; k++)
            {
                unsigned long long nnIndex = getNeighborNeuronIndex(updatedNeuronIdx, k);
                if (nnIndex == insertedNeuronIdx)
                {
                    insertedNeuronIdxInNeigborList = k;
                }
            }

            assert(insertedNeuronIdxInNeigborList >= 0);

            Synapse* pUpdatedSynapses = getSynapses(updatedNeuronIdx);
            // [N0 N1 N2 original inserted N4 N5 N6], M = 2.
            // Case: neurons in range [N0 N1 N2 original], right synapses will be affected
            if (delta < 0)
            {
                // Left side is kept as it is, only need to shift to the right side
                for (long long k = numberOfNeighbors - 1 ; k >= insertedNeuronIdxInNeigborList; --k)
                {
                    // Updated synapse
                    pUpdatedSynapses[k] = pUpdatedSynapses[k - 1];
                }

                // Incomming synapse from original neuron -> inserted neuron must be zero
                if (delta == -1)
                {
                    pUpdatedSynapses[insertedNeuronIdxInNeigborList].weight = 0;
                }
            }
            else // Case: neurons in range [inserted N4 N5 N6], left synapses will be affected
            {
                // Right side is kept as it is, only need to shift to the left side
                for (unsigned long long k = 0; k < insertedNeuronIdxInNeigborList; ++k)
                {
                    // Updated synapse
                    pUpdatedSynapses[k] = pUpdatedSynapses[k + 1];
                }
            }

        }
    }

    long long getIndexInSynapsesBuffer(unsigned long long neuronIdx, long long neighborOffset)
    {
        // Skip the case neuron point to itself and too far neighbor
        if (neighborOffset == 0
            || neighborOffset < -(long long)numberOfNeighbors / 2
            || neighborOffset > (long long)numberOfNeighbors / 2)
        {
            return -1;
        }

        long long synapseIdx = (long long)numberOfNeighbors / 2 + neighborOffset;
        if (neighborOffset >= 0)
        {
            synapseIdx = synapseIdx - 1;
        }

        return synapseIdx;
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
                for (long long neighborOffset = -(long long)numberOfNeighbors / 2; neighborOffset <= (long long)numberOfNeighbors / 2; neighborOffset++)
                {
                    unsigned long long nnIdx = clampNeuronIndex(i, neighborOffset);
                    Synapse* nnSynapses = getSynapses(nnIdx);

                    long long synapseIdx = getIndexInSynapsesBuffer(nnIdx, -neighborOffset);
                    if (synapseIdx < 0)
                    {
                        continue;
                    }
                    char synapseW = nnSynapses[synapseIdx].weight;

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
                    nnIndex = clampNeuronIndex(n + m, -(long long)numberOfNeighbors / 2);
                }
                else
                {
                    nnIndex = clampNeuronIndex(n + m + 1, -(long long)numberOfNeighbors / 2);
                }

                // Weight-sum
                neuronValueBuffer[nnIndex] += synapseWeight * neuronValue;
            }
        }

        // Clamp the neuron value and update
        for (long long n = 0; n < population; ++n)
        {
            // Skip updating input neuron
            if (neurons[n].type != Neuron::kInput)
            {
                long long neuronValue = clampNeuron(neuronValueBuffer[n], neurons[n].clampLeftBoundary, neurons[n].clampRightBoundary);
                neurons[n].value = neuronValue;
            }
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

        // Compute the clamp threshold
        // Loop though all neurons to find number of non-zero input synapses of neurons
        memset(numberOfNonZeroInputSynapse, 0, sizeof(numberOfNonZeroInputSynapse));
        for (long long n = 0; n < population; ++n)
        {
            const Synapse* kSynapses = getSynapses(n);
            // Scan through all neighbor neurons and sum all connected neurons.
            // The synapses are arranged as neuronIndex * numberOfNeighbors
            for (long long m = 0; m < numberOfNeighbors; m++)
            {
                char synapseWeight = kSynapses[m].weight;
                unsigned long long nnIndex = 0 ;
                if ( m < numberOfNeighbors / 2)
                {
                    nnIndex = clampNeuronIndex(n + m, -(long long)numberOfNeighbors / 2);
                }
                else
                {
                    nnIndex = clampNeuronIndex(n + m + 1, -(long long)numberOfNeighbors / 2);
                }

                if (synapseWeight != 0)
                {
                    numberOfNonZeroInputSynapse[nnIndex]++;
                }
            }
        }
        
        for (long long n = 0; n < population; ++n)
        {
            neurons[n].clampLeftBoundary = 0;
            neurons[n].clampRightBoundary = 0;
            if (Neuron::kInput != neurons[n].type)
            {
                computeNeuronValueThreshold(numberOfNonZeroInputSynapse[n], &neurons[n].clampLeftBoundary, &neurons[n].clampRightBoundary);
            }
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

    void initInputNeuron()
    {
        unsigned long long population = currentANN.population;
        Neuron* neurons = currentANN.neurons;
        unsigned long long inputNeuronInitIndex = 0;

        char neuronArray[64] = { 0 };
        for (unsigned long long i = 0 ; i < population; ++i)
        {
            // Input will use the init value
            if (neurons[i].type == Neuron::kInput)
            {
                // Prepare new pack
                if (inputNeuronInitIndex % 64 == 0)
                {
                    extract64Bits(miningData.inputNeuronRandomNumber[inputNeuronInitIndex / 64], neuronArray);
                }
                char neuronValue = neuronArray[inputNeuronInitIndex % 64];

                // Convert value of neuron to trits (keeping 1 as 1, and changing 0 to -1.).
                neurons[i].value = (neuronValue == 0) ? -1 : neuronValue;

                inputNeuronInitIndex++;
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
        char neuronArray[64] = { 0 };
        for (unsigned long long i = 0 ; i < numberOfOutputNeurons; ++i)
        {
            // Prepare new pack
            if (i % 64 == 0)
            {
                extract64Bits(miningData.outputNeuronRandomNumber[i / 64], neuronArray);
            }
            char neuronValue = neuronArray[i % 64];
            // Convert value of neuron (keeping 1 as 1, and changing 0 to -1.).
            outputNeuronExpectedValue[i] = (neuronValue == 0) ? -1 : neuronValue;
        }
    }

    unsigned int initializeANN(unsigned char* publicKey, unsigned char* nonce)
    {
        unsigned char hash[32];
        unsigned char combined[64];
        memcpy(combined, publicKey, 32);
        memcpy(combined + 32, nonce, 32);
        KangarooTwelve(combined, 64, hash, 32);

        unsigned long long& population = currentANN.population;
        Synapse* synapses = currentANN.synapses;
        Neuron* neurons = currentANN.neurons;

        // Initialization
        population = numberOfNeurons;

        // Initalize with nonce and public key
        random2(hash, poolVec.data(), (unsigned char*)&initValue, sizeof(InitValue));

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
        for (unsigned long long i = 0 ; i < (initNumberOfSynapses / 32); ++i)
        {
            const unsigned long long mask = 0b11;

            for (int j = 0; j < 32; ++j)
            {
                int shiftVal = j * 2;
                unsigned char extractValue = (unsigned char)((initValue.synapseWeight[i] >> shiftVal) & mask);
                switch (extractValue)
                {
                    case 2: synapses[32 * i + j].weight = -1; break;
                    case 3: synapses[32 * i + j].weight = 1; break;
                    default: synapses[32 * i + j].weight = 0;
                }
            }
        }

        // Init the neuron input and expected output value
        memcpy((unsigned char*)&miningData, poolVec.data(), sizeof(miningData));

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
        // Initialize
        unsigned int bestR = initializeANN(publicKey, nonce);

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