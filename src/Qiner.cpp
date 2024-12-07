#include <chrono>
#include <thread>
#include <mutex>
#include <cstdio>
#include <cstring>
#include <array>
#include <queue>
#include <atomic>
#include <vector>
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
static constexpr unsigned long long DATA_LENGTH = 256;
static constexpr unsigned long long NUMBER_OF_HIDDEN_NEURONS = 3000;
static constexpr unsigned long long NUMBER_OF_NEIGHBOR_NEURONS = 3000;
static constexpr unsigned long long MAX_DURATION = 3000*3000;
static constexpr unsigned long long NUMBER_OF_OPTIMIZATION_STEPS = 100;
static constexpr unsigned int SOLUTION_THRESHOLD = 130;

static_assert(((DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH)* NUMBER_OF_NEIGHBOR_NEURONS) % 64 == 0, "Synapse size need to be a multipler of 64");
static_assert(NUMBER_OF_OPTIMIZATION_STEPS < MAX_DURATION, "Number of retries need to smaller than MAX_DURATION");

struct Miner
{
    long long data[DATA_LENGTH];
    unsigned char computorPublicKey[32];
    unsigned char currentRandomSeed[32];

    void initialize(unsigned char randomSeed[32])
    {
        random(randomSeed, randomSeed, (unsigned char*)data, sizeof(data));
        for (unsigned long long i = 0; i < DATA_LENGTH; i++)
        {
            data[i] = (data[i] >= 0 ? 1 : -1);
        }

        memcpy(currentRandomSeed, randomSeed, sizeof(currentRandomSeed));
        memset(computorPublicKey, 0, sizeof(computorPublicKey));
    }

    void getComputorPublicKey(unsigned char computorPublicKey[32])
    {
        memcpy(computorPublicKey, this->computorPublicKey, sizeof(this->computorPublicKey));
    }

    void setComputorPublicKey(unsigned char computorPublicKey[32])
    {
        memcpy(this->computorPublicKey, computorPublicKey, sizeof(this->computorPublicKey));
    }

    struct
    {
        long long input[DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH];
    } neurons;
    struct
    {
        unsigned long long signs[(DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH) * NUMBER_OF_NEIGHBOR_NEURONS / 64];
        unsigned long long sequence[MAX_DURATION];
        // Use for randomly select skipped ticks
        unsigned long long skipTicksNumber[NUMBER_OF_OPTIMIZATION_STEPS];
    } synapses;

    // Save skipped ticks
    long long skipTicks[NUMBER_OF_OPTIMIZATION_STEPS];

    // Contained all ticks possible value
    long long ticksNumbers[MAX_DURATION];

    // Main function for mining
    bool findSolution(unsigned char nonce[32])
    {
        _rdrand64_step((unsigned long long*)&nonce[0]);
        _rdrand64_step((unsigned long long*)&nonce[8]);
        _rdrand64_step((unsigned long long*)&nonce[16]);
        _rdrand64_step((unsigned long long*)&nonce[24]);
        random2(computorPublicKey, nonce, (unsigned char*)&synapses, sizeof(synapses));

        unsigned int score = 0;
        long long tailTick = MAX_DURATION - 1;
        for (long long tick = 0; tick < MAX_DURATION; tick++)
        {
            ticksNumbers[tick] = tick;
        }

        for (long long l = 0; l < NUMBER_OF_OPTIMIZATION_STEPS; l++)
        {
            skipTicks[l] = -1LL;
        }

        // Calculate the score with a list of randomly skipped ticks. This list grows if an additional skipped tick
        // does not worsen the score compared to the previous one.
        // - Initialize skippedTicks = []
        // - First, use all ticks. Compute score0 and update the score with score0.
        // - In the second run, ignore ticks in skippedTicks and try skipping a random tick 'a'.
        //    + Compute score1.
        //    + If score1 is not worse than score, add tick 'a' to skippedTicks and update the score with score1.
        //    + Otherwise, ignore tick 'a'.
        // - In the third run, ignore ticks in skippedTicks and try skipping a random tick 'b'.
        //    + Compute score2.
        //    + If score2 is not worse than score, add tick 'b' to skippedTicks and update the score with score2.
        //    + Otherwise, ignore tick 'b'.
        // - Continue this process iteratively.
        unsigned long long numberOfSkippedTicks = 0;
        long long skipTick = -1;
        for (long long l = 0; l < NUMBER_OF_OPTIMIZATION_STEPS; l++)
        {
            memset(&neurons, 0, sizeof(neurons));
            memcpy(&neurons.input[0], data, sizeof(data));

            for (long long tick = 0; tick < MAX_DURATION; tick++)
            {
                // Check if current tick should be skipped
                if (tick == skipTick)
                {
                    continue;
                }

                // Skip recorded skipped ticks
                bool tickShouldBeSkipped = false;
                for (long long tickIdx = 0; tickIdx < numberOfSkippedTicks; tickIdx++)
                {
                    if (skipTicks[tickIdx] == tick)
                    {
                        tickShouldBeSkipped = true;
                        break;
                    }
                }
                if (tickShouldBeSkipped)
                {
                    continue;
                }

                // Compute neurons
                const unsigned long long neuronIndex = DATA_LENGTH + synapses.sequence[tick] % (NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH);
                const unsigned long long neighborNeuronIndex = (synapses.sequence[tick] / (NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH)) % NUMBER_OF_NEIGHBOR_NEURONS;
                unsigned long long supplierNeuronIndex;
                if (neighborNeuronIndex < NUMBER_OF_NEIGHBOR_NEURONS / 2)
                {
                    supplierNeuronIndex = (neuronIndex - (NUMBER_OF_NEIGHBOR_NEURONS / 2) + neighborNeuronIndex + (DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH)) % (DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH);
                }
                else
                {
                    supplierNeuronIndex = (neuronIndex + 1 - (NUMBER_OF_NEIGHBOR_NEURONS / 2) + neighborNeuronIndex + (DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH)) % (DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH);
                }
                const unsigned long long offset = neuronIndex * NUMBER_OF_NEIGHBOR_NEURONS + neighborNeuronIndex;

                if (!(synapses.signs[offset / 64] & (1ULL << (offset % 64))))
                {
                    neurons.input[neuronIndex] += neurons.input[supplierNeuronIndex];
                }
                else
                {
                    neurons.input[neuronIndex] -= neurons.input[supplierNeuronIndex];
                }

                if (neurons.input[neuronIndex] > 1)
                {
                    neurons.input[neuronIndex] = 1;
                }
                if (neurons.input[neuronIndex] < -1)
                {
                    neurons.input[neuronIndex] = -1;
                }
            }

            // Compute the score
            unsigned int currentScore = 0;
            for (unsigned long long i = 0; i < DATA_LENGTH; i++)
            {
                if (data[i] == neurons.input[DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + i])
                {
                    currentScore++;
                }
            }

            // Update score if below satisfied
            // - This is the first run without skipping any ticks
            // - Current score is not worse than previous score
            if (skipTick == -1 || currentScore >= score)
            {
                score = currentScore;
                // For the first run, don't need to update the skipped ticks list
                if (skipTick != -1 )
                {
                    skipTicks[numberOfSkippedTicks] = skipTick;
                    numberOfSkippedTicks++;
                }
            }

            // Randomly choose a tick to skip for the next round and avoid duplicated pick already chosen one
            long long randomTick = synapses.skipTicksNumber[l] % (MAX_DURATION - l);
            skipTick = ticksNumbers[randomTick];
            // Replace the chosen tick position with current tail to make sure if this tick is not chosen again
            // the skipTick is still not duplicated with previous ones.
            ticksNumbers[randomTick] = ticksNumbers[tailTick];
            tailTick--;

        }

        // Check score
        if (score >= SOLUTION_THRESHOLD)
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

int miningThreadProc()
{
    std::unique_ptr<Miner> miner(new Miner());
    miner->initialize(randomSeed);
    miner->setComputorPublicKey(computorPublicKey);

    std::array<unsigned char, 32> nonce;
    while (!state)
    {
        if (miner->findSolution(nonce.data()))
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
    if (argc != 6)
    {
        printf("Usage:   Qiner [Node IP] [Node Port] [Seed] [Mining Seed] [Number of threads]\n");
    }
    else
    {
        nodeIp = argv[1];
        nodePort = std::atoi(argv[2]);
        char miningID[61];
        printf("Qiner is launched. Connecting to %s:%d\n", nodeIp, nodePort);

        consoleCtrlHandler();

        {
            // Data for signing the solution
            char* seed = argv[3];
            unsigned char privateKey[32];
            unsigned char subseed[32];
            char privateKeyQubicFormat[128] = {0};
            char publicKeyQubicFormat[128] = {0};
            char publicIdentity[128] = {0};
            getSubseedFromSeed((unsigned char*)seed, subseed);
            getPrivateKeyFromSubSeed(subseed, privateKey);
            getPublicKeyFromPrivateKey(privateKey, computorPublicKey);
            getIdentityFromPublicKey(computorPublicKey, miningID, false);

            hexToByte(argv[4], randomSeed, 32);
            unsigned int numberOfThreads = atoi(argv[5]);
            printf("%d threads are used.\n", numberOfThreads);
            miningThreads.resize(numberOfThreads);
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

                        // Source and destination is the same
                        memcpy(packet.message.sourcePublicKey, computorPublicKey, sizeof(packet.message.sourcePublicKey));
                        memcpy(packet.message.destinationPublicKey, computorPublicKey, sizeof(packet.message.destinationPublicKey));

                        unsigned char sharedKeyAndGammingNonce[64];
                        // First 32 bytes is the shared key
                        getSharedKey(privateKey, computorPublicKey, sharedKeyAndGammingNonce);
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
                        sign(subseed, computorPublicKey, digest, signature);
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