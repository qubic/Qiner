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
#include "connection.h"
#include <iostream>
#include <random>
#include <algorithm>
#include <csignal>
#include <sstream>

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

#define BROADCAST_MESSAGE 1

typedef struct
{
    unsigned char sourcePublicKey[32];
    unsigned char destinationPublicKey[32];
    unsigned char gammingNonce[32];
} Message;

char* nodeIp = NULL;
int nodePort = 0;

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


constexpr int MESSAGE_TYPE_CUSTOM_MINING_TASK = 1;
constexpr int MESSAGE_TYPE_CUSTOM_MINING_SOLUTION = 2;

constexpr uint16_t NUMBER_OF_TASK_PARTITIONS = 4;
struct
{
    uint16_t firstComputorIdx;
    uint16_t lastComputorIdx;
    uint32_t domainSize;
} gTaskPartition[NUMBER_OF_TASK_PARTITIONS];
uint16_t gComputorPartitionMap[676];

struct CustomMiningTask
{
    unsigned long long taskIndex; // ever increasing number (unix timestamp in ms)
    unsigned short firstComputorIndex, lastComputorIndex;
    unsigned int padding;

    unsigned char blob[408]; // Job data from pool
    unsigned long long size;  // length of the blob
    unsigned long long target; // Pool difficulty
    unsigned long long height; // Block height
    unsigned char seed[32]; // Seed hash for XMR
};

class CustomMiningTaskMessage
{

public:
CustomMiningTaskMessage() = default;

    RequestResponseHeader _header;

    unsigned char _sourcePublicKey[32];
    unsigned char _destinationPublicKey[32];
    unsigned char _gammingNonce[32];
    CustomMiningTask _task;

    unsigned char _signature[64];

    size_t serialize(char* buffer) const
    {
        memcpy(buffer, this, getTotalSizeInBytes());
        return getTotalSizeInBytes();
    }

    size_t getTotalSizeInBytes() const
    {
        return sizeof(CustomMiningTaskMessage);
    }
    size_t getPayLoadSize() const
    {
        return sizeof(CustomMiningTaskMessage) - sizeof(_header) - sizeof(_signature);
    }
};

class CustomSolution
{
public:

    unsigned long long _taskIndex;
    unsigned short firstComputorIndex, lastComputorIndex;
    unsigned int nonce;         // xmrig::JobResult.nonce
    unsigned char result[32];   // xmrig::JobResult.result
};

class CustomMiningSolutionMessage
{

public:
    CustomMiningSolutionMessage() = default;

    RequestResponseHeader _header;

    unsigned char _sourcePublicKey[32];
    unsigned char _destinationPublicKey[32];
    unsigned char _gammingNonce[32];
    CustomSolution _solution;

    unsigned char _signature[64];

    size_t serialize(char* buffer) const
    {
        memcpy(buffer, this, getTotalSizeInBytes());
        return getTotalSizeInBytes();
    }

    size_t getTotalSizeInBytes() const
    {
        return sizeof(CustomMiningSolutionMessage);
    }
    size_t getPayLoadSize() const
    {
        return sizeof(CustomMiningSolutionMessage) - sizeof(_header) - sizeof(_signature);
    }
};

int craftTaskMessage(
    const unsigned char* signingSubseed,
    const unsigned char* signingPublicKey,
    uint32_t partID,
    CustomMiningTaskMessage& taskMessage)
{
    // Header
    taskMessage._header.setSize(taskMessage.getTotalSizeInBytes());
    taskMessage._header.zeroDejavu();
    taskMessage._header.setType(BROADCAST_MESSAGE);

    memcpy(taskMessage._sourcePublicKey, signingPublicKey, sizeof(taskMessage._sourcePublicKey));

    // Zero destination is used for custom mining
    memset(taskMessage._destinationPublicKey, 0, sizeof(taskMessage._destinationPublicKey));

    // Payload of a dummy task
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    taskMessage._task.taskIndex = (uint64_t)milliseconds;
    taskMessage._task.size = 76;
    taskMessage._task.target = 38427114278264;
    taskMessage._task.height = 3401164;

    // Random the first and last computor index
    taskMessage._task.firstComputorIndex = gTaskPartition[partID].firstComputorIdx;
    taskMessage._task.lastComputorIndex = gTaskPartition[partID].lastComputorIdx;

    // Gamming nonce
    unsigned char sharedKeyAndGammingNonce[64];
    // Default behavior when provided seed is just a signing address
    // first 32 bytes of sharedKeyAndGammingNonce is set as zeros
    memset(sharedKeyAndGammingNonce, 0, 32);

    // Last 32 bytes of sharedKeyAndGammingNonce is randomly created so that gammingKey[0] = 0 (MESSAGE_TYPE_CUSTOM_MINING_TASK)
    unsigned char gammingKey[32];
    do
    {
        _rdrand64_step((unsigned long long*) & taskMessage._gammingNonce[0]);
        _rdrand64_step((unsigned long long*) & taskMessage._gammingNonce[8]);
        _rdrand64_step((unsigned long long*) & taskMessage._gammingNonce[16]);
        _rdrand64_step((unsigned long long*) & taskMessage._gammingNonce[24]);

        memcpy(&sharedKeyAndGammingNonce[32], taskMessage._gammingNonce, 32);
        KangarooTwelve(sharedKeyAndGammingNonce, 64, gammingKey, 32);
    } while (gammingKey[0] != MESSAGE_TYPE_CUSTOM_MINING_TASK);


    // Sign the message
    uint8_t digest[32] = {0};
    uint8_t signature[64] = {0};
    KangarooTwelve(
        (unsigned char*)&taskMessage + sizeof(RequestResponseHeader),
        taskMessage.getTotalSizeInBytes() - sizeof(RequestResponseHeader) - SIGNATURE_SIZE,
        digest,
        32);
    sign(signingSubseed, signingPublicKey, digest, signature);
    memcpy(taskMessage._signature, signature, 64);

    return 0;
}

int craftSolutionMessage(
    const unsigned char* signingSubseed,
    const unsigned char* signingPublicKey,
    const CustomMiningTask& task,
    const unsigned long long nonce,
    CustomMiningSolutionMessage& solMessage)
{
    // Header
    solMessage._header.setSize(solMessage.getTotalSizeInBytes());
    solMessage._header.zeroDejavu();
    solMessage._header.setType(BROADCAST_MESSAGE);

    memcpy(solMessage._sourcePublicKey, signingPublicKey, sizeof(solMessage._sourcePublicKey));

    // Zero destination is used for custom mining
    memset(solMessage._destinationPublicKey, 0, sizeof(solMessage._destinationPublicKey));

    // Payload of a dummy solution
    solMessage._solution._taskIndex = task.taskIndex;
    solMessage._solution.firstComputorIndex = task.firstComputorIndex;
    solMessage._solution.lastComputorIndex = task.lastComputorIndex;
    solMessage._solution.nonce = nonce;

    // Gamming nonce
    unsigned char sharedKeyAndGammingNonce[64];
    // Default behavior when provided seed is just a signing address
    // first 32 bytes of sharedKeyAndGammingNonce is set as zeros
    memset(sharedKeyAndGammingNonce, 0, 32);

    // Last 32 bytes of sharedKeyAndGammingNonce is randomly created so that gammingKey[0] = 0 (MESSAGE_TYPE_CUSTOM_MINING_TASK)
    unsigned char gammingKey[32];
    do
    {
        _rdrand64_step((unsigned long long*) & solMessage._gammingNonce[0]);
        _rdrand64_step((unsigned long long*) & solMessage._gammingNonce[8]);
        _rdrand64_step((unsigned long long*) & solMessage._gammingNonce[16]);
        _rdrand64_step((unsigned long long*) & solMessage._gammingNonce[24]);

        memcpy(&sharedKeyAndGammingNonce[32], solMessage._gammingNonce, 32);
        KangarooTwelve(sharedKeyAndGammingNonce, 64, gammingKey, 32);
    } while (gammingKey[0] != MESSAGE_TYPE_CUSTOM_MINING_SOLUTION);


    // Sign the message
    uint8_t digest[32] = {0};
    uint8_t signature[64] = {0};
    KangarooTwelve(
        (unsigned char*)&solMessage + sizeof(RequestResponseHeader),
        solMessage.getTotalSizeInBytes() - sizeof(RequestResponseHeader) - SIGNATURE_SIZE,
        digest,
        32);
    sign(signingSubseed, signingPublicKey, digest, signature);
    memcpy(solMessage._signature, signature, 64);

    return 0;
}

std::queue<CustomMiningTask> taskVectors[676];
std::queue<CustomMiningSolutionMessage> solsQueue;
std::mutex taskLockVectors[676];
std::mutex solLock;

int randomPartId[] = {0, 1, 2, 3};
void runDispatcher(char* ip, int port, char* signingSeed, int numberOfComputors)
{
    // Set up a random number generator
    std::random_device rd;
    std::mt19937 g(rd());

    unsigned char signingPrivateKey[32];
    unsigned char signingSubseed[32];
    unsigned char signingPublicKey[32];
    char privateKeyQubicFormat[128] = {0};
    char publicKeyQubicFormat[128] = {0};
    char publicIdentity[128] = {0};
    getSubseedFromSeed((unsigned char*)signingSeed, signingSubseed);
    getPrivateKeyFromSubSeed(signingSubseed, signingPrivateKey);
    getPublicKeyFromPrivateKey(signingPrivateKey, signingPublicKey);
    getIdentityFromPublicKey(signingPublicKey, publicIdentity, false);

    printf("Start DISPATCHER ID %s \n", publicIdentity);

    CustomMiningTaskMessage taskMessage;
    while (0 == state.load())
    {
        // Spawn part ID
        // Shuffle the array
        std::shuffle(std::begin(randomPartId), std::end(randomPartId), g);

        for (int i = 0; i < NUMBER_OF_TASK_PARTITIONS; i++)
        {
            craftTaskMessage(signingSubseed, signingPublicKey, randomPartId[i], taskMessage);

            // Send the data
            ServerSocket serverSocket;
            if (serverSocket.establishConnection(nodeIp))
            {
                // Send message
                if (!serverSocket.sendData((char*)&taskMessage, taskMessage._header.size()))
                {
                    printf("Failed to send data.\n");
                }
                serverSocket.closeConnection();
                printf("Sent task %llu, firstIdx %d, lastIdx %d \n", taskMessage._task.taskIndex, taskMessage._task.firstComputorIndex, taskMessage._task.lastComputorIndex);
            }
            else
            {
                printf("Failed to establishConnection %s:%d.\n", nodeIp, nodePort);
            }

            for (int k = 0; k < numberOfComputors; k++)
            {
                std::lock_guard<std::mutex> lock(taskLockVectors[k]);
                taskVectors[k].push(taskMessage._task);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    }
}

std::atomic<int> nPeer(0);
CustomMiningTaskMessage currentTaskMessage;
std::mutex taskLock;

void listenerThread(const char* nodeIp, int port, unsigned char* dispatcherPublicKey)
{
    printf("Start listerner ID %s:%d \n", nodeIp, port);
    long long prevTs = 0;
    QCPtr qc;
    bool needReconnect = true;
    std::string log_header = "[" + std::string(nodeIp) + "]: ";
    while (0 == state.load())
    {
        try {
            if (needReconnect) {
                needReconnect = false;
                nPeer.fetch_add(1);
                qc = make_qc(nodeIp, port);
                qc->exchangePeer();// do the handshake stuff
                // TODO: connect to received peers
                printf("Connected to %s\n", nodeIp);
            }
            auto header = qc->receiveHeader();
            std::vector<uint8_t> buff;
            uint32_t sz = header.size();
            if (sz > 0xFFFFFF)
            {
                needReconnect = true;
                nPeer.fetch_add(-1);
                continue;
            }
            sz -= sizeof(RequestResponseHeader);
            buff.resize(sz);
            int revSize = qc->receiveData(buff.data(), sz);
            if (revSize == sz && header.type() == 1) // broadcast msg
            {
                if (buff.size() == sizeof(CustomMiningTaskMessage) - sizeof(RequestResponseHeader))
                {
                    CustomMiningTaskMessage tk;
                    memcpy((char*)&tk + sizeof(RequestResponseHeader),buff.data(), buff.size());
                    if (memcmp(dispatcherPublicKey, tk._sourcePublicKey, 32) != 0)
                    {
                        printf("Job not from dispatcher\n");
                        continue;
                    }
                    uint8_t sharedKeyAndGammingNonce[64];
                    memset(sharedKeyAndGammingNonce, 0, 32);
                    memcpy(&sharedKeyAndGammingNonce[32], tk._gammingNonce, 32);
                    uint8_t gammingKey[32];
                    KangarooTwelve(sharedKeyAndGammingNonce, 64, gammingKey, 32);
                    if (gammingKey[0] != 1)
                    {
                        printf("Wrong type from dispatcher\n");
                        continue;
                    }
                    uint8_t digest[32];
                    KangarooTwelve(buff.data(), buff.size() - 64, digest, 32);
                    if (!verify(dispatcherPublicKey, digest, buff.data() + buff.size() - 64))
                    {
                        printf("Wrong sig from dispatcher\n");
                        continue;
                    }
                    {
                        std::lock_guard<std::mutex> glock(taskLock);
                        if (currentTaskMessage._task.taskIndex <  tk._task.taskIndex)
                        {
                            currentTaskMessage = tk;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    uint64_t delta = 0;

                    if (prevTs)
                    {
                        delta = (tk._task.taskIndex - prevTs);
                    }
                    prevTs = tk._task.taskIndex;
                    char dbg[256] = {0};
                    std::string debug_log = log_header;
                    sprintf(dbg, "Received task index %lu (d_prev: %lu ms)", tk._task.taskIndex, delta);
                    debug_log += std::string(dbg); memset(dbg, 0, sizeof(dbg));
                    printf("%s\n", debug_log.c_str());
                }

            }
            fflush(stdout);
        }
        catch (std::logic_error &ex) {
            printf("%s\n", ex.what());
            fflush(stdout);
            needReconnect = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        }
        catch (...)
        {
            printf("Unknown exception caught!\n");
            fflush(stdout);
            needReconnect = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
}

void solutionThread(const char* computorSeed, int noncePerTask, unsigned int constantNonce, int compIdx)
{
    unsigned char signingPrivateKey[32];
    unsigned char signingSubseed[32];
    unsigned char signingPublicKey[32];
    char privateKeyQubicFormat[128] = {0};
    char publicKeyQubicFormat[128] = {0};
    char publicIdentity[128] = {0};
    getSubseedFromSeed((unsigned char*)computorSeed, signingSubseed);
    getPrivateKeyFromSubSeed(signingSubseed, signingPrivateKey);
    getPublicKeyFromPrivateKey(signingPrivateKey, signingPublicKey);
    getIdentityFromPublicKey(signingPublicKey, publicIdentity, false);

    // Message send to node
    CustomMiningSolutionMessage solMessage;
    printf("Start solution thread for %s \n", publicIdentity);
    CustomMiningTask local_task;
    local_task.taskIndex = 0;
    while (0 == state.load())
    {
        bool hasNewTask = false;
        {
            std::lock_guard<std::mutex> glock(taskLockVectors[compIdx]);
            hasNewTask = !taskVectors[compIdx].empty();
            if (hasNewTask)
            {
                local_task = taskVectors[compIdx].front();
                taskVectors[compIdx].pop();
            }
        }

        if (!hasNewTask)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            continue;
        }

        // Generate a specific number of solution and submit to the node
        int submittedCount = 0;
        while (submittedCount < noncePerTask)
        {
            unsigned int nonce = 0;
            if (constantNonce != 0xFFFFFFFF)
            {
                nonce = constantNonce;
            }
            else
            {
                _rdrand32_step((unsigned int*) &nonce);
            }
            craftSolutionMessage(signingSubseed, signingPublicKey, local_task, nonce, solMessage);
            {
                std::lock_guard<std::mutex> lock(solLock);
                solsQueue.push(solMessage);
            }
            submittedCount++;
        }
    }
}

void submitionThread(const char* nodeIp, int port)
{
    QCPtr qc;
    bool needReconnect = true;
    CustomMiningSolutionMessage solMessage;
    while (0 == state.load())
    {
        try {
            if (needReconnect)
            {
                needReconnect = false;
                qc = make_qc(nodeIp, port);
            }
            bool hasSols = false;
            {
                std::lock_guard<std::mutex> lock(solLock);
                hasSols = !solsQueue.empty();
                if (hasSols)
                {
                    solMessage = solsQueue.front();
                    solsQueue.pop();
                }
            }

            if (!hasSols)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                continue;
            }

            int dataSend = qc->sendData((uint8_t*)&solMessage, sizeof(solMessage));
            if (dataSend == sizeof(solMessage))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            else
            {
                needReconnect = true;
            }
        }
        catch (std::logic_error &ex)
        {
            printf("Connection FAILED %s\n", ex.what());
            fflush(stdout);
            needReconnect = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        catch (...)
        {
            printf("Unknown exception caught!\n");
            fflush(stdout);
            needReconnect = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
}

void printHelp()
{
    std::cout << "Usage:\n";
    std::cout << "- Submit solution: broadcastMessageSolution [Node IP] [Node Port] [Dispatcher ID] [Seed0,Seed1,seed2...] [NoncePerTask] [DuplicatedSolutionNonce]\n";
}

std::vector<std::string> getListOfComputorSeeds(std::string input)
{
    std::vector<std::string> result;
    std::stringstream ss(input);
    std::string str;
    while (std::getline(ss, str, ','))
    {
        result.push_back(str);
    }
    return result;
}

int main(int argc, char* argv[])
{
    // Ignore SIGPIPE globally
    std::signal(SIGPIPE, SIG_IGN);

    // Generate computor groups
    for (int i = 0; i < NUMBER_OF_TASK_PARTITIONS; i++)
    {
        gTaskPartition[i].firstComputorIdx = i * NUMBER_OF_COMPUTORS / 4;
        gTaskPartition[i].lastComputorIdx = gTaskPartition[i].firstComputorIdx + NUMBER_OF_COMPUTORS / 4 - 1;
        gTaskPartition[i].domainSize =  0xFFFFFFFFU / (gTaskPartition[i].lastComputorIdx  - gTaskPartition[i].firstComputorIdx + 1);
    }

    std::vector<std::thread> miningThreads;
    if (argc < 4)
    {
        printHelp();
        return 0;
    }

    //std::string mode = argv[1];
    //if (mode == "dispatch")
    {
        consoleCtrlHandler();
        nodeIp = argv[1];
        nodePort = std::atoi(argv[2]);

        // Data for signing the solution
        char* signingSeed = argv[3];
        std::string listSeed = argv[4];
        std::vector<std::string> computorsSeed = getListOfComputorSeeds(listSeed);
        std::cout << argv[4];
        unsigned int noncePerTask = std::atoi(argv[5]);
        unsigned int constantSolutionNonce = 0xFFFFFFFFUL;
        if (argc == 7)
        {
            constantSolutionNonce = std::atoi(argv[6]);
            printf("Enable submit duplicated solution! Nonce = %u\n", constantSolutionNonce);
        }

        std::cout << "noncePerTask: " << noncePerTask << std::endl;
        std::cout << "IP:" << nodeIp << ":" << nodePort << std::endl;
        std::cout << "Dispatcher seed:" << signingSeed << std::endl;
        std::cout << "Computor seed:" << std::endl;
        for (const auto it : computorsSeed)
        {
            std::cout << it << std::endl;
        }
        //taskVectors.resize(computorsSeed.size());

        std::vector<std::thread> thr;

        // Run the dispatcher
        thr.push_back(std::thread(runDispatcher, nodeIp, nodePort, signingSeed, computorsSeed.size()));

        // Run the solution submitition

        for (int compI = 0; compI < computorsSeed.size(); ++compI)
        {
            thr.push_back(std::thread(solutionThread, computorsSeed[compI].c_str(), noncePerTask, constantSolutionNonce, compI));
        }

        thr.push_back(std::thread(submitionThread, nodeIp, nodePort));

        for (auto& t : thr)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }

    // else if (mode == "solution")
    // {
    //     nodeIp = argv[2];
    //     nodePort = std::atoi(argv[3]);
    //     char* dispatcherPubID = argv[4];
    //     char* computorSeed = argv[5];
    //     unsigned int noncePerTask = std::atoi(argv[6]);

    //     unsigned int constantSolutionNonce = 0xFFFFFFFFUL;
    //     if (argc == 8)
    //     {
    //         constantSolutionNonce = std::atoi(argv[7]);
    //         printf("Enable submit duplicated solution! Nonce = %u\n", constantSolutionNonce);
    //     }

    //     uint8_t dispatcherPubkey[32];
    //     getPublicKeyFromIdentity(dispatcherPubID, dispatcherPubkey);
    //     std::vector<std::thread> thr;

    //     // Fetch task from peers
    //     for (int i = 0; i < ipVec.size(); i++)
    //     {
    //         thr.push_back(std::thread(listenerThread, ipVec[i].c_str(), nodePort, dispatcherPubkey));
    //     }

    //     // Submit the task to node
    //     //const char* nodeIp, int port, unsigned char* computorSeed, unsigned int noncePerTask, unsigned int constantNonce
    //     thr.push_back(std::thread(solutionThread, nodeIp, nodePort, computorSeed, noncePerTask, constantSolutionNonce));

    //     for (auto& t : thr)
    //     {
    //         if (t.joinable())
    //         {
    //             t.join();
    //         }
    //     }
    // }
    // else
    // {
    //     printHelp();
    // }

    return 0;
}