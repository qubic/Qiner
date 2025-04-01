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

static void hexToByte(const char* hex, uint8_t* byte, const int sizeInByte)
{
    for (int i = 0; i < sizeInByte; i++){
        sscanf(hex+i*2, "%2hhx", &byte[i]);
    }
}

constexpr int MESSAGE_TYPE_CUSTOM_MINING_SOLUTION = 2;
constexpr int BROADCAST_COMPUTORS = 2;
constexpr int NUMBER_OF_COMPUTORS = 676;
constexpr int SIGNATURE_SIZE = 64;

class CustomSolution
{
public:

    unsigned long long _taskIndex;
    unsigned int nonce;         // xmrig::JobResult.nonce
    unsigned int padding;
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

typedef struct
{
    // TODO: Padding
    unsigned short epoch;
    unsigned char publicKeys[NUMBER_OF_COMPUTORS][32];
    unsigned char signature[SIGNATURE_SIZE];
} Computors;

struct BroadcastComputors
{
    Computors computors;

    static constexpr unsigned char type()
    {
        return BROADCAST_COMPUTORS;
    }
};


int main(int argc, char* argv[])
{
    std::vector<std::thread> miningThreads;
    if (argc < 5)
    {
        printf("Usage:   Qiner [Node IP] [Node Port] [Seed] [Computors File]  \n");
    }
    else
    {
        nodeIp = argv[1];
        nodePort = std::atoi(argv[2]);
        char* miningID = argv[3];
        char* compFileName = argv[4];

        BroadcastComputors bc;
        {
            FILE* f = fopen(compFileName, "rb");
            if (fread(&bc, 1, sizeof(BroadcastComputors), f) != sizeof(BroadcastComputors))
            {
                printf("Failed to read comp list\n");
                fclose(f);
                return 1;
            }
            fclose(f);
        }

        consoleCtrlHandler();

        {
            // Data for signing the solution
            char* signingSeed = argv[3];
            unsigned char signingPrivateKey[32];
            unsigned char signingSubseed[32];
            unsigned char signingPublicKey[32];
            char privateKeyQubicFormat[128] = {0};
            char publicKeyQubicFormat[128] = {0};
            char publicIdentity[128] = {0};
            getSubseedFromSeed((unsigned char*)signingSeed, signingSubseed);
            getPrivateKeyFromSubSeed(signingSubseed, signingPrivateKey);
            getPublicKeyFromPrivateKey(signingPrivateKey, signingPublicKey);
            getIdentityFromPublicKey(computorPublicKey, publicIdentity, false);

            // Look for computorIdx
            int computorIdx = -1;
            // Find the computor index
            for (int k = 0; k < NUMBER_OF_COMPUTORS; k++)
            {
                if (memcmp(signingPublicKey, bc.computors.publicKeys[k], 32) == 0)
                {
                    computorIdx = k;
                }
            }
            if (computorIdx < 0)
            {
                printf("Can not find the computor index! PubID: %s\n", publicIdentity);
                return 1;
            }
            else
            {
                printf("Detect computor index %d\n", computorIdx);
            }

            CustomSolution solution;

            // Adjust the nonce with computor index
            _rdrand32_step(&solution.nonce);
            solution.nonce = solution.nonce % 1024;
            solution.nonce = solution.nonce * 676 + computorIdx;

            CustomMiningSolutionMessage solutionMessage;

            // Header
            solutionMessage._header.setSize(solutionMessage.getTotalSizeInBytes());
            solutionMessage._header.zeroDejavu();
            solutionMessage._header.setType(BROADCAST_MESSAGE);

            memcpy(solutionMessage._sourcePublicKey, signingPublicKey, sizeof(solutionMessage._sourcePublicKey));

            // Zero destination is used for custom mining
            memset(solutionMessage._destinationPublicKey, 0, sizeof(solutionMessage._destinationPublicKey));

            // Payload
            memcpy(&solutionMessage._solution, &solution, sizeof(solutionMessage._solution));

            unsigned char sharedKeyAndGammingNonce[64];
            // Default behavior when provided seed is just a signing address
            // first 32 bytes of sharedKeyAndGammingNonce is set as zeros
            memset(sharedKeyAndGammingNonce, 0, 32);

            // Last 32 bytes of sharedKeyAndGammingNonce is randomly created so that gammingKey[0] = 0 (MESSAGE_TYPE_SOLUTION)
            unsigned char gammingKey[32];
            do
            {
                _rdrand64_step((unsigned long long*) & solutionMessage._gammingNonce[0]);
                _rdrand64_step((unsigned long long*) & solutionMessage._gammingNonce[8]);
                _rdrand64_step((unsigned long long*) & solutionMessage._gammingNonce[16]);
                _rdrand64_step((unsigned long long*) & solutionMessage._gammingNonce[24]);

                memcpy(&sharedKeyAndGammingNonce[32], solutionMessage._gammingNonce, 32);
                KangarooTwelve(sharedKeyAndGammingNonce, 64, gammingKey, 32);
            } while (gammingKey[0] != MESSAGE_TYPE_CUSTOM_MINING_SOLUTION);

            // Sign the message
            uint8_t digest[32] = {0};
            uint8_t signature[64] = {0};
            KangarooTwelve(
                (unsigned char*)&solutionMessage + sizeof(RequestResponseHeader),
                solutionMessage.getTotalSizeInBytes() - sizeof(RequestResponseHeader) - 64,
                digest,
                32);
            sign(signingSubseed, signingPublicKey, digest, signature);
            memcpy(solutionMessage._signature, signature, 64);


            // Send data
            ServerSocket serverSocket;
            if (serverSocket.establishConnection(nodeIp))
            {
                // Send message
                if (!serverSocket.sendData((char*)&solutionMessage, solutionMessage._header.size()))
                {
                    printf("Failed to send data.\n");
                }
                serverSocket.closeConnection();
            }
            else
            {
                printf("Failed to establishConnection %s:%d.\n", nodeIp, nodePort);
            }
        }
    }

    return 0;
}