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

int main(int argc, char* argv[])
{
    std::vector<std::thread> miningThreads;
    if (argc != 7)
    {
        printf("Usage:   Qiner [Node IP] [Node Port] [MiningID] [Signing Seed] [Mining Seed] [Nonces]\n");
    }
    else
    {
        nodeIp = argv[1];
        nodePort = std::atoi(argv[2]);
        char* miningID = argv[3];

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

            unsigned char nonce[32];
            hexToByte(argv[5], randomSeed, 32);
            hexToByte(argv[6], nonce, 32);

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

            unsigned char gamma[32 + 32];
            KangarooTwelve(gammingKey, sizeof(gammingKey), gamma, sizeof(gamma));
            for (unsigned int i = 0; i < 32; i++)
            {
                packet.solutionMiningSeed[i] = randomSeed[i] ^ gamma[i];
                packet.solutionNonce[i] = nonce[i] ^ gamma[i + 32];
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

            // Send data
            ServerSocket serverSocket;
            if (serverSocket.establishConnection(nodeIp))
            {
                // Send message
                if (!serverSocket.sendData((char*)&packet, packet.header.size()))
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