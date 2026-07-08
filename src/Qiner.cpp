#include <chrono>
#include <thread>
#include <mutex>
#include <cstdio>
#include <cstring>
#include <array>
#include <queue>
#include <atomic>
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

#include "score_hyperidentity.h"
#include "score_generic_lut.h"
#include "keyUtils.h"

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
// Epoch-start Spectrum Digest selects the Addition training subset; empty (all zero) when none is provided.
static unsigned char epochStartSpectrumDigest[32];
// Path to the Generic LUT task file (input/output trit data); overridable via CLI.
static const char* taskFilePath = "task_lut.bin";
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

struct Stat
{
    std::atomic<unsigned long long> totalAdditionNonce;
    std::atomic<unsigned long long> totalHyperIdentityNonce;
    std::atomic<unsigned long long> totalHyperIdentitySols;
    std::atomic<unsigned long long> totalAdditionSols;

    Stat()
    {
        totalAdditionNonce.store(0);
        totalHyperIdentityNonce.store(0);
        totalHyperIdentitySols.store(0);
        totalAdditionSols.store(0);
    }

} qinerStat;

using LutMiner = score_generic_lut::Miner<
    score_generic_lut::NUMBER_OF_INPUT_NEURONS,
    score_generic_lut::NUMBER_OF_OUTPUT_NEURONS,
    score_generic_lut::SEQUENCE_LENGTH,
    score_generic_lut::WINDOW_WIDTH,
    score_generic_lut::MAX_NUMBER_OF_TICKS,
    score_generic_lut::NUMBER_OF_NEIGHBORS,
    score_generic_lut::POPULATION_THRESHOLD,
    score_generic_lut::NUMBER_OF_MUTATIONS,
    score_generic_lut::SOLUTION_THRESHOLD>;
using HyperIdentityMiner = score_hyberidentity::Miner<
    score_hyberidentity::NUMBER_OF_INPUT_NEURONS,
    score_hyberidentity::NUMBER_OF_OUTPUT_NEURONS,
    score_hyberidentity::NUMBER_OF_TICKS,
    score_hyberidentity::MAX_NEIGHBOR_NEURONS,
    score_hyberidentity::POPULATION_THRESHOLD,
    score_hyberidentity::NUMBER_OF_MUTATIONS,
    score_hyberidentity::SOLUTION_THRESHOLD>;

int miningThreadProc()
{
    std::unique_ptr<LutMiner> lutMiner(new LutMiner());
    // Generic LUT needs the epoch-start Spectrum Digest (placement/wiring) and the task file (data).
    if (!lutMiner->initialize(randomSeed, epochStartSpectrumDigest, taskFilePath))
    {
        printf("Failed to load task file '%s' for the Generic LUT miner.\n", taskFilePath);
        return -1;
    }

    std::unique_ptr<HyperIdentityMiner> hyperIdentityMiner(new HyperIdentityMiner());
    hyperIdentityMiner->initialize(randomSeed);

    std::array<unsigned char, 32> nonce;
    while (!state)
    {
        _rdrand64_step((unsigned long long*)&nonce.data()[0]);
        _rdrand64_step((unsigned long long*)&nonce.data()[8]);
        _rdrand64_step((unsigned long long*)&nonce.data()[16]);
        _rdrand64_step((unsigned long long*)&nonce.data()[24]);

        bool solutionFound = false;

        // First byte of nonce is used for determine type of score
        if ((nonce[0] & 1) == 0)
        {
            solutionFound = hyperIdentityMiner->findSolution(computorPublicKey, nonce.data());
            // Stats
            qinerStat.totalHyperIdentityNonce.fetch_add(1);
            if (solutionFound)
            {
                qinerStat.totalHyperIdentitySols.fetch_add(1);
            }
        }
        else
        {
            solutionFound = lutMiner->findSolution(computorPublicKey, nonce.data());
            // Stats
            qinerStat.totalAdditionNonce.fetch_add(1);
            if (solutionFound)
            {
                qinerStat.totalAdditionSols.fetch_add(1);
            }
        }

        if (solutionFound)
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
    if (argc != 7 && argc != 8 && argc != 9)
    {
        printf("Usage:   Qiner [Node IP] [Node Port] [MiningID] [Signing Seed] [Mining Seed] [Number of threads] [Epoch-start Spectrum Digest (optional)] [Task file path (optional)]\n");
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

            // Epoch-start Spectrum Digest is optional (argv[7]); without it the Addition training subset uses an empty digest.
            if (argc >= 8)
            {
                hexToByte(argv[7], epochStartSpectrumDigest, 32);
            }
            else
            {
                memset(epochStartSpectrumDigest, 0, sizeof(epochStartSpectrumDigest));
                printf("WARNING: no Epoch-start Spectrum Digest provided, using an empty one for the Addition training subset.\n");
            }

            // Task file path is optional (argv[8]); defaults to task_lut.bin.
            if (argc >= 9)
            {
                taskFilePath = argv[8];
            }

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

        // Print stats
        printf("Hyperidentity sols / nonces: %llu / %llu \n", qinerStat.totalHyperIdentitySols.load(), qinerStat.totalHyperIdentityNonce.load());
        printf("Addition sols / nonces: %llu / %llu \n", qinerStat.totalAdditionSols.load(), qinerStat.totalAdditionNonce.load());

        printf("Qiner is shut down.\n");
    }

    return 0;
}