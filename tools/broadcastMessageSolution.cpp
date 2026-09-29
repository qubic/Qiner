// Usage:
//   broadcastMessageSolution <Node IP> <Node Port> <MiningID> <Signing Seed> <Mining Seed hex> [score=-1] [count=1] [intervalMs=0]
//     score      : 0..8088 encodes an exact error (good if below the node threshold, bad if above);
//                  < 0 (default) uses a random score per solution.
//     count      : number of solutions to send (default 1).
//     intervalMs : delay between sends (default 0).

#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

#ifdef _MSC_VER
#include <intrin.h>
#include <winsock2.h>
#else
#include <immintrin.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#endif

#include "keyUtils.h"
#include "K12AndKeyUtil.h"
#include "score_bpp9000.h"


char* nodeIp = NULL;
int nodePort = 0;

#define BROADCAST_MESSAGE 1

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

    inline void zeroDejavu()
    {
        _dejavu = 0;
    }

    inline void setType(const unsigned char type)
    {
        _type = type;
    }
};

typedef struct
{
    unsigned char sourcePublicKey[32];
    unsigned char destinationPublicKey[32];
    unsigned char gammingNonce[32];
} Message;

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
};

static void hexToByte(const char* hex, uint8_t* byte, const int sizeInByte)
{
    for (int i = 0; i < sizeInByte; i++)
    {
        sscanf(hex + i * 2, "%2hhx", &byte[i]);
    }
}

static void byteToHex(const unsigned char* bytes, int sizeInByte, char* out)
{
    static const char* h = "0123456789abcdef";
    for (int i = 0; i < sizeInByte; i++)
    {
        out[2 * i] = h[bytes[i] >> 4];
        out[2 * i + 1] = h[bytes[i] & 0xF];
    }
    out[2 * sizeInByte] = 0;
}

// Build, encrypt, sign and broadcast one solution message (miningSeed + nonce + score).
static bool broadcastSolution(
    const unsigned char* miningSeed,
    const unsigned char* nonce,
    unsigned int score,
    const unsigned char* signingSubseed,
    const unsigned char* signingPrivateKey,
    const unsigned char* signingPublicKey,
    const unsigned char* computorPublicKey)
{
    ServerSocket serverSocket;
    if (!serverSocket.establishConnection(nodeIp))
    {
        return false;
    }

    struct
    {
        RequestResponseHeader header;
        Message message;
        unsigned char solutionMiningSeed[32];
        unsigned char solutionNonce[32];
        unsigned int solutionScore;
        unsigned char signature[64];
    } packet;

    packet.header.setSize(sizeof(packet));
    packet.header.zeroDejavu();
    packet.header.setType(BROADCAST_MESSAGE);

    memcpy(packet.message.sourcePublicKey, signingPublicKey, sizeof(packet.message.sourcePublicKey));
    memcpy(packet.message.destinationPublicKey, computorPublicKey, sizeof(packet.message.destinationPublicKey));

    unsigned char sharedKeyAndGammingNonce[64];
    // Default behavior when provided seed is just a signing address: first 32 bytes are zeros.
    memset(sharedKeyAndGammingNonce, 0, 32);
    // If the signing seed is the computor itself, derive the shared key to encrypt the message.
    if (memcmp(computorPublicKey, signingPublicKey, 32) == 0)
    {
        getSharedKey(signingPrivateKey, computorPublicKey, sharedKeyAndGammingNonce);
    }
    // Last 32 bytes are random so that gammingKey[0] = 0 (MESSAGE_TYPE_SOLUTION).
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

    // Encrypt the message payload: mining seed (32) + nonce (32) + score (4).
    unsigned char gamma[32 + 32 + 4];
    KangarooTwelve(gammingKey, sizeof(gammingKey), gamma, sizeof(gamma));
    for (unsigned int i = 0; i < 32; i++)
    {
        packet.solutionMiningSeed[i] = miningSeed[i] ^ gamma[i];
        packet.solutionNonce[i] = nonce[i] ^ gamma[i + 32];
    }
    for (unsigned int i = 0; i < 4; i++)
    {
        ((unsigned char*)&packet.solutionScore)[i] = ((const unsigned char*)&score)[i] ^ gamma[64 + i];
    }

    // Sign the message.
    uint8_t digest[32] = {0};
    uint8_t signature[64] = {0};
    KangarooTwelve(
        (unsigned char*)&packet + sizeof(RequestResponseHeader),
        sizeof(packet) - sizeof(RequestResponseHeader) - 64,
        digest,
        32);
    sign(signingSubseed, signingPublicKey, digest, signature);
    memcpy(packet.signature, signature, 64);

    bool sent = serverSocket.sendData((char*)&packet, packet.header.size());
    serverSocket.closeConnection();
    return sent;
}

int main(int argc, char* argv[])
{
    if (argc < 6 || argc > 9)
    {
        printf("Usage:   broadcastMessageSolution [Node IP] [Node Port] [MiningID] [Signing Seed] [Mining Seed hex] [Score or Nonce hex (optional)] [Count (optional, default 1)] [Interval ms (optional, default 0)]\n");
        printf("         Score or Nonce hex: a 64-char hex nonce is sent verbatim and REUSED for every send (send it twice to test dedup); a small number is the score with a fresh random nonce; omitted = random score.\n");
        return 1;
    }

    nodeIp = argv[1];
    nodePort = std::atoi(argv[2]);
    char* miningID = argv[3];
    char* signingSeed = argv[4];

    unsigned char miningSeed[32];
    hexToByte(argv[5], miningSeed, 32);

    // argv[6] is either a full 64-hex nonce (used verbatim and reused every send) or a score (random nonce).
    int score = -1;
    bool hasFixedNonce = false;
    unsigned char fixedNonce[32];
    if (argc >= 7)
    {
        if (strlen(argv[6]) == 64)
        {
            hexToByte(argv[6], fixedNonce, 32);
            hasFixedNonce = true;
        }
        else
        {
            score = std::atoi(argv[6]);
        }
    }
    int count = (argc >= 8) ? std::atoi(argv[7]) : 1;
    const int intervalMs = (argc >= 9) ? std::atoi(argv[8]) : 0;
    if (count < 1)
    {
        count = 1;
    }

    // Derive the signing keys and the destination computor key.
    unsigned char computorPublicKey[32];
    unsigned char signingSubseed[32];
    unsigned char signingPrivateKey[32];
    unsigned char signingPublicKey[32];
    getPublicKeyFromIdentity(miningID, computorPublicKey);
    getSubseedFromSeed((unsigned char*)signingSeed, signingSubseed);
    getPrivateKeyFromSubSeed(signingSubseed, signingPrivateKey);
    getPublicKeyFromPrivateKey(signingPrivateKey, signingPublicKey);

    printf("broadcastMessageSolution -> %s:%d, %s, count %d, interval %d ms\n",
           nodeIp, nodePort,
           hasFixedNonce ? "fixed nonce" : (score >= 0 ? "fixed score" : "random score"),
           count, intervalMs);

    unsigned char nonce[32];
    unsigned int target = 0;

    unsigned int sent = 0;
    for (int c = 0; c < count; c++)
    {
        if (hasFixedNonce)
        {
            // Send the given nonce verbatim - identical every iteration, so the node dedups all but the
            // first. The score must match what the node decodes from nonce[30..31].
            memcpy(nonce, fixedNonce, 32);
            target = ((unsigned int)nonce[30] | ((unsigned int)nonce[31] << 8)) % (score_bpp9000::NUMBER_OF_WINDOWS + 1);
        }
        else
        {
            // Fresh random nonce; fix the algo id and encode the target score into nonce[30..31].
            _rdrand64_step((unsigned long long*)&nonce[0]);
            _rdrand64_step((unsigned long long*)&nonce[8]);
            _rdrand64_step((unsigned long long*)&nonce[16]);
            _rdrand64_step((unsigned long long*)&nonce[24]);
            // Canonical nonce: nonce[0]=algo, nonce[1]=L in [1,10] (bits 0-3) + mode in [1,3] (bits 4-5),
            // nonce[2]=K=0; else core scores it INVALID once the canonical activation tick hits.
            nonce[0] = (unsigned char)AlgoType::Bpp9000;
            const unsigned char L = (unsigned char)((nonce[1] % score_bpp9000::MAX_CHANGES_PER_STEP) + 1);
            const unsigned char mode = (unsigned char)(((nonce[1] >> 4) % 3) + 1);
            nonce[1] = (unsigned char)(L | (mode << 4));
            // K: canonical 0
            nonce[2] = 0;

            if (score >= 0)
            {
                target = (unsigned int)score % (score_bpp9000::NUMBER_OF_WINDOWS + 1);
            }
            else
            {
                unsigned int r = 0;
                _rdrand32_step(&r);
                target = r % (score_bpp9000::NUMBER_OF_WINDOWS + 1);
            }
            nonce[30] = (unsigned char)(target & 0xFF);
            nonce[31] = (unsigned char)((target >> 8) & 0xFF);
        }

        char nonceHex[65];
        byteToHex(nonce, 32, nonceHex);
        if (broadcastSolution(miningSeed, nonce, target, signingSubseed, signingPrivateKey, signingPublicKey, computorPublicKey))
        {
            sent++;
            printf("[%d/%d] sent solution: score %u, nonce %s\n", c + 1, count, target, nonceHex);
        }
        else
        {
            printf("[%d/%d] failed to send solution: score %u, nonce %s\n", c + 1, count, target, nonceHex);
        }

        if (intervalMs > 0 && c + 1 < count)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        }
    }

    printf("Done. %u/%d solutions sent.\n", sent, count);
    return 0;
}
