// For connection to node
#pragma once

#include "K12AndKeyUtil.h"
#include "keyUtils.h"

#include <array>
#include <assert.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>

#ifdef _MSC_VER
#include <intrin.h>
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")

#else
#include <arpa/inet.h>
#include <immintrin.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#endif

#define BROADCAST_MESSAGE 1

struct RequestResponseHeader
{
private:
    unsigned char _size[3];
    unsigned char _type;
    unsigned int _dejavu;

public:
    inline unsigned int size() { return (*((unsigned int*)_size)) & 0xFFFFFF; }

    inline void setSize(unsigned int size)
    {
        _size[0] = (unsigned char)size;
        _size[1] = (unsigned char)(size >> 8);
        _size[2] = (unsigned char)(size >> 16);
    }

    inline bool isDejavuZero() const { return !_dejavu; }

    inline void zeroDejavu() { _dejavu = 0; }

    inline unsigned int dejavu() const { return _dejavu; }

    inline void setDejavu(unsigned int dejavu) { _dejavu = dejavu; }

    inline void randomizeDejavu()
    {
        _rdrand32_step(&_dejavu);
        if (!_dejavu)
        {
            _dejavu = 1;
        }
    }

    inline unsigned char type() const { return _type; }

    inline void setType(const unsigned char type) { _type = type; }
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

    ~ServerSocket() { WSACleanup(); }

    void closeConnection() { closesocket(serverSocket); }

    bool establishConnection(const char* address, int nodePort)
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
        sscanf_s(
            address,
            "%hhu.%hhu.%hhu.%hhu",
            &addr.sin_addr.S_un.S_un_b.s_b1,
            &addr.sin_addr.S_un.S_un_b.s_b2,
            &addr.sin_addr.S_un.S_un_b.s_b3,
            &addr.sin_addr.S_un.S_un_b.s_b4);
        if (connect(serverSocket, (const sockaddr*)&addr, sizeof(addr)))
        {
            printf(
                "Fail to connect to %d.%d.%d.%d (%d)!\n",
                addr.sin_addr.S_un.S_un_b.s_b1,
                addr.sin_addr.S_un.S_un_b.s_b2,
                addr.sin_addr.S_un.S_un_b.s_b3,
                addr.sin_addr.S_un.S_un_b.s_b4,
                WSAGetLastError());
            closeConnection();
            return false;
        }

        return true;
    }

    SOCKET serverSocket;
#else
    void closeConnection() { close(serverSocket); }
    bool establishConnection(const char* address, int nodePort)
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
            deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - beginningTime)
                            .count();
        }

        return true;
    }
};

struct SolutionSubmitter
{
    SolutionSubmitter(
        char* address,
        int nodePort,
        const unsigned char* miningSeed,
        const char* miningID,
        const char* signingSeed)
    {
        _nodePort = nodePort;
        _nodeIp = address;
        memcpy(_miningSeed, miningSeed, sizeof(_miningSeed));

        // Set data for submission
        getSubseedFromSeed((unsigned char*)signingSeed, _signingSubseed);
        getPrivateKeyFromSubSeed(_signingSubseed, _signingPrivateKey);
        getPublicKeyFromPrivateKey(_signingPrivateKey, _signingPublicKey);
        getPublicKeyFromIdentity(miningID, _computorPublicKey);
    }

    bool submit(const unsigned char* sendNonce)
    {
        bool sendSuccess = false;
        if (serverSocket.establishConnection(_nodeIp.c_str(), _nodePort))
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

            memcpy(
                packet.message.sourcePublicKey,
                _signingPublicKey,
                sizeof(packet.message.sourcePublicKey));
            memcpy(
                packet.message.destinationPublicKey,
                _computorPublicKey,
                sizeof(packet.message.destinationPublicKey));

            unsigned char sharedKeyAndGammingNonce[64];
            // Default behavior when provided seed is just a signing address
            // first 32 bytes of sharedKeyAndGammingNonce is set as zeros
            memset(sharedKeyAndGammingNonce, 0, 32);
            // If provided seed is the for computor public key, generate sharedKey into first 32
            // bytes to encrypt message
            if (memcmp(_computorPublicKey, _signingPublicKey, 32) == 0)
            {
                getSharedKey(_signingPrivateKey, _computorPublicKey, sharedKeyAndGammingNonce);
            }
            // Last 32 bytes of sharedKeyAndGammingNonce is randomly created so that gammingKey[0] =
            // 0 (MESSAGE_TYPE_SOLUTION)
            unsigned char gammingKey[32];
            do
            {
                _rdrand64_step((unsigned long long*)&packet.message.gammingNonce[0]);
                _rdrand64_step((unsigned long long*)&packet.message.gammingNonce[8]);
                _rdrand64_step((unsigned long long*)&packet.message.gammingNonce[16]);
                _rdrand64_step((unsigned long long*)&packet.message.gammingNonce[24]);
                memcpy(&sharedKeyAndGammingNonce[32], packet.message.gammingNonce, 32);
                KangarooTwelve(sharedKeyAndGammingNonce, 64, gammingKey, 32);
            } while (gammingKey[0]);

            // Encrypt the message payload
            unsigned char gamma[32 + 32];
            KangarooTwelve(gammingKey, sizeof(gammingKey), gamma, sizeof(gamma));
            for (unsigned int i = 0; i < 32; i++)
            {
                packet.solutionMiningSeed[i] = _miningSeed[i] ^ gamma[i];
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
            sign(_signingSubseed, _signingPublicKey, digest, signature);
            memcpy(packet.signature, signature, 64);

            // Send message
            if (serverSocket.sendData((char*)&packet, packet.header.size()))
            {
                sendSuccess = true;
            }
            serverSocket.closeConnection();
        }
        return sendSuccess;
    }

    std::string _nodeIp;
    int _nodePort;
    ServerSocket serverSocket;
    unsigned char _miningSeed[32];

    unsigned char _computorPublicKey[32];
    unsigned char _signingPrivateKey[32];
    unsigned char _signingSubseed[32];
    unsigned char _signingPublicKey[32];
};
