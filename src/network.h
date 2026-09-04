#pragma once

// Network common functions

#include <cstdio>
#include <cstring>

#ifdef _MSC_VER
#include <intrin.h>
#include <winsock2.h>
#pragma comment (lib, "ws2_32.lib")
#else
#include <errno.h>
#include <immintrin.h>
#include <arpa/inet.h>
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

    bool establishConnection(const char* address, int port)
    {
        serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (serverSocket == INVALID_SOCKET)
        {
            printf("Fail to create a socket (%d)!\n", WSAGetLastError());
            return false;
        }
        DWORD timeoutMs = 5000;
        setsockopt(serverSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeoutMs, sizeof(timeoutMs));

        sockaddr_in addr;
        ZeroMemory(&addr, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        sscanf_s(address, "%hhu.%hhu.%hhu.%hhu", &addr.sin_addr.S_un.S_un_b.s_b1, &addr.sin_addr.S_un.S_un_b.s_b2, &addr.sin_addr.S_un.S_un_b.s_b3, &addr.sin_addr.S_un.S_un_b.s_b4);
        if (connect(serverSocket, (const sockaddr*)&addr, sizeof(addr)))
        {
            printf("Fail to connect to %s (%d)!\n", address, WSAGetLastError());
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

    bool establishConnection(const char* address, int port)
    {
        serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket == -1)
        {
            printf("Fail to create a socket (%d)!\n", errno);
            return false;
        }
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        setsockopt(serverSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        if (inet_pton(AF_INET, address, &addr.sin_addr) <= 0)
        {
            printf("Invalid address (%s)\n", address);
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
        while (size)
        {
            int numberOfBytes;
            if ((numberOfBytes = recv(serverSocket, buffer, size, 0)) <= 0)
            {
                return false;
            }
            buffer += numberOfBytes;
            size -= numberOfBytes;
        }
        return true;
    }
};
