#ifdef _MSC_VER
#pragma comment(lib, "Ws2_32.lib")
#include <Winsock2.h>
#include <Ws2tcpip.h>
#define close(x) closesocket(x)
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
#include <cstring>
#include <string>
#include <stdexcept>

#include "connection.h"
#include "structs.h"

#define DEFAULT_TIMEOUT_MSEC 11000 // time of each job + 1sec

#ifdef _MSC_VER

static bool setTimeout(int serverSocket, int optName, unsigned long milliseconds)
{
    DWORD tv = milliseconds;
    if (setsockopt(serverSocket, SOL_SOCKET, optName, (const char*)&tv, sizeof tv) != 0)
    {
        printf("setsockopt failed with error: %d\n", WSAGetLastError());
        return false;
    }
    return true;
}

static int connect(const char* nodeIp, int nodePort)
{
    WSADATA wsa_data;
    WSAStartup(MAKEWORD(2, 0), &wsa_data);

    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (!setTimeout(serverSocket, SO_RCVTIMEO, DEFAULT_TIMEOUT_MSEC))
        return -1;
    if (!setTimeout(serverSocket, SO_SNDTIMEO, DEFAULT_TIMEOUT_MSEC))
        return -1;
    sockaddr_in addr;
    memset((char*)&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(nodePort);

    if (inet_pton(AF_INET, nodeIp, &addr.sin_addr) <= 0) 
    {
        printf("Error translating command line ip address to usable one.");
        return -1;
    }
    int res = connect(serverSocket, (const sockaddr*)&addr, sizeof(addr));
    if (res < 0) 
    {
        printf("Failed to connect %s | error %d\n", nodeIp, res);
        return -1;
    }
    return serverSocket;
}

#else

static bool setTimeout(int serverSocket, int optName, unsigned long milliseconds)
{
    struct timeval tv;
    tv.tv_sec = milliseconds / 1000;
    tv.tv_usec = (milliseconds % 1000) * 1000;
    if (setsockopt(serverSocket, SOL_SOCKET, optName, (const char*)&tv, sizeof tv) != 0)
        return false;
    return true;
}

static int connect(const char* nodeIp, int nodePort)
{
	int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (!setTimeout(serverSocket, SO_RCVTIMEO, DEFAULT_TIMEOUT_MSEC))
        return -1;
    if (!setTimeout(serverSocket, SO_SNDTIMEO, DEFAULT_TIMEOUT_MSEC))
        return -1;
    sockaddr_in addr;
    memset((char*)&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(nodePort);

    if (inet_pton(AF_INET, nodeIp, &addr.sin_addr) <= 0) 
    {
        printf("Error translating command line ip address to usable one.");
        return -1;
    }

    if (connect(serverSocket, (const sockaddr *)&addr, sizeof(addr)) < 0) 
    {
        printf("Failed to connect %s\n", nodeIp);
        return -1;
    }
    return serverSocket;
}

#endif

QubicConnection::QubicConnection(const char* nodeIp, int nodePort)
{
	memset(mNodeIp, 0, 32);
	memcpy(mNodeIp, nodeIp, strlen(nodeIp));
	mNodePort = nodePort;
	mSocket = connect(nodeIp, nodePort);
    if (mSocket < 0)
        throw std::logic_error("Unable to establish connection.");

    // receive handshake - exchange peer packets
    mHandshakeData.resize(sizeof(ExchangePublicPeers));
    uint8_t* data = mHandshakeData.data();
    *((ExchangePublicPeers*)data) = receivePacketWithHeaderAs<ExchangePublicPeers>();
    setTimeout(mSocket, SO_RCVTIMEO, DEFAULT_TIMEOUT_MSEC);
}

void QubicConnection::exchangePeer()
{
    struct
    {
        RequestResponseHeader header;
        ExchangePublicPeers peers;
    } payload;
    payload.header.setType(ExchangePublicPeers::type());
    payload.header.setSize(sizeof(payload));
    payload.header.randomizeDejavu();
    memcpy(payload.peers.peers, mHandshakeData.data(), mHandshakeData.size());
    sendData((uint8_t*)&payload, payload.header.size());
}

void QubicConnection::getHandshakeData(std::vector<uint8_t>& buffer)
{
    buffer = mHandshakeData;
}

QubicConnection::~QubicConnection()
{
	close(mSocket);
}

RequestResponseHeader QubicConnection::receiveHeader()
{
    RequestResponseHeader header;
    int recvByte = receiveData((uint8_t*)&header, sizeof(RequestResponseHeader));
    if (recvByte != sizeof(RequestResponseHeader))
    {
        memset(&header, 0, sizeof(header));
    }
    return header;
}

// Receive the requested number of bytes (sz) or less if sz bytes have not been received after timeout. Return number of received bytes.
int QubicConnection::receiveData(uint8_t* buffer, int sz)
{
    int totalRecvSz = 0;
    while (sz)
    {
        // Note that recv may return before sz bytes have been received, it only blocks until socket timeout if no
        // data has been received!
        // Linux manual page:
        //   "If no messages are available at the socket, the receive calls wait for a message to arrive [...]
        //   The receive calls normally return any data available, up to the requested amount,
        //   rather than waiting for receipt of the full amount requested."
        // Microsoft docs:
        //   "For connection-oriented sockets (type SOCK_STREAM for example), calling recv will
        //   return as much data as is currently available�up to the size of the buffer specified. [...]
        //   If no incoming data is available at the socket, the recv call blocks and waits for data to arrive [...]"
        int recvSz = recv(mSocket, (char*)buffer + totalRecvSz, sz, 0);
        if (recvSz <= 0)
        {
            // timeout, closed connection, or other error
            break;
        }
        totalRecvSz += recvSz;
        sz -= recvSz;
    }
    return totalRecvSz;
}

int QubicConnection::receiveAllDataOrThrowException(uint8_t* buffer, int sz)
{
    int recvSz = receiveData(buffer, sz);
    if (recvSz != sz)
    {
        throw std::logic_error("Received incomplete data! Expected " + std::to_string(sz) + " bytes, received " + std::to_string(recvSz) + " bytes");
    }
    return recvSz;
}

void QubicConnection::resolveConnection()
{
    mSocket = connect(mNodeIp, mNodePort);
    if (mSocket < 0)
        throw std::logic_error("Unable to establish connection.");
}

// Receive the next qubic packet with a RequestResponseHeader that matches T
template <typename T>
T QubicConnection::receivePacketWithHeaderAs()
{
    // first receive the header
    RequestResponseHeader header;
    int recvByte = -1, packetSize = -1, remainingSize = -1;
    while (true)
    {
        recvByte = receiveData((uint8_t*)&header, sizeof(RequestResponseHeader));
        if (recvByte != sizeof(RequestResponseHeader))
        {
            throw std::logic_error("No connection.");
        }
        if (header.type() == END_RESPOND)
        {
            throw EndResponseReceived();
        }
        if (header.type() != T::type())
        {
            // skip this packet and keep receiving
            packetSize = header.size();
            remainingSize = packetSize - sizeof(RequestResponseHeader);
            receiveAllDataOrThrowException(mBuffer, remainingSize);
            continue;
        }
        break;
    }
    
    packetSize = header.size();
    remainingSize = packetSize - sizeof(RequestResponseHeader);
    T result;
    memset(&result, 0, sizeof(T));
    if (remainingSize)
    {
        memset(mBuffer, 0, sizeof(T));
        receiveAllDataOrThrowException(mBuffer, remainingSize);
        result = *((T*)mBuffer);
    }
    return result;
}

// same as receivePacketWithHeaderAs but without the header
template <typename T>
T QubicConnection::receivePacketAs()
{
    int packetSize = sizeof(T);
    T result;
    memset(&result, 0, sizeof(T));
    int recvByte = receiveData(mBuffer, packetSize);
    if (recvByte != packetSize)
    {
        throw std::logic_error("Unexpected data size.");
    }
    result = *((T*)mBuffer);
    return result;
}

template <typename T>
std::vector<T> QubicConnection::getLatestVectorPacketAs()
{
    std::vector<T> results;
    while (true)
    {
        try
        {
            results.push_back(receivePacketWithHeaderAs<T>());
        }
        catch (EndResponseReceived& e)
        {
            break;
        }
        catch (std::logic_error& e)
        {
            printf("%s\n", e.what());
            break;
        }
    }
    return results;
}

int QubicConnection::sendData(uint8_t* buffer, int sz)
{
    int size = sz;
    int numberOfBytes;
    while (size) 
    {
        if ((numberOfBytes = send(mSocket, (char*)buffer, size, 0)) <= 0) 
        {
            return 0;
        }
        buffer += numberOfBytes;
        size -= numberOfBytes;
    }
	return sz - size;
}

template SpecialCommand QubicConnection::receivePacketWithHeaderAs<SpecialCommand>();
template SpecialCommandToggleMainModeResquestAndResponse QubicConnection::receivePacketWithHeaderAs<SpecialCommandToggleMainModeResquestAndResponse>();
template SpecialCommandSetSolutionThresholdResquestAndResponse QubicConnection::receivePacketWithHeaderAs<SpecialCommandSetSolutionThresholdResquestAndResponse>();
template SpecialCommandSendTime QubicConnection::receivePacketWithHeaderAs<SpecialCommandSendTime>();
template SpecialCommandSetConsoleLoggingModeRequestAndResponse QubicConnection::receivePacketWithHeaderAs<SpecialCommandSetConsoleLoggingModeRequestAndResponse>();
template GetSendToManyV1Fee_output QubicConnection::receivePacketWithHeaderAs<GetSendToManyV1Fee_output>();
template CurrentTickInfo QubicConnection::receivePacketWithHeaderAs<CurrentTickInfo>();
template CurrentSystemInfo QubicConnection::receivePacketWithHeaderAs<CurrentSystemInfo>();
template TickData QubicConnection::receivePacketWithHeaderAs<TickData>();
template RespondTxStatus QubicConnection::receivePacketWithHeaderAs<RespondTxStatus>();
template BroadcastComputors QubicConnection::receivePacketWithHeaderAs<BroadcastComputors>();
template RespondContractIPO QubicConnection::receivePacketWithHeaderAs<RespondContractIPO>();
template ExchangePublicPeers QubicConnection::receivePacketAs<ExchangePublicPeers>();