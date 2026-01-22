#include <chrono>
#include <thread>
#include <mutex>

#include "node_connection.h"

static void hexToByte(const char* hex, uint8_t* byte, const int sizeInByte)
{
    for (int i = 0; i < sizeInByte; i++){
        sscanf(hex+i*2, "%2hhx", &byte[i]);
    }
}

int main(int argc, char* argv[])
{
    if (argc < 7)
    {
        printf("Usage:  broadcastMessageSolution [Node IP] [Node Port] [MiningID] [Signing Seed] [Mining Seed] [Nonces] [Algo ID (Optional)]\n");
    }
    else
    {
        char* nodeIp = argv[1];
        int nodePort = std::atoi(argv[2]);
        char* miningID = argv[3];

        int selectedAlgoId = -1;
        if (argc > 7)
        {
            selectedAlgoId = std::atoi(argv[7]);
        }

        char* signingSeed = argv[4];
        unsigned char nonce[32];
        unsigned char randomSeed[32];
        hexToByte(argv[5], randomSeed, 32);
        hexToByte(argv[6], nonce, 32);
        
        SolutionSubmitter solutionSubmitter(nodeIp, nodePort, randomSeed, miningID, signingSeed);
        // Adjust the nonce if user request
        if (selectedAlgoId >=0 )
        {
            // Hyperidentity scoring
            if (selectedAlgoId == 0)
            {
                nonce[0] &= 0xFE;
            }
            // Addition scoring
            else if (selectedAlgoId == 1)
            {
                nonce[0] |= 0x1;
            }
            // Other just as it is
        }

        if (!solutionSubmitter.submit(nonce))
        {
            printf("Failed to send data.\n");
        }
    }
    return 0;
}