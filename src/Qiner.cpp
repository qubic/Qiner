#include <chrono>
#include <thread>
#include <mutex>
#include <cstdio>
#include <cstring>
#include <array>
#include <queue>
#include <atomic>
#include <assert.h>

#include "node_connection.h"

#include "score_hyperidentity.h"
#include "score_addition.h"
#include "keyUtils.h"

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

using AdditionMiner = score_addition::Miner<
    score_addition::NUMBER_OF_INPUT_NEURONS,
    score_addition::NUMBER_OF_OUTPUT_NEURONS,
    score_addition::NUMBER_OF_TICKS,
    score_addition::MAX_NEIGHBOR_NEURONS,
    score_addition::POPULATION_THRESHOLD,
    score_addition::NUMBER_OF_MUTATIONS,
    score_addition::SOLUTION_THRESHOLD>;
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
    std::unique_ptr<AdditionMiner> additionMiner(new AdditionMiner());
    additionMiner->initialize(randomSeed);

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
            solutionFound = additionMiner->findSolution(computorPublicKey, nonce.data());
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

static void hexToByte(const char* hex, uint8_t* byte, const int sizeInByte)
{
    for (int i = 0; i < sizeInByte; i++){
        sscanf(hex+i*2, "%2hhx", &byte[i]);
    }
}

int main(int argc, char* argv[])
{
    char* nodeIp = NULL;
    int nodePort = 0;
    std::vector<std::thread> miningThreads;
    if (argc != 7)
    {
        printf(
            "Usage:   Qiner [Node IP] [Node Port] [MiningID] [Signing Seed] [Mining Seed] [Number "
            "of threads]\n");
    }
    else
    {
        nodeIp = argv[1];
        nodePort = std::atoi(argv[2]);
        char* miningID = argv[3];
        printf("Qiner is launched. Connecting to %s:%d\n", nodeIp, nodePort);

        consoleCtrlHandler();

        char* signingSeed = argv[4];
        hexToByte(argv[5], randomSeed, 32);
        getPublicKeyFromIdentity(miningID, computorPublicKey);
        
        unsigned int numberOfThreads = atoi(argv[6]);
        printf("%d threads are used.\n", numberOfThreads);

        SolutionSubmitter solutionSubmitter(nodeIp, nodePort, randomSeed, miningID, signingSeed);
        miningThreads.reserve(numberOfThreads);
        for (unsigned int i = numberOfThreads; i-- > 0;)
        {
            miningThreads.emplace_back(miningThreadProc);
        }

        {
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
                    bool solutionSendSuccess = solutionSubmitter.submit(&sendNonce[0]);
                    if (solutionSendSuccess)
                    {
                        std::lock_guard<std::mutex> guard(foundNonceLock);
                        // Send data successfully. Remove it from the queue
                        foundNonce.pop();
                        itemToSend = foundNonce.size();
                    }
                }

                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));

                unsigned long long delta = std::chrono::duration_cast<std::chrono::milliseconds>(
                                               std::chrono::steady_clock::now() - timestamp)
                                               .count();
                if (delta >= 1000)
                {
                    // Get current time in UTC
                    std::time_t now_time =
                        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                    std::tm* utc_time = std::gmtime(&now_time);
                    printf(
                        "|   %04d-%02d-%02d %02d:%02d:%02d   |   %llu it/s   |   %d solutions   |  "
                        " %.10s...   |\n",
                        utc_time->tm_year + 1900,
                        utc_time->tm_mon,
                        utc_time->tm_mday,
                        utc_time->tm_hour,
                        utc_time->tm_min,
                        utc_time->tm_sec,
                        (numberOfMiningIterations - prevNumberOfMiningIterations) * 1000 / delta,
                        numberOfFoundSolutions.load(),
                        miningID);
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
        printf(
            "Hyperidentity sols / nonces: %llu / %llu \n",
            qinerStat.totalHyperIdentitySols.load(),
            qinerStat.totalHyperIdentityNonce.load());
        printf(
            "Addition sols / nonces: %llu / %llu \n",
            qinerStat.totalAdditionSols.load(),
            qinerStat.totalAdditionNonce.load());

        printf("Qiner is shut down.\n");
    }

    return 0;
}