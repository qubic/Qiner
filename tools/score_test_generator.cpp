#include <iostream>
#include <thread>
#include <vector>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <memory>
#include <cassert>
#include <algorithm>
#include <mutex>
#ifdef _MSC_VER
#include <intrin.h>
#else
#include <immintrin.h>

#endif

#include "score_params.h"
#include "score_hyperidentity.h"
#include "score_addition.h"

using namespace score_params;

union m256i
{
    // access for loops and compatibility with __m256i
    int8_t m256i_i8[32];
    int16_t m256i_i16[16];
    int32_t m256i_i32[8];
    int64_t m256i_i64[4];
    uint8_t m256i_u8[32];
    uint16_t m256i_u16[16];
    uint32_t m256i_u32[8];
    uint64_t m256i_u64[4];

    void setRandomValue()
    {
        _rdrand64_step(reinterpret_cast<unsigned long long *>(&m256i_u64[0]));
        _rdrand64_step(reinterpret_cast<unsigned long long *>(&m256i_u64[1]));
        _rdrand64_step(reinterpret_cast<unsigned long long *>(&m256i_u64[2]));
        _rdrand64_step(reinterpret_cast<unsigned long long *>(&m256i_u64[3]));
    }
};

static std::string byteToHex(const unsigned char *byteArray, size_t sizeInByte)
{
    std::ostringstream oss;
    for (size_t i = 0; i < sizeInByte; ++i)
    {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byteArray[i]);
    }
    return oss.str();
}

static void hexToByte(const std::string &hex, const int sizeInByte, unsigned char *out)
{
    if (hex.length() != sizeInByte * 2)
    {
        throw std::invalid_argument("Hex string length does not match the expected size");
    }

    for (size_t i = 0; i < sizeInByte; ++i)
    {
        out[i] = std::stoi(hex.substr(i * 2, 2), nullptr, 16);
    }
}

// Function to read and parse the CSV file
static std::vector<std::vector<std::string>> readCSV(const std::string &filename)
{
    std::vector<std::vector<std::string>> data;
    std::ifstream file(filename);
    std::string line;

    // First line is the header
    std::getline(file, line);
    
    // Read each line from the file
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> parsedLine;

        // Parse each item separated by commas
        while (std::getline(ss, item, ','))
        {
            // Remove any spaces in the string
            item.erase(remove_if(item.begin(), item.end(), isspace), item.end());

            parsedLine.push_back(item);
        }
        data.push_back(parsedLine);
    }
    return data;
}

constexpr unsigned int kDefaultTotalSamples = 32;
std::vector<m256i> miningSeeds;
std::vector<m256i> publicKeys;
std::vector<m256i> nonces;
std::vector<m256i> spectrumDigests;
std::vector<std::vector<unsigned int>> scoreResults;
std::vector<std::vector<unsigned long long>> scoreProcessingTimes;
unsigned int processedSamplesCount = 0;
AlgoType gSelectedAlgorithm = AlgoType::HyperIdentity;
std::mutex gMutex;

template <typename P>
void writeParamsHeader(std::ostream &os, const std::string &sep = "-")
{
    // Because currently 2 params set shared the same things, incase of new algo have different params
    // need to make a separate check
    if constexpr (P::algoType == AlgoType::HyperIdentity)
    {
        os << P::numberOfInputNeurons << sep
           << P::numberOfOutputNeurons << sep
           << P::numberOfTicks << sep
           << P::numberOfNeighbors << sep
           << P::populationThreshold << sep
           << P::numberOfMutations << sep
           << P::solutionThreshold;
    }
    else if constexpr (P::algoType == AlgoType::Addition)
    {
        os << P::numberOfInputNeurons << sep
           << P::numberOfOutputNeurons << sep
           << P::numberOfTicks << sep
           << P::numberOfNeighbors << sep
           << P::populationThreshold << sep
           << P::numberOfMutations << sep
           << P::solutionThreshold;
    }
    else
    {
        std::cerr << "UNKNOWN ALGO !" << std::endl;
    }
}

template <std::size_t... Is>
void writeConfigs(std::ostream &oFile, std::index_sequence<Is...>)
{
    constexpr std::size_t N = sizeof...(Is);

    switch (gSelectedAlgorithm)
    {
    case AlgoType::HyperIdentity:
        // HyperIdentity
        ((writeParamsHeader<typename std::tuple_element_t<Is, ConfigList>::HyperIdentity>(oFile),
          (Is < N - 1 ? (oFile << ", ", 0) : 0)),
         ...);
        break;
    case AlgoType::Addition:
        // Addition
        ((writeParamsHeader<typename std::tuple_element_t<Is, ConfigList>::Addition>(oFile),
          (Is < N - 1 ? (oFile << ", ", 0) : 0)),
         ...);
        break;
    default:
        break;
    }
}

// Recursive template to process each element in scoreSettings
template <unsigned long long i>
static void processElement(unsigned char *miningSeed, unsigned char *publicKey, unsigned char *nonce, unsigned char *spectrumDigest, int threadId, bool writeFile)
{
    using CurrentConfig = std::tuple_element_t<i, ConfigList>;
    auto t0 = std::chrono::high_resolution_clock::now();

    using AdditionMiner = typename score_addition::Miner<
        CurrentConfig::Addition::AdditionParams::numberOfInputNeurons,
        CurrentConfig::Addition::AdditionParams::numberOfOutputNeurons,
        CurrentConfig::Addition::AdditionParams::numberOfTicks,
        CurrentConfig::Addition::AdditionParams::numberOfNeighbors,
        CurrentConfig::Addition::AdditionParams::populationThreshold,
        CurrentConfig::Addition::AdditionParams::numberOfMutations,
        CurrentConfig::Addition::AdditionParams::solutionThreshold>;

    using HyperIdentityMiner = typename score_hyberidentity::Miner<
        CurrentConfig::HyperIdentity::HyperIdentityParams::numberOfInputNeurons,
        CurrentConfig::HyperIdentity::HyperIdentityParams::numberOfOutputNeurons,
        CurrentConfig::HyperIdentity::HyperIdentityParams::numberOfTicks,
        CurrentConfig::HyperIdentity::HyperIdentityParams::numberOfNeighbors,
        CurrentConfig::HyperIdentity::HyperIdentityParams::populationThreshold,
        CurrentConfig::HyperIdentity::HyperIdentityParams::numberOfMutations,
        CurrentConfig::HyperIdentity::HyperIdentityParams::solutionThreshold>;

    unsigned int score_value = 0;
    if (gSelectedAlgorithm == AlgoType::HyperIdentity)
    {
        std::unique_ptr<HyperIdentityMiner> miner = std::make_unique<HyperIdentityMiner>();
        miner->initialize(miningSeed);
        score_value = miner->computeScore(publicKey, nonce);
    }
    else if (gSelectedAlgorithm == AlgoType::Addition)
    {
        std::unique_ptr<AdditionMiner> miner = std::make_unique<AdditionMiner>();
        miner->initialize(miningSeed, spectrumDigest);
        score_value = miner->computeScore(publicKey, nonce);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    auto d = t1 - t0;
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(d);
    scoreResults[threadId][i] = score_value;
    scoreProcessingTimes[threadId][i] = elapsed.count();
}

// Main processing function
template <std::size_t N, std::size_t... Is>
static void processHelper(unsigned char *miningSeed, unsigned char *publicKey, unsigned char *nonce, unsigned char *spectrumDigest, int threadId, bool writeFile, std::index_sequence<Is...>)
{
    (processElement<Is>(miningSeed, publicKey, nonce, spectrumDigest, threadId, writeFile), ...);
}

// Recursive template to process each element in scoreSettings
template <std::size_t N>
static void process(unsigned char *miningSeed, unsigned char *publicKey, unsigned char *nonce, unsigned char *spectrumDigest, int threadId = 0, bool writeFile = true)
{
    processHelper<N>(miningSeed, publicKey, nonce, spectrumDigest, threadId, writeFile, std::make_index_sequence<N>{});
}

int generateSamples(std::string sampleFileName, unsigned int numberOfSamples, bool initMiningZeros = false)
{
    // Generate a list of samples
    if (numberOfSamples > 0)
    {
        // Open the sample file if there is any
        if (sampleFileName.empty())
        {
            std::cerr << "Sample file name is empty. Exit!";
            return 1;
        }

        std::cout << "Generating sample file " << sampleFileName << " ..." << std::endl;
        miningSeeds.resize(numberOfSamples);
        publicKeys.resize(numberOfSamples);
        nonces.resize(numberOfSamples);
        spectrumDigests.resize(numberOfSamples);
        for (unsigned int i = 0; i < numberOfSamples; i++)
        {
            publicKeys[i].setRandomValue();
            nonces[i].setRandomValue();
            spectrumDigests[i].setRandomValue();
            if (initMiningZeros)
            {
                memset(miningSeeds[i].m256i_u8, 0, 32);
            }
            else
            {
                miningSeeds[i].setRandomValue();
            }
        }

        std::ofstream sampleFile;
        sampleFile.open(sampleFileName);
        if (!sampleFile.is_open())
        {
            std::cerr << "Open file " << sampleFileName << " failed. Exit!";
            return 1;
        }

        // Write the input to file
        sampleFile << "seed, publickey, nonce, spectrumdigest" << std::endl;
        for (unsigned int i = 0; i < numberOfSamples; i++)
        {
            auto miningSeedHexStr = byteToHex(miningSeeds[i].m256i_u8, 32);
            auto publicKeyHexStr = byteToHex(publicKeys[i].m256i_u8, 32);
            auto nonceHexStr = byteToHex(nonces[i].m256i_u8, 32);
            auto spectrumDigestHexStr = byteToHex(spectrumDigests[i].m256i_u8, 32);
            sampleFile
                << miningSeedHexStr << ", "
                << publicKeyHexStr << ", "
                << nonceHexStr << ", "
                << spectrumDigestHexStr << std::endl;
        }
        if (sampleFile.is_open())
        {
            sampleFile.close();
        }
        std::cout << "Generated sample file DONE " << std::endl;
    }
    else // Read the samples from file
    {
        std::cout << "Reading sample file " << sampleFileName << " ..." << std::endl;
        // Open the sample file if there is any
        if (!std::filesystem::exists(sampleFileName))
        {
            std::cerr << "Sample file name is not existed. Exit!";
            return 1;
        }

        auto sampleString = readCSV(sampleFileName);
        unsigned long long totalSamples = sampleString.size();
        std::cout << "There are " << totalSamples << " samples " << std::endl;

        miningSeeds.resize(totalSamples);
        publicKeys.resize(totalSamples);
        nonces.resize(totalSamples);
        spectrumDigests.resize(totalSamples);
        for (auto i = 0; i < totalSamples; i++)
        {
            if (sampleString[i].size() != 3 && sampleString[i].size() != 4)
            {
                std::cout << "Number of elements is mismatched. " << sampleString[i].size() << " vs 3 or 4" << " Exiting..." << std::endl;
                return 1;
            }
            if (initMiningZeros)
            {
                memset(miningSeeds[i].m256i_u8, 0, 32);
            }
            else
            {
                hexToByte(sampleString[i][0], 32, miningSeeds[i].m256i_u8);
            }

            hexToByte(sampleString[i][1], 32, publicKeys[i].m256i_u8);
            hexToByte(sampleString[i][2], 32, nonces[i].m256i_u8);

            // Spectrum Digest (Addition only). Legacy 3-column files default to a zero digest.
            if (sampleString[i].size() == 4)
            {
                hexToByte(sampleString[i][3], 32, spectrumDigests[i].m256i_u8);
            }
            else
            {
                memset(spectrumDigests[i].m256i_u8, 0, 32);
            }
        }
        std::cout << "Read sample file DONE " << std::endl;
    }
    return 0;
}

void generateScore(
    std::string sampleFileName,
    std::string outputFile,
    unsigned int threadsCount,
    unsigned int numberOfSamples,
    bool initMiningZeros = false)
{
    int sts = 0;

    // Generate samples
    sts = generateSamples(sampleFileName, numberOfSamples, initMiningZeros);
    if (sts)
    {
        return;
    }

    // Check if the ouput file name is not empty
    if (outputFile.empty())
    {
        std::cout << "Empty output file exiting..." << std::endl;
        return;
    }

    // Write the headers for output score file
    std::ofstream scoreFile;
    scoreFile.open(outputFile);
    if (!scoreFile.is_open())
    {
        std::cerr << "Open file " << outputFile << " failed. Exit!";
        return;
    }

    // Number of params settings
    constexpr unsigned long long numberOfGeneratedSetting = CONFIG_COUNT;

    // Write the header config
    writeConfigs(scoreFile, std::make_index_sequence<std::tuple_size_v<ConfigList>>{});
    scoreFile << std::endl;
    if (scoreFile.is_open())
    {
        scoreFile.close();
    }

    // Prepare memory for generated scores
    unsigned long long totalSamples = nonces.size();
    scoreResults.resize(totalSamples);
    scoreProcessingTimes.resize(totalSamples);
    for (unsigned long long i = 0; i < totalSamples; ++i)
    {
        scoreResults[i].resize(numberOfGeneratedSetting);
        scoreProcessingTimes[i].resize(numberOfGeneratedSetting);
    }

    bool writeFilePerSample = false;
    auto worker = [&](unsigned int threadIdx, unsigned int numThreads) 
    {
        for (unsigned int i = threadIdx; i < totalSamples; i += numThreads)
        {
            if (writeFilePerSample)
            {
                std::string fileName = "score_" + std::to_string(i) + ".txt";
                std::ofstream output_file(fileName);
                if (output_file.is_open())
                {
                    output_file.close();
                }
            }
            process<numberOfGeneratedSetting>(miningSeeds[i].m256i_u8, publicKeys[i].m256i_u8, nonces[i].m256i_u8, spectrumDigests[i].m256i_u8, i, writeFilePerSample);

            {
                std::lock_guard<std::mutex> lock(gMutex);
                processedSamplesCount++;
                if (processedSamplesCount % 16 == 0)
                {
                    std::cout << "\rProcessed  " << processedSamplesCount << " / " << totalSamples;
                }
            }
        }
    };

    unsigned int processThreadCount = threadsCount > totalSamples ? totalSamples : threadsCount;
    std::vector<std::thread> threads;
    for (unsigned int t = 0; t < processThreadCount; ++t)
    {
        threads.emplace_back(worker, t, threadsCount);
    }

    // Wait for all threads
    for (auto& t : threads)
    {
        t.join();
    }

    // Write to a general file
    std::cout << "\nGenerate scores DONE. Collect all into a file..." << std::endl;
    scoreFile.open(outputFile, std::ios::app);
    if (!scoreFile.is_open())
    {
        return;
    }
    for (int i = 0; i < totalSamples; i++)
    {
        for (int j = 0; j < numberOfGeneratedSetting; j++)
        {
            scoreFile << scoreResults[i][j];
            if (j < numberOfGeneratedSetting - 1)
            {
                scoreFile << ", ";
            }
        }
        scoreFile << std::endl;
    }
    scoreFile.close();
}

void print_random_test_case()
{
    // generate random input data
    m256i nonce;
    nonce.setRandomValue();
    m256i publicKey;
    publicKey.setRandomValue();

    if (gSelectedAlgorithm == AlgoType::HyperIdentity)
    {
        nonce.m256i_u8[0] &= 0xFE;
    }
    else if (gSelectedAlgorithm == AlgoType::Addition)
    {
        nonce.m256i_u8[0] |= 0x1;
    }

    auto publicKeyHexStr = byteToHex(publicKey.m256i_u8, 32);
    auto nonceHexStr = byteToHex(nonce.m256i_u8, 32);
    std::cout 
        << publicKeyHexStr << ", "
        << nonceHexStr << std::endl;
}

void printHelp()
{
    std::cout << "Usage: program [options]\n";
    std::cout << "--help, -h  Show this help message\n";
    std::cout << "--mode, -m <mode>                         Available mode: unittest, generator\n";
    std::cout << "                                            unittest: print the random unitest code that can be passed into score unittest \n";
    std::cout << "                                            generator: generate grouthtruth file in csv format that use for score unittest \n";
    std::cout << "--samplefile, -s <filename>              [generator] Sample file \n";
    std::cout << "--numsamples, -n <number>                [generator] Number of samples,  \n";
    std::cout << "                                              zeros/unset sample in samplefile will be use\n";
    std::cout << "                                              otherwise generate new samplefile\n";
    std::cout << "--threads, -t    <number>                [generator] Number of threads use for generating\n";
    std::cout << "--miningzero, -z                         [generator] Force mining seed init as zeros \n";
    std::cout << "--scorefile, -o <output score file>      [generator] Output score file \n";
}

int main(int argc, char *argv[])
{
    std::string mode;
    std::string sampleFile;
    std::string scoreFile;
    std::string selectedAlgorithm;
    unsigned int numberOfSamples = 0;
    bool miningInitZeros = false;
    unsigned int numberOfThreads = std::thread::hardware_concurrency();

    // Loop through each argument
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        // Check for specific arguments
        if (arg == "--help" || arg == "-h")
        {
            printHelp();
            return 0;
        }
        else if (arg == "--mode" || arg == "-m")
        {
            mode = std::string(argv[++i]);
        }
        else if (arg == "--samplefile" || arg == "-s")
        {
            sampleFile = std::string(argv[++i]);
        }
        else if (arg == "--numsamples" || arg == "-n")
        {
            numberOfSamples = std::stoi(argv[++i]);
        }
        else if (arg == "--threads" || arg == "-t")
        {
            numberOfThreads = std::stoi(argv[++i]);
        }
        else if (arg == "--miningzero" || arg == "-z")
        {
            miningInitZeros = true;
        }
        else if (arg == "--scorefile" || arg == "-o")
        {
            scoreFile = std::string(argv[++i]);
        }
        else if (arg == "--algo" || arg == "-a")
        {
            selectedAlgorithm = argv[++i];
            if (selectedAlgorithm == "addition")
            {
                gSelectedAlgorithm = AlgoType::Addition;
            }
            else if (selectedAlgorithm == "hyperidentity")
            {
                gSelectedAlgorithm = AlgoType::HyperIdentity;
            }
            else
            {
                std::cerr << "Unknown algorithm selection.!";
                return -1;
            }
        }
        else
        {
            std::cout << "Unknown argument: " << arg << "\n";
            printHelp();
        }
    }

    if (gSelectedAlgorithm == AlgoType::Addition)
    {
        selectedAlgorithm = "addition";
    }
    else if (gSelectedAlgorithm == AlgoType::HyperIdentity)
    {
        selectedAlgorithm = "hyperidentity";
    }
    std::cout << "SelectedAlgorithm: " << selectedAlgorithm << std::endl;
    std::cout << "Mode: " << mode << std::endl;

    // Print random unittest and exit
    if (mode == "unittest")
    {
        numberOfSamples = std::max(numberOfSamples, kDefaultTotalSamples);
        std::cout << "  Number of samples: " << numberOfSamples << std::endl;
        std::cout << "Publickey, Nonce" << std::endl;
        for (unsigned int i = 0; i < numberOfSamples; ++i)
        {
            print_random_test_case();
        }
        return 0;
    }

    // Generate score to file
    std::cout << "  Sample file: " << sampleFile << std::endl;
    std::cout << "  Init mining zeros: " << miningInitZeros << std::endl;
    std::cout << "  Output score file: " << scoreFile << std::endl;

    std::cout << "Score generator using " << numberOfThreads << " threads." << std::endl;
    generateScore(sampleFile, scoreFile, numberOfThreads, numberOfSamples, miningInitZeros);

    return 0;
}
