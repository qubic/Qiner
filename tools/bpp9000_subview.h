#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

#include "task_file.h"
#include "K12AndKeyUtil.h"
#include "bpp9000_params.h"

// Try to read reduced params from task file
// Its main purpose for the regression test only
namespace bpp9000_subview
{

template<typename C>
bool readSubviewBlocks(const char* taskFilePath,
                       std::vector<unsigned char>& topoBlockOut,
                       std::vector<unsigned char>& dataBlockOut)
{
    task_file::TaskFileHeader header;
    if (!task_file::readTaskFileHeader(taskFilePath, &header))
    {
        return false;
    }

    const unsigned int N = (unsigned int)C::numberOfInputNeurons;
    const unsigned int M = (unsigned int)C::numberOfOutputNeurons;
    const unsigned int P = (unsigned int)C::populationThreshold;
    const unsigned int K = (unsigned int)C::numberOfNeighbors;
    const unsigned long long T = (unsigned long long)C::sequenceLength;

    if (header.magic != task_file::MAGIC ||
        header.version != task_file::VERSION ||
        header.numInputTrits < N ||
        header.numOutputTrits != M ||
        header.numPairs < T ||
        header.population != P ||
        header.numNeighbors != K)
    {
        return false;
    }

    const unsigned int fileN = header.numInputTrits;
    const unsigned int fileM = header.numOutputTrits;
    const unsigned long long fileTopoBytes = task_file::topologyBytes(fileN, fileM, P, K);
    const unsigned long long fileInBytes = task_file::packedBytes(fileN);
    const unsigned long long fileRowBytes = fileInBytes + task_file::packedBytes(fileM);

    unsigned char hash[task_file::DATA_HASH_SIZE];

    // Topology: read the full block, verify its hash, parse with FILE dims, re-serialise the first N
    // inputs / M outputs / signal / all P*K neighbours into C's layout.
    std::vector<unsigned char> fileTopo((size_t)fileTopoBytes);
    if (!task_file::readTaskFileBlock(taskFilePath, sizeof(task_file::TaskFileHeader), fileTopo.data(), fileTopoBytes))
    {
        return false;
    }
    KangarooTwelve(fileTopo.data(), (unsigned int)fileTopoBytes, hash, task_file::DATA_HASH_SIZE);
    if (memcmp(hash, header.topologyHash, task_file::DATA_HASH_SIZE) != 0)
    {
        return false;
    }

    std::vector<uint32_t> fileInputIdx(fileN);
    std::vector<uint32_t> fileOutputIdx(fileM);
    std::vector<uint32_t> neighborIdx((size_t)P * K);
    uint32_t signalIdx = 0;
    task_file::parseTopologyBlock(fileTopo.data(), fileN, fileM, P, K,
                                  fileInputIdx.data(), fileOutputIdx.data(), &signalIdx, neighborIdx.data());
    topoBlockOut.resize((size_t)task_file::topologyBytes(N, M, P, K));
    task_file::serializeTopologyBlock(N, M, P, K, fileInputIdx.data(), fileOutputIdx.data(),
                                      signalIdx, neighborIdx.data(), topoBlockOut.data());

    // Data: read the first T rows, verify dataHash only if full, unpack per FILE row layout keeping the
    // first N inputs / M outputs, then re-pack into C's data layout.
    const unsigned long long dataOffset = sizeof(task_file::TaskFileHeader) + fileTopoBytes;
    const unsigned long long dataPrefixBytes = T * fileRowBytes;
    std::vector<unsigned char> fileData((size_t)dataPrefixBytes);
    if (!task_file::readTaskFileBlock(taskFilePath, dataOffset, fileData.data(), dataPrefixBytes))
    {
        return false;
    }
    // header.dataHash covers the scored region: the production sequenceLength rows. A file may
    // carry holdout rows beyond them (numPairs > scored length), excluded from the hash.
    if (N == fileN && M == fileM && T == bpp9000_params::ProdConfig::sequenceLength && header.numPairs >= T)
    {
        KangarooTwelve(fileData.data(), (unsigned int)dataPrefixBytes, hash, task_file::DATA_HASH_SIZE);
        if (memcmp(hash, header.dataHash, task_file::DATA_HASH_SIZE) != 0)
        {
            return false;
        }
    }

    std::vector<unsigned char> inputs((size_t)(T * N));
    std::vector<unsigned char> outputs((size_t)(T * M));
    std::vector<unsigned char> tmpIn(fileN);
    std::vector<unsigned char> tmpOut(fileM);
    for (unsigned long long t = 0; t < T; ++t)
    {
        const unsigned char* row = fileData.data() + t * fileRowBytes;
        if (!task_file::unpackTrits(row, fileN, tmpIn.data()))
        {
            return false;
        }
        if (!task_file::unpackTrits(row + fileInBytes, fileM, tmpOut.data()))
        {
            return false;
        }
        for (unsigned int i = 0; i < N; ++i)
        {
            inputs[(size_t)(t * N + i)] = tmpIn[i];
        }
        for (unsigned int j = 0; j < M; ++j)
        {
            outputs[(size_t)(t * M + j)] = tmpOut[j];
        }
    }
    dataBlockOut.resize((size_t)task_file::dataBytes(N, M, T));
    task_file::packDataBlock(N, M, T, inputs.data(), outputs.data(), dataBlockOut.data());
    return true;
}

}
