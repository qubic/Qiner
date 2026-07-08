#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

// Generic binary container of P time-ordered (input, output) trit rows. This module owns only the
// header layout, trit packing, and file IO. 
// Trits are packed five per byte, one byte holds
// t0 + 3*t1 + 9*t2 + 27*t3 + 81*t4 with each trit in {0, 1, 2}, so a valid byte is 0..242. Input
// and output are packed per field; the last byte of each field zero-pads unused positions. The
// file is native little-endian.
namespace task_file
{

// Convenience magic/version for the Generic LUT task; callers pass these to write/load.
static constexpr uint32_t MAGIC = 0x5454554CU;
static constexpr uint32_t VERSION = 1;

// Store each value as a trit (3 states {0,1,2}, not a bit {0,1}) so the format can carry a third
// state in future tasks. TRIT_BASE is how many states one value can take; 
// TRITS_PER_BYTE is how many trits are packed into a byte (5 because 3^5 = 243 <= 256 is the tightest fit).
static constexpr unsigned int TRIT_BASE = 3;
static constexpr unsigned int TRITS_PER_BYTE = 5;

// exp-th power of base, evaluated at compile time.
constexpr uint64_t ipow(uint64_t base, unsigned int exp)
{
    return exp == 0 ? 1 : base * ipow(base, exp - 1);
}

// A packed byte holds values 0 .. BYTE_VALUE_LIMIT - 1 (243 = 3^5); anything at or above is invalid.
static constexpr unsigned int BYTE_VALUE_LIMIT = (unsigned int)ipow(TRIT_BASE, TRITS_PER_BYTE);

// Byte length of the data hash carried in the header.
static constexpr unsigned int DATA_HASH_SIZE = 32;

#pragma pack(push, 1)
struct TaskFileHeader
{
    uint32_t magic;
    uint32_t version;
    uint32_t numInputTrits;      // N, features per sample
    uint32_t numOutputTrits;     // M, graded outputs per sample
    uint64_t numPairs;           // P, number of samples
    uint32_t population;         // ANN population (compatibility check)
    unsigned char dataHash[DATA_HASH_SIZE];  // hash of the data (filled from outside)
    uint32_t reserved;
};
#pragma pack(pop)

static_assert(sizeof(TaskFileHeader) == 64, "TaskFileHeader must be exactly 64 bytes");

// ceil(tritCount / TRITS_PER_BYTE): bytes needed to hold that many trits at five trits per byte.
inline uint64_t packedBytes(uint64_t tritCount)
{
    return (tritCount + (TRITS_PER_BYTE - 1)) / TRITS_PER_BYTE;
}

// Pack count trits {0,1,2} into out[], five per byte. Unused positions in the last byte are zero.
inline void packTrits(const unsigned char* trits, uint64_t count, unsigned char* out)
{
    uint64_t byteIndex = 0;
    for (uint64_t i = 0; i < count; i += TRITS_PER_BYTE)
    {
        unsigned int packed = 0;
        unsigned int weight = 1;
        for (unsigned int k = 0; k < TRITS_PER_BYTE; ++k)
        {
            const unsigned char t = (i + k < count) ? trits[i + k] : 0;
            packed += (unsigned int)t * weight;
            weight *= TRIT_BASE;
        }
        out[byteIndex] = (unsigned char)packed;
        byteIndex++;
    }
}

// Unpack count trits from bytes[] (five per byte). Returns false if any byte is not a valid
// base-243 group (>= BYTE_VALUE_LIMIT). Only the first count trits are written; padding is skipped.
inline bool unpackTrits(const unsigned char* bytes, uint64_t count, unsigned char* trits)
{
    uint64_t byteIndex = 0;
    for (uint64_t i = 0; i < count; i += TRITS_PER_BYTE)
    {
        unsigned int packed = bytes[byteIndex];
        if (packed >= BYTE_VALUE_LIMIT)
        {
            return false;
        }
        for (unsigned int k = 0; k < TRITS_PER_BYTE; ++k)
        {
            if (i + k < count)
            {
                trits[i + k] = (unsigned char)(packed % TRIT_BASE);
            }
            packed /= TRIT_BASE;
        }
        byteIndex++;
    }
    return true;
}

// Write a task file at path. The caller supplies magic, version, and the 32-byte dataHash.
// inputsTrits holds numPairs * numInputTrits trits row-major (row t is sample t's N input trits);
// outputsTrits holds numPairs * numOutputTrits trits row-major.
inline bool writeTaskFile(const char* path,
                          uint32_t magic,
                          uint32_t version,
                          uint32_t numInputTrits,
                          uint32_t numOutputTrits,
                          uint64_t numPairs,
                          uint32_t population,
                          const unsigned char* dataHash,
                          const unsigned char* inputsTrits,
                          const unsigned char* outputsTrits)
{
    const uint64_t inBytes = packedBytes(numInputTrits);
    const uint64_t outBytes = packedBytes(numOutputTrits);
    const uint64_t rowBytes = inBytes + outBytes;
    const uint64_t dataBytes = numPairs * rowBytes;

    std::vector<unsigned char> data((size_t)dataBytes, 0);
    for (uint64_t p = 0; p < numPairs; ++p)
    {
        unsigned char* row = data.data() + p * rowBytes;
        packTrits(inputsTrits + p * numInputTrits, numInputTrits, row);
        packTrits(outputsTrits + p * numOutputTrits, numOutputTrits, row + inBytes);
    }

    TaskFileHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = magic;
    header.version = version;
    header.numInputTrits = numInputTrits;
    header.numOutputTrits = numOutputTrits;
    header.numPairs = numPairs;
    header.population = population;
    memcpy(header.dataHash, dataHash, DATA_HASH_SIZE);

    FILE* f = fopen(path, "wb");
    if (f == nullptr)
    {
        return false;
    }
    const bool ok =
        fwrite(&header, 1, sizeof(header), f) == sizeof(header) &&
        fwrite(data.data(), 1, (size_t)dataBytes, f) == dataBytes;
    fclose(f);
    return ok;
}

// Read just the 64-byte header, so the caller can validate magic, version, dimensions, and
// population before committing to reading (and allocating) the data section.
inline bool readTaskFileHeader(const char* path, TaskFileHeader* outHeader)
{
    FILE* f = fopen(path, "rb");
    if (f == nullptr)
    {
        return false;
    }
    const bool ok = fread(outHeader, 1, sizeof(*outHeader), f) == sizeof(*outHeader);
    fclose(f);
    return ok;
}

// Read and unpack the data section into the caller's outInputs / outOutputs buffers
// each must hold header.numPairs * numInputTrits and header.numPairs *
// numOutputTrits trits, which the caller has already validated via the header). header is the one
// returned by readTaskFileHeader
inline bool readTaskFileData(const char* path, 
                             const TaskFileHeader& header,
                             unsigned char* outInputs,
                             unsigned char* outOutputs,
                             const unsigned char* expectedDataHash = nullptr)
{
    if (expectedDataHash != nullptr && memcmp(header.dataHash, expectedDataHash, DATA_HASH_SIZE) != 0)
    {
        return false;
    }

    FILE* f = fopen(path, "rb");
    if (f == nullptr)
    {
        return false;
    }

    const uint64_t inBytes = packedBytes(header.numInputTrits);
    const uint64_t outBytes = packedBytes(header.numOutputTrits);
    const uint64_t rowBytes = inBytes + outBytes;

    // Confirm the file size matches the header's dimensions exactly, without trusting (or overflowing
    // on) the declared numPairs. This also bounds the read below to the real file size.
    if (fseek(f, 0, SEEK_END) != 0)
    {
        fclose(f);
        return false;
    }
    const long fileSize = ftell(f);
    if (fileSize < (long)sizeof(header))
    {
        fclose(f);
        return false;
    }
    const uint64_t availData = (uint64_t)fileSize - sizeof(header);
    if (rowBytes == 0 || availData % rowBytes != 0 || availData / rowBytes != header.numPairs)
    {
        fclose(f);
        return false;
    }
    if (fseek(f, (long)sizeof(header), SEEK_SET) != 0)
    {
        fclose(f);
        return false;
    }

    std::vector<unsigned char> data((size_t)availData, 0);
    if (fread(data.data(), 1, (size_t)availData, f) != availData)
    {
        fclose(f);
        return false;
    }
    fclose(f);

    const uint64_t numPairs = header.numPairs;
    const uint32_t numInputTrits = header.numInputTrits;
    const uint32_t numOutputTrits = header.numOutputTrits;

    for (uint64_t p = 0; p < numPairs; ++p)
    {
        const unsigned char* row = data.data() + p * rowBytes;
        if (!unpackTrits(row, numInputTrits, outInputs + p * numInputTrits))
        {
            return false;
        }
        if (!unpackTrits(row + inBytes, numOutputTrits, outOutputs + p * numOutputTrits))
        {
            return false;
        }
    }
    return true;
}

} // namespace task_file
