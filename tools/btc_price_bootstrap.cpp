// Build btc_price_bootstrap.bin, the price series a node starts its first live-feed epoch from.
//
// Reads "<hour>,<price>" lines (written by fetch_btc_prices.py), keeps the last MIN_POINTS hours and
// writes them as the node's price image, kind BOOTSTRAP. Prints the two values to copy into
// core/src/public_settings.h: BTC_FEED_MIN_HOUR and BTC_PRICE_BOOTSTRAP_HASH.
//
// Usage:
//   btc_price_bootstrap <prices.csv> <epoch> [out.bin]
//   btc_price_bootstrap --check <file.bin>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "K12AndKeyUtil.h"

static constexpr unsigned int RING_SIZE = 16384;
static constexpr unsigned int IMAGE_MAGIC = 0x46435442;
// Fewest consecutive hours a node accepts: one frame, one week of shifts and the hour before the frame.
static constexpr unsigned int MIN_POINTS = 24 * 365 + 24 * 7 + 1;
static constexpr unsigned int KIND_BOOTSTRAP = 1;
static constexpr unsigned int POINT_FLAG_BOOTSTRAP = 2;
static constexpr unsigned int MAX_FRACTION_DIGITS = 8;
static constexpr unsigned long long PRICE_SCALE = 100000000ULL;
// Largest whole part whose price in 1e-8 units still fits in 63 bits.
static constexpr unsigned long long MAX_WHOLE_PRICE = 92233720367ULL;

// Same layout as the node's price image.
struct PricePoint
{
    unsigned int hour;
    unsigned int appendTick;
    unsigned int flags;
    unsigned int reserved;
    long long priceE8;
};

struct PriceImage
{
    unsigned int magic;
    unsigned int headerSize;
    unsigned int recordSize;
    unsigned int kind;
    unsigned int epoch;
    unsigned int savedAtTick;
    unsigned char checksum[32];
    unsigned int nextHour;
    unsigned int failedAttempts;
    long long firstFailMs;
    long long lastFailMs;
    unsigned int baseHour;
    unsigned int reserved2;
    PricePoint points[RING_SIZE];
};

static constexpr unsigned int IMAGE_HEADER_SIZE = 88;

static_assert(sizeof(PricePoint) == 24, "point record must be 24 bytes without padding");
static_assert(offsetof(PriceImage, checksum) == 24, "unexpected image layout");
static_assert(offsetof(PriceImage, nextHour) == 56, "unexpected image layout");
static_assert(offsetof(PriceImage, points) == IMAGE_HEADER_SIZE, "unexpected image layout");
static_assert(sizeof(PriceImage) == IMAGE_HEADER_SIZE + RING_SIZE * sizeof(PricePoint), "unexpected image size");

struct HourPrice
{
    unsigned int hour;
    long long priceE8;
};

static PriceImage gImage;

static bool isDigits(const std::string& text)
{
    if (text.empty())
    {
        return false;
    }
    for (const char c : text)
    {
        if (c < '0' || c > '9')
        {
            return false;
        }
    }
    return true;
}

static std::string trimmed(const std::string& text)
{
    const char* const blanks = " \t\r\n";
    const size_t first = text.find_first_not_of(blanks);
    if (first == std::string::npos)
    {
        return std::string();
    }
    const size_t last = text.find_last_not_of(blanks);
    return text.substr(first, last - first + 1);
}

// Exact decimal string to 1e-8 units; no floating point in between.
static bool parsePriceE8(const std::string& text, long long& priceE8)
{
    const size_t dot = text.find('.');
    const std::string whole = text.substr(0, dot);
    const std::string fraction = (dot == std::string::npos) ? std::string() : text.substr(dot + 1);
    if (!isDigits(whole) || whole.size() > 11 || fraction.size() > MAX_FRACTION_DIGITS)
    {
        return false;
    }
    if (!fraction.empty() && !isDigits(fraction))
    {
        return false;
    }
    const unsigned long long wholeValue = strtoull(whole.c_str(), nullptr, 10);
    if (wholeValue > MAX_WHOLE_PRICE)
    {
        return false;
    }
    std::string padded = fraction;
    padded.append(MAX_FRACTION_DIGITS - fraction.size(), '0');
    const unsigned long long fractionValue = strtoull(padded.c_str(), nullptr, 10);
    priceE8 = (long long)(wholeValue * PRICE_SCALE + fractionValue);
    return true;
}

static bool readPrices(const char* path, std::vector<HourPrice>& rows)
{
    std::ifstream in(path);
    if (!in)
    {
        printf("Cannot open %s\n", path);
        return false;
    }
    std::string line;
    unsigned int lineNumber = 0;
    while (std::getline(in, line))
    {
        lineNumber++;
        const std::string text = trimmed(line);
        if (text.empty())
        {
            continue;
        }
        const size_t comma = text.find(',');
        const std::string hourText = trimmed(text.substr(0, comma));
        const std::string priceText = (comma == std::string::npos) ? std::string() : trimmed(text.substr(comma + 1));
        HourPrice row;
        if (!isDigits(hourText) || hourText.size() > 9 || !parsePriceE8(priceText, row.priceE8))
        {
            printf("%s line %u: expected <hour>,<price with at most 8 decimals>\n", path, lineNumber);
            return false;
        }
        row.hour = (unsigned int)strtoul(hourText.c_str(), nullptr, 10);
        rows.push_back(row);
    }
    return true;
}

// K12 of the image with its checksum field zeroed.
static void computeChecksum(PriceImage& image, unsigned char checksum[32])
{
    unsigned char stored[32];
    memcpy(stored, image.checksum, sizeof(stored));
    memset(image.checksum, 0, sizeof(image.checksum));
    KangarooTwelve((const unsigned char*)&image, (unsigned int)sizeof(image), checksum, 32);
    memcpy(image.checksum, stored, sizeof(stored));
}

// The last MIN_POINTS hours of rows; a later line for the same hour replaces an earlier one.
static bool buildImage(const std::vector<HourPrice>& rows, unsigned int epoch, PriceImage& image)
{
    if (rows.empty())
    {
        printf("No prices\n");
        return false;
    }
    unsigned int lastHour = 0;
    for (const HourPrice& row : rows)
    {
        if (row.hour > lastHour)
        {
            lastHour = row.hour;
        }
    }
    const unsigned int nextHour = lastHour + 1;
    if (nextHour < MIN_POINTS)
    {
        printf("Last hour %u is too low for %u hours\n", lastHour, MIN_POINTS);
        return false;
    }
    const unsigned int firstHour = nextHour - MIN_POINTS;

    std::vector<long long> prices(MIN_POINTS, 0);
    for (const HourPrice& row : rows)
    {
        if (row.hour >= firstHour)
        {
            prices[row.hour - firstHour] = row.priceE8;
        }
    }

    memset(&image, 0, sizeof(image));
    for (unsigned int hour = firstHour; hour < nextHour; hour++)
    {
        const long long priceE8 = prices[hour - firstHour];
        if (priceE8 <= 0)
        {
            printf("Hour %u has no price\n", hour);
            return false;
        }
        PricePoint& point = image.points[hour & (RING_SIZE - 1)];
        point.hour = hour;
        point.flags = POINT_FLAG_BOOTSTRAP;
        point.priceE8 = priceE8;
    }
    image.magic = IMAGE_MAGIC;
    image.headerSize = IMAGE_HEADER_SIZE;
    image.recordSize = (unsigned int)sizeof(PricePoint);
    image.kind = KIND_BOOTSTRAP;
    image.epoch = epoch;
    image.nextHour = nextHour;
    image.baseHour = lastHour;
    unsigned char checksum[32];
    computeChecksum(image, checksum);
    memcpy(image.checksum, checksum, sizeof(checksum));
    return true;
}

// True for a bootstrap image a node can use; firstHour is the first of its consecutive hours.
static bool checkImage(PriceImage& image, unsigned int& firstHour)
{
    if (image.magic != IMAGE_MAGIC || image.headerSize != IMAGE_HEADER_SIZE
        || image.recordSize != sizeof(PricePoint) || image.kind != KIND_BOOTSTRAP)
    {
        printf("Not a bootstrap image\n");
        return false;
    }
    unsigned char checksum[32];
    computeChecksum(image, checksum);
    if (memcmp(checksum, image.checksum, sizeof(checksum)) != 0)
    {
        printf("Checksum mismatch\n");
        return false;
    }
    if (image.baseHour + 1 != image.nextHour)
    {
        printf("Base hour is not the last hour\n");
        return false;
    }
    unsigned int hour = image.nextHour;
    while (hour > 0)
    {
        const PricePoint& point = image.points[(hour - 1) & (RING_SIZE - 1)];
        if (point.hour != hour - 1 || point.priceE8 <= 0)
        {
            break;
        }
        hour--;
    }
    if (image.nextHour - hour < MIN_POINTS)
    {
        printf("Only %u consecutive hours, need %u\n", image.nextHour - hour, MIN_POINTS);
        return false;
    }
    firstHour = hour;
    return true;
}

static void printSettings(const PriceImage& image, unsigned int firstHour)
{
    unsigned char hash[32];
    KangarooTwelve((const unsigned char*)&image, (unsigned int)sizeof(image), hash, 32);
    printf("epoch %u, hours %u .. %u (%u points), base hour %u\n", image.epoch, firstHour, image.nextHour - 1,
        image.nextHour - firstHour, image.baseHour);
    printf("static constexpr unsigned int BTC_FEED_MIN_HOUR = %u;\n", firstHour);
    printf("static constexpr unsigned char BTC_PRICE_BOOTSTRAP_HASH[32] =\n");
    for (unsigned int row = 0; row < 2; row++)
    {
        fputs((row == 0) ? "    { " : "      ", stdout);
        for (unsigned int i = 0; i < 16; i++)
        {
            printf("0x%02x%s", hash[row * 16 + i], (i < 15) ? ", " : "");
        }
        fputs((row == 0) ? ",\n" : " };\n", stdout);
    }
}

static bool readImage(const char* path, PriceImage& image)
{
    FILE* file = fopen(path, "rb");
    if (!file)
    {
        printf("Cannot open %s\n", path);
        return false;
    }
    const size_t readBytes = fread(&image, 1, sizeof(image), file);
    const bool atEnd = (fgetc(file) == EOF);
    fclose(file);
    if (readBytes != sizeof(image) || !atEnd)
    {
        printf("%s: wrong size, expected %zu bytes\n", path, sizeof(image));
        return false;
    }
    return true;
}

static bool writeImage(const char* path, const PriceImage& image)
{
    FILE* file = fopen(path, "wb");
    if (!file)
    {
        printf("Cannot write %s\n", path);
        return false;
    }
    const size_t written = fwrite(&image, 1, sizeof(image), file);
    const bool closed = (fclose(file) == 0);
    if (written != sizeof(image) || !closed)
    {
        printf("Cannot write %s\n", path);
        return false;
    }
    return true;
}

int main(int argc, char* argv[])
{
    if (argc == 3 && strcmp(argv[1], "--check") == 0)
    {
        unsigned int firstHour = 0;
        if (!readImage(argv[2], gImage) || !checkImage(gImage, firstHour))
        {
            return 1;
        }
        printSettings(gImage, firstHour);
        return 0;
    }
    if (argc < 3 || argc > 4 || argv[1][0] == '-')
    {
        printf("Usage: btc_price_bootstrap <prices.csv> <epoch> [out.bin]\n");
        printf("       btc_price_bootstrap --check <file.bin>\n");
        return 1;
    }

    const std::string epochText = argv[2];
    if (!isDigits(epochText) || epochText.size() > 5)
    {
        printf("Epoch must be a number\n");
        return 1;
    }
    const unsigned int epoch = (unsigned int)strtoul(epochText.c_str(), nullptr, 10);
    const char* const outPath = (argc == 4) ? argv[3] : "btc_price_bootstrap.bin";

    std::vector<HourPrice> rows;
    unsigned int firstHour = 0;
    if (!readPrices(argv[1], rows) || !buildImage(rows, epoch, gImage) || !checkImage(gImage, firstHour))
    {
        return 1;
    }
    if (!writeImage(outPath, gImage))
    {
        return 1;
    }
    printf("wrote %s (%zu bytes)\n", outPath, sizeof(gImage));
    printSettings(gImage, firstHour);
    return 0;
}
