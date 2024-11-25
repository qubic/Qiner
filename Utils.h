#pragma once

#include <sstream>
#include <string>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <vector>
#include <algorithm>

namespace test_utils
{

std::string byteToHex(const std::vector<uint8_t>& byteArray, size_t sizeInByte)
{
    std::ostringstream oss;
    for (size_t i = 0; i < sizeInByte; ++i)
    {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byteArray[i]);
    }
    return oss.str();

}
std::vector<uint8_t> hexToByte(const std::string& hex, const int sizeInByte)
{
    if (hex.length() != sizeInByte * 2) {
        throw std::invalid_argument("Hex string length does not match the expected size");
    }

    std::vector<uint8_t> byteArray(sizeInByte);
    for (size_t i = 0; i < sizeInByte; ++i)
    {
        byteArray[i] = std::stoi(hex.substr(i * 2, 2), nullptr, 16);
    }

    return byteArray;
}

// Function to read and parse the CSV file
std::vector<std::vector<std::string>> readCSV(const std::string& filename)
{
    std::vector<std::vector<std::string>> data;
    std::ifstream file(filename);
    std::string line;

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
            item.erase(std::remove(item.begin(), item.end(), ' '), item.end());

            parsedLine.push_back(item);
        }
        data.push_back(parsedLine);
    }
    return data;
}

std::vector<uint8_t> convertFromString(std::string& rStr)
{
    std::vector<uint8_t> value(32);
    std::stringstream ss(rStr);
    std::string item;
    int i = 0;
    while (std::getline(ss, item, '-'))
    {
        value[i++] = std::stoull(item);
    }
    return value;
}

std::vector<unsigned long long> convertULLFromString(std::string& rStr)
{
    std::vector<unsigned long long> values;
    std::stringstream ss(rStr);
    std::string item;
    int i = 0;
    while (std::getline(ss, item, '-'))
    {
        values.push_back(std::stoull(item));
    }
    return values;
}

} // test_utils
