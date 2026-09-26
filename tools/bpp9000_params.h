#pragma once

#include <tuple>

namespace bpp9000_params
{

template<
    unsigned long long N, unsigned long long M, unsigned long long T, unsigned long long W,
    unsigned long long maxTicks, unsigned long long K, unsigned long long P,
    unsigned long long mutations, unsigned int threshold, unsigned long long shiftCapValue>
struct Bpp9000Config
{
    static constexpr unsigned long long numberOfInputNeurons = N;
    static constexpr unsigned long long numberOfOutputNeurons = M;
    static constexpr unsigned long long sequenceLength = T;
    static constexpr unsigned long long windowWidth = W;
    static constexpr unsigned long long maxNumberOfTicks = maxTicks;
    static constexpr unsigned long long numberOfNeighbors = K;
    static constexpr unsigned long long populationThreshold = P;
    static constexpr unsigned long long numberOfMutations = mutations;
    static constexpr unsigned int solutionThreshold = threshold;
    static constexpr unsigned long long shiftCap = shiftCapValue;
};

// Production config. The frame slides one week ahead at most.
using ProdConfig = Bpp9000Config<18, 1, 24 * 365 + 24 * 7, 24 * 365, 100000, 3, 2048, 1000,
                             (unsigned int)(24 * 365 * 45 / 100), 24 * 7>;

// Regression configs that all SUBVIEW-load from the one full production task file
// but reduce some params for faster test. Their frames carry no calendar meaning, so the
// cap is scaled to the frame width instead of a week.
using ConfigA = Bpp9000Config<18, 1, 128,      32,      5000,   3, 64, 10, (unsigned int)(32 * 45 / 100),      32 / 4>;
using ConfigB = Bpp9000Config<18, 1, 512,      128,     20000,  3, 64, 15, (unsigned int)(128 * 45 / 100),     128 / 4>;
using ConfigC = Bpp9000Config<18, 1, 2048,     512,     80000,  3, 64, 20, (unsigned int)(512 * 45 / 100),     512 / 4>;
using ConfigD = Bpp9000Config<18, 1, 24 * 365 + 24 * 7, 24 * 365, 100000, 3, 64, 5,  (unsigned int)(24 * 365 * 45 / 100), 24 * 7>;
using ConfigList = std::tuple<ConfigA, ConfigB, ConfigC, ConfigD>;

}
