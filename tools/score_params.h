#pragma once

#include <tuple>

enum AlgoType
{
    HyperIdentity = 0,
    Addition = 1,
    MaxAlgoCount // for counting current supported algo
};

// =============================================================================
// Algorithm 0: HyperIdentity Parameters
// =============================================================================
template<
    unsigned long long inputNeurons,   // numberOfInputNeurons
    unsigned long long outputNeurons,   // numberOfOutputNeurons
    unsigned long long ticks,   // numberOfTicks
    unsigned long long neighbor,  // numberOfNeighbors
    unsigned long long population,   // populationThreshold
    unsigned long long mutations,   // numberOfMutations
    unsigned int threshold          // solutionThreshold
>
struct HyperIdentityParams
{
    static constexpr unsigned long long numberOfInputNeurons = inputNeurons;
    static constexpr unsigned long long numberOfOutputNeurons = outputNeurons;
    static constexpr unsigned long long numberOfTicks = ticks;
    static constexpr unsigned long long numberOfNeighbors = neighbor;
    static constexpr unsigned long long populationThreshold = population;
    static constexpr unsigned long long numberOfMutations = mutations;
    static constexpr unsigned int solutionThreshold = threshold;

    static constexpr AlgoType algoType = AlgoType::HyperIdentity;
    static constexpr unsigned int paramsCount = 7;
};

// =============================================================================
// Algorithm 1: Addition Parameters
// =============================================================================
template<
    unsigned long long inputNeurons,   // numberOfInputNeurons
    unsigned long long outputNeurons,   // numberOfOutputNeurons
    unsigned long long ticks,   // numberOfTicks
    unsigned long long neighbor,  // maxNumberOfNeigbor
    unsigned long long population,   // populationThreshold
    unsigned long long mutations,   // numberOfMutations
    unsigned int threshold          // solutionThreshold
>
struct AdditionParams
{
    static constexpr unsigned long long numberOfInputNeurons = inputNeurons;
    static constexpr unsigned long long numberOfOutputNeurons = outputNeurons;
    static constexpr unsigned long long numberOfTicks = ticks;
    static constexpr unsigned long long numberOfNeighbors = neighbor;
    static constexpr unsigned long long populationThreshold = population;
    static constexpr unsigned long long numberOfMutations = mutations;
    static constexpr unsigned int solutionThreshold = threshold;

    static constexpr AlgoType algoType = AlgoType::Addition;
    static constexpr unsigned int paramsCount = 7;
};

namespace score_params
{

static constexpr unsigned int MAX_PARAM_TYPE = 7;

template<typename HI, typename ADD>
struct ConfigPair
{
    using HyperIdentity = HI;
    using Addition = ADD;
};

// All configurations
using Config0 = ConfigPair<
    HyperIdentityParams<64, 64, 50, 64, 178, 50, 36>,
    AdditionParams<2 * 2, 3, 120, 3, 64, 100, 19>
>;

using Config1 = ConfigPair<
    HyperIdentityParams<256, 256, 120, 256, 612, 100, 171>,
    AdditionParams<4 * 2, 5, 120, 3, 128, 100, 512>
>;

using Config2 = ConfigPair<
    HyperIdentityParams<512, 512, 150, 512, 1174, 150, 300>,
    AdditionParams<7 * 2, 8, 120, 3, 256, 100, 52428>
>;

using Config3 = ConfigPair<
    HyperIdentityParams<1024, 1024, 200, 1024, 3000, 200, 600>,
    AdditionParams<7 * 2, 8, 120, 3, 512, 100, 52428>
>;

using ConfigList = std::tuple<Config0, Config1, Config2, Config3>;

static constexpr std::size_t CONFIG_COUNT = std::tuple_size_v<ConfigList>;

}
