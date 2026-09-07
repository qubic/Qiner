// Core SIMD backend for the topology miner: scores candidates with core's AVX-512 window-batched
// engine. This is the only TU that includes core headers - Qiner's K12AndKeyUtil.h and core's
// kangaroo_twelve.h define the same K12 symbols, so the two scorers stay in separate TUs behind
// topo_scorer.h. Built only when CMake finds a core checkout (see tools/CMakeLists.txt); the
// shim include dir precedes the core src dir so score_common.h picks up the no-op
// platform/{profiling,memory_util,concurrency}.h shims while everything else resolves to the
// real core headers. Requires an AVX512-VBMI CPU at runtime (vpermb).

#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include "bpp9000_params.h"
#include "topo_scorer.h"

// score_common.h reads these from core's public_settings.h, which does not compile on clang/gcc
// (unguarded L"" arrays). Only threshold helpers outside the fixed-LUT scoring path use them.
static constexpr unsigned int NEURAXON_SOLUTION_THRESHOLD_DEFAULT = 1;
static constexpr unsigned long long BPP9000_NUMBER_OF_WINDOWS =
    bpp9000_params::ProdConfig::sequenceLength - bpp9000_params::ProdConfig::windowWidth;
static constexpr unsigned int BPP9000_SOLUTION_THRESHOLD_DEFAULT =
    bpp9000_params::ProdConfig::solutionThreshold;

#include "mining/score_bpp9000.h"

// Core platform/memory.h declares these extern under NO_UEFI.
void setMem(void* buffer, unsigned long long size, unsigned char value)
{
    memset(buffer, value, size);
}

void copyMem(void* destination, const void* source, unsigned long long length)
{
    memcpy(destination, source, length);
}

bool allocatePool(unsigned long long size, void** buffer)
{
    *buffer = malloc(size);
    return *buffer != nullptr;
}

void freePool(void* buffer)
{
    free(buffer);
}

namespace
{

using Engine = score_engine::ScoreBpp9000<bpp9000_params::ProdConfig>;

}

struct TopoScorer
{
    std::unique_ptr<Engine> engine;
    std::vector<unsigned char> lutAbsolute;
};

TopoScorer* topoScorerCreate(const unsigned char* dataBlock, const unsigned char* lut)
{
    auto scorer = std::make_unique<TopoScorer>();
    scorer->engine = std::make_unique<Engine>();
    scorer->engine->initMemory();
    scorer->lutAbsolute.assign(lut, lut + Engine::maxNumberOfNeurons * Engine::lutSize);
    // The data block never changes during a run; unpack it once here instead of per candidate.
    if (!score_task_file::unpackDataBlock(
        Engine::numberOfInputNeurons, Engine::numberOfOutputNeurons, Engine::sequenceLength,
        dataBlock, &scorer->engine->inputs[0][0], &scorer->engine->outputs[0][0]))
    {
        return nullptr;
    }
    return scorer.release();
}

unsigned int topoScorerScore(TopoScorer* scorer, const unsigned char* topoBlock)
{
    Engine* engine = scorer->engine.get();
    score_task_file::parseTopologyBlock(topoBlock,
        Engine::numberOfInputNeurons, Engine::numberOfOutputNeurons,
        Engine::populationThreshold, Engine::numberOfNeighbors,
        engine->inputNeuronIndices, engine->outputNeuronIndices,
        &engine->signalNeuronIndex, engine->neighborIndices);
    if (!engine->validateTopology())
    {
        return Engine::INFINITE_ERROR;
    }
    engine->deriveNeuronRoles();

    // The engine stores the LUT densely by updated-neuron position: row k holds neuron
    // updatedNeuronIndices[k]'s LUT. Input neurons have no row - their LUT is never evaluated.
    Engine::ANN dense;
    memset(&dense, 0, sizeof(dense));
    for (unsigned long long k = 0; k < engine->numberOfUpdatedNeurons; ++k)
    {
        memcpy(dense.lut + k * Engine::lutSize,
               scorer->lutAbsolute.data() + engine->updatedNeuronIndices[k] * Engine::lutSize,
               Engine::lutSize);
    }
    engine->expand(dense, engine->currentANN);
    return engine->score();
}

void topoScorerDestroy(TopoScorer* scorer)
{
    delete scorer;
}

const char* topoScorerBackendName()
{
    return "core-simd";
}
