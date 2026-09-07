#pragma once

// Fixed-LUT topology scorer behind a build-time backend switch: the core SIMD engine when a core
// checkout was found at configure time (TOPO_HAVE_CORE_SIMD, see CMakeLists.txt), the scalar
// Qiner scorer otherwise. The LUT is passed in absolute-neuron layout (row n = neuron n's 27
// entries); a backend reorders internally as its engine requires.
// Instances are independent so every search thread owns one; a single instance is not thread-safe.

struct TopoScorer;

// The data block and the LUT stay fixed across all candidates an instance scores. Returns null
// when the data block does not unpack.
TopoScorer* topoScorerCreate(const unsigned char* dataBlock, const unsigned char* lutAbsolute);

// Error of the fixed LUT under one candidate topology; 0xFFFFFFFF on invalid topology or timeout.
unsigned int topoScorerScore(TopoScorer* scorer, const unsigned char* topoBlock);

void topoScorerDestroy(TopoScorer* scorer);

const char* topoScorerBackendName();
