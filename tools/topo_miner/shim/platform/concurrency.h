#pragma once
// Shim for the core SIMD backend TU: the score path is single-threaded and the real concurrency.h
// is MSVC-only (_InterlockedExchange8). Provide nothing - nothing in the score path uses it.
