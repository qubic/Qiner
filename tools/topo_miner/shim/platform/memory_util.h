#pragma once
// Shim for the core SIMD backend TU: provide setMem/copyMem via the real memory.h (clean under
// NO_UEFI), skipping the console_logging/uefi chain the real memory_util.h drags in.
#include "platform/memory.h"
