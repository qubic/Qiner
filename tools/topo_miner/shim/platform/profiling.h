#pragma once
// Shim for the core SIMD backend TU: the real profiling.h pulls time_stamp_counter.h (__cpuid) and
// Windows file_io.h. The score path only needs the scope macros; make them no-ops.
#define PROFILE_NAMED_SCOPE(name)
#define PROFILE_SCOPE()
