# Offline pre-epoch topology mining.
# QUBIC_CORE_DIR is declared in the root CMakeLists.txt.
option(TOPO_MINER_AVX512 "Build the topology miners' AVX-512 (vbmi) fast paths" ON)

# --- Error-based miner: scores with core's AVX-512 engine when a core checkout is present, else the
#     scalar reference. Needs QUBIC_CORE_DIR for the SIMD backend.
if(EXISTS "${QUBIC_CORE_DIR}/src/mining/score_bpp9000.h")
    if(TOPO_MINER_AVX512)
        set(TOPO_SIMD_FLAGS "-O2;-mbmi;-mavx;-mavx2;-mavx512f;-mavx512cd;-mavx512vl;-mavx512bw;-mavx512dq;-mavx512vbmi")
    else()
        set(TOPO_SIMD_FLAGS "-O2;-mbmi;-mavx;-mavx2")
    endif()
    message(STATUS "bpp9000_topo_miner: core SIMD backend, flags ${TOPO_SIMD_FLAGS}")
    add_executable(bpp9000_topo_miner bpp9000_topo_miner.cpp topo_simd_scorer.cpp)
    target_compile_definitions(bpp9000_topo_miner PRIVATE TOPO_HAVE_CORE_SIMD)
    set_source_files_properties(topo_simd_scorer.cpp PROPERTIES
        COMPILE_OPTIONS "${TOPO_SIMD_FLAGS}"
        COMPILE_DEFINITIONS "NO_UEFI;NDEBUG"
        INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/shim;${QUBIC_CORE_DIR}/src;${QUBIC_CORE_DIR}")
else()
    message(STATUS "bpp9000_topo_miner: QUBIC_CORE_DIR not set or invalid, reference scorer only")
    add_executable(bpp9000_topo_miner bpp9000_topo_miner.cpp)
endif()

target_include_directories(bpp9000_topo_miner
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(bpp9000_topo_miner
    PRIVATE
    ${TOOLS_LINK_LIBS})

# --- Avalanche (sensitivity) miner: self-contained scalar + AVX-512 vpermb kernel, no core needed.
if(TOPO_MINER_AVX512)
    message(STATUS "bpp9000_avalanche_miner: scalar + AVX-512 (vbmi) kernels")
    add_executable(bpp9000_avalanche_miner
        bpp9000_avalanche_miner.cpp avalanche_kernel.cpp avalanche_kernel_avx512.cpp)
    target_compile_definitions(bpp9000_avalanche_miner PRIVATE TOPO_HAVE_AVALANCHE_AVX512)
    set_source_files_properties(avalanche_kernel_avx512.cpp PROPERTIES
        COMPILE_OPTIONS "-O2;-mbmi;-mavx;-mavx2;-mavx512f;-mavx512cd;-mavx512vl;-mavx512bw;-mavx512dq;-mavx512vbmi")
else()
    message(STATUS "bpp9000_avalanche_miner: scalar kernel only")
    add_executable(bpp9000_avalanche_miner bpp9000_avalanche_miner.cpp avalanche_kernel.cpp)
endif()

target_include_directories(bpp9000_avalanche_miner
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(bpp9000_avalanche_miner
    PRIVATE
    ${TOOLS_LINK_LIBS})
