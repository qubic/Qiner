cmake_minimum_required(VERSION 3.21)
project(Qiner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

option(ENABLE_AVX512 "Enable AVX512 instruction sets" OFF)

if(CMAKE_CMAKE_BUILD_TYPE MATCHES DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
endif()

# Additional cmake flags
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    if(ENABLE_AVX512)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx512f  -mrdrnd -mbmi")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx512f  -mrdrnd -mbmi")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2 -mrdrnd -mbmi")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx2 -mrdrnd -mbmi")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    if(ENABLE_AVX512)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX512")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX512")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX2")
    endif()
else()
    message(FATAL "Unsupported compiler.")
endif()

# Main executable
add_executable(${PROJECT_NAME} src/Qiner.cpp)

# Linking
if(WIN32)
    target_link_libraries(${PROJECT_NAME} wsock32 ws2_32)
else()
    target_link_libraries(${PROJECT_NAME} pthread)
endif()

if(ENABLE_AVX512)
    target_compile_definitions(${PROJECT_NAME} PUBLIC ENABLE_AVX512=1)
endif()
