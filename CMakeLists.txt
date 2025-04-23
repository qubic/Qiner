cmake_minimum_required(VERSION 3.21)
project(Qiner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if(CMAKE_CMAKE_BUILD_TYPE MATCHES DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
endif()

# Additional cmake flags
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mrdrnd -mbmi")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mrdrnd -mbmi")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
else()
    message(FATAL "Unsupported compiler.")
endif()

# Main executable
add_executable(broadcastMessageSolution src/Qiner.cpp src/keyUtils.cpp src/connection.cpp)

include_directories(broadcastMessageSolution ./src)

# Linking
if(WIN32)
    target_link_libraries(broadcastMessageSolution wsock32 ws2_32)
else()
    target_link_libraries(broadcastMessageSolution pthread)
endif()
