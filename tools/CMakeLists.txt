project(tools LANGUAGES CXX)
find_package(Threads REQUIRED)

# Main executable
add_executable(score_test_generator score_test_generator.cpp)

target_include_directories(score_test_generator 
    PRIVATE 
    ${CMAKE_SOURCE_DIR}/src)

# Linking
set(TOOLS_LINK_LIBS Threads::Threads)

target_link_libraries(score_test_generator
    PRIVATE
    ${TOOLS_LINK_LIBS})

add_executable(lut_task_generator lut_task_generator.cpp)

target_include_directories(lut_task_generator
    PRIVATE
    ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(lut_task_generator
    PRIVATE
    ${TOOLS_LINK_LIBS})