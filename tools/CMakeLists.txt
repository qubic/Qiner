project(tools LANGUAGES CXX)
find_package(Threads REQUIRED)

# Main executable
add_executable(score_test_generator score_test_generator.cpp)
add_executable(broadcastMessageSolution 
    broadcastMessageSolution.cpp
    ${CMAKE_SOURCE_DIR}/src/keyUtils.cpp)

target_include_directories(score_test_generator 
    PRIVATE 
    ${CMAKE_SOURCE_DIR}/src)

target_include_directories(broadcastMessageSolution 
    PRIVATE 
    ${CMAKE_SOURCE_DIR}/src)

# Linking
set(TOOLS_LINK_LIBS Threads::Threads)

target_link_libraries(score_test_generator 
    PRIVATE    
    ${TOOLS_LINK_LIBS})
target_link_libraries(broadcastMessageSolution 
    PRIVATE    
    ${TOOLS_LINK_LIBS})