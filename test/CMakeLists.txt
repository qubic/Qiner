project(qiner_test LANGUAGES C CXX)

add_executable(${PROJECT_NAME}
    main.cpp
    bpp9000_ant_colony_test.cpp
    # score_addition_test.cpp  # obsolete: targets the removed synapse-buffer architecture; needs a LUT rewrite
    )

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/external/catch2/include
        )

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        QINER_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

if(WIN32)
    target_link_libraries(${PROJECT_NAME} wsock32 ws2_32)
else()
    target_link_libraries(${PROJECT_NAME} pthread)
endif()