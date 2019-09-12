cmake_minimum_required(VERSION 2.8)

project(simple_tests)
set (CMAKE_CXX_STANDARD 11)

include_directories(../../cpp/include)
add_executable(${PROJECT_NAME}
    simple_tests.cpp
    test_issue_24.cpp
    ../../cpp/src/DQ.cpp
    )
