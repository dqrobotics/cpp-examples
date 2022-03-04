cmake_minimum_required(VERSION 3.1)
project(simple_tests)
set (CMAKE_CXX_STANDARD 11)

include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    test_issue_24.cpp
    test_issue_25.cpp
    test_issue_26.cpp
    test_issue_py_18.cpp
    test_issue_py_22.cpp
    )
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    -ldqrobotics
)

