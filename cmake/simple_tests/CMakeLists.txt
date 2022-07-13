CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

project(simple_tests)

add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    test_issue_24.cpp
    test_issue_25.cpp
    test_issue_26.cpp
    test_issue_44.cpp
    test_issue_py_18.cpp
    test_issue_py_22.cpp
    )
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    dqrobotics
)

