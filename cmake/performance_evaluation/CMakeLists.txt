cmake_minimum_required(VERSION 3.1)

project(performance_evaluation)
set (CMAKE_CXX_STANDARD 11)

include_directories(../../cpp/include)
add_executable(${PROJECT_NAME}
    performance_evaluation.cpp
    )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    dqrobotics
    )
