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


# Multiplication evaluation
add_executable(multiplication_evaluation  multiplication_evaluation.cpp)

target_link_libraries(multiplication_evaluation dqrobotics)
