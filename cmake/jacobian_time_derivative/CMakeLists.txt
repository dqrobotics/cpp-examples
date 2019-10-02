cmake_minimum_required(VERSION 3.1)

project(jacobian_time_derivative)

set (CMAKE_CXX_STANDARD 11)

add_executable(${PROJECT_NAME}
    jacobian_time_derivative.cpp
    )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    dqrobotics
    )
