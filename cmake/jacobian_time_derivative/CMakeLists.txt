cmake_minimum_required(VERSION 3.10)
include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

project(jacobian_time_derivative)

add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    dqrobotics
    )

add_executable(jacobian_time_derivative_test
    jacobian_time_derivative_test.cpp
    )

TARGET_LINK_LIBRARIES(jacobian_time_derivative_test
    dqrobotics
    )
