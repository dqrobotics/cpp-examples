cmake_minimum_required(VERSION 3.1)
project(multiplication_evaluation)
set (CMAKE_CXX_STANDARD 11)

include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    -ldqrobotics
    )
