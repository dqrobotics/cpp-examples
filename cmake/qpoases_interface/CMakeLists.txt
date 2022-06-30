cmake_minimum_required(VERSION 3.1)

project(qpoases_interface)

set (CMAKE_CXX_STANDARD 11)

add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    qpOASES
    dqrobotics
    )
