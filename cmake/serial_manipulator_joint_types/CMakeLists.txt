cmake_minimum_required(VERSION 3.16)
include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

project(serial_manipulator_joint_types)

add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    dqrobotics
    )
