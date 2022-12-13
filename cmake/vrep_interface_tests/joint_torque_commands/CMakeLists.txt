cmake_minimum_required(VERSION 3.1)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../dqrobotics_dependencies.cmake)

find_package(Threads REQUIRED)

project(joint_torque_commands)


add_executable(joint_torque_commands
    joint_torque_commands.cpp
    )


TARGET_LINK_LIBRARIES(joint_torque_commands
    pthread
    dqrobotics
    dqrobotics-interface-vrep
)

