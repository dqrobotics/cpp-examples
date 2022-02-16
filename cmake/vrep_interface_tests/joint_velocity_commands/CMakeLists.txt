cmake_minimum_required(VERSION 3.1)

project(joint_velocity_commands)

find_package (Threads REQUIRED)

set (CMAKE_CXX_STANDARD 11)
find_package (Threads REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

add_definitions(-DIL_STD)
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)



add_executable(joint_velocity_commands
    joint_velocity_commands.cpp
    )


TARGET_LINK_LIBRARIES(joint_velocity_commands
    pthread
    dqrobotics
    dqrobotics-interface-vrep
)

