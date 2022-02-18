cmake_minimum_required(VERSION 3.1)

project(extracting_inertial_parameters)

find_package (Threads REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

set (CMAKE_CXX_STANDARD 11)


add_definitions(-DIL_STD)
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)




add_executable(extracting_inertial_parameters
               extracting_inertial_parameters.cpp
    )


TARGET_LINK_LIBRARIES(extracting_inertial_parameters
    pthread
    dqrobotics
    dqrobotics-interface-vrep
)



