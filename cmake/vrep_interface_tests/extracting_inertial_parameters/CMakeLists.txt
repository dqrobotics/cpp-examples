cmake_minimum_required(VERSION 3.1)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../dqrobotics_dependencies.cmake)
find_package (Threads REQUIRED)

project(extracting_inertial_parameters)


add_executable(extracting_inertial_parameters
               extracting_inertial_parameters.cpp
    )


TARGET_LINK_LIBRARIES(extracting_inertial_parameters
    pthread
    dqrobotics
    dqrobotics-interface-vrep
)



