cmake_minimum_required(VERSION 3.5)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../dqrobotics_dependencies.cmake)
find_package (Threads REQUIRED)

project(free_flying_robot LANGUAGES CXX)


add_executable(free_flying_robot free_flying_robot.cpp)

TARGET_LINK_LIBRARIES(free_flying_robot
    pthread
    qpOASES
    dqrobotics
    dqrobotics-interface-vrep
)
