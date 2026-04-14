cmake_minimum_required(VERSION 3.10)
include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

PROJECT(json11_interface_example)

add_executable(json11_interface_example
    json11_interface_example.cpp
    )

target_link_libraries(json11_interface_example
    -ldqrobotics
    -ldqrobotics-interface-json11
    )
