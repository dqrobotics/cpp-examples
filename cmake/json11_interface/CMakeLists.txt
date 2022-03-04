CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(json11_interface_example)
set (CMAKE_CXX_STANDARD 11)

include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

add_executable(json11_interface_example
    json11_interface_example.cpp
    )

target_link_libraries(json11_interface_example
    -ldqrobotics
    -ldqrobotics-interface-json11
    )
