CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(json11_interface_example)
set (CMAKE_CXX_STANDARD 11)

if(APPLE)
        include_directories(
        /usr/local/include/
        /usr/local/include/eigen3
        )
endif()

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(EIGEN3_INCLUDE_DIR)

add_executable(json11_interface_example
    json11_interface_example.cpp
    )
    
target_link_libraries(json11_interface_example
    -ldqrobotics
    -ldqrobotics-interface-json11
    )
