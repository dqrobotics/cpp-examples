cmake_minimum_required(VERSION 3.1)


include(${CMAKE_CURRENT_SOURCE_DIR}/../../dqrobotics_dependencies.cmake)
find_package (Threads REQUIRED)

project(test_dynamic_conic_constraint)

add_executable(test_dynamic_conic_constraint
    test_dynamic_conic_constraint.cpp
    )


TARGET_LINK_LIBRARIES(test_dynamic_conic_constraint
    pthread
    qpOASES
    dqrobotics
    dqrobotics-interface-vrep
)

