CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

find_package(Threads REQUIRED)

project(vrep_interface)

add_executable(vrep_interface_move_kuka
    vrep_interface_move_kuka.cpp
    )


TARGET_LINK_LIBRARIES(vrep_interface_move_kuka
    dqrobotics
    dqrobotics-interface-vrep
    qpOASES
    Threads::Threads
)

add_executable(vrep_interface_move_kuka_and_youbot
    vrep_interface_move_kuka_and_youbot.cpp
    )


TARGET_LINK_LIBRARIES(vrep_interface_move_kuka_and_youbot
    dqrobotics
    dqrobotics-interface-vrep
    qpOASES
    Threads::Threads
)
