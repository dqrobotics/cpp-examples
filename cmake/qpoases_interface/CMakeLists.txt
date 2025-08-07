CMAKE_MINIMUM_REQUIRED(VERSION 3.5...3.26)
include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

project(qpoases_interface)

set (CMAKE_CXX_STANDARD 11)

add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    qpOASES
    dqrobotics
    )
