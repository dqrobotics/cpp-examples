CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

project(performance_evaluation)

add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    dqrobotics
    )
