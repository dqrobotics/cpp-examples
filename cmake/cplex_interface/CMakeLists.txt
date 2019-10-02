cmake_minimum_required(VERSION 3.1)

project(cplex_interface_test)

find_package (Threads REQUIRED)

set (CMAKE_CXX_STANDARD 11)
#This needs to be set to your CPLEX library PATH
set( CPLEX_PATH /opt/ibm/ILOG/CPLEX_Studio1271 )

add_definitions(-DIL_STD)

link_directories(
    ${CPLEX_PATH}/cplex/lib/x86-64_linux/static_pic
    ${CPLEX_PATH}/concert/lib/x86-64_linux/static_pic
    ${CPLEX_PATH}/bin/glnxa64
    )

include_directories(
    ${CPLEX_PATH}/concert/include
    ${CPLEX_PATH}/cplex/include
    )

add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    )


TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    Threads::Threads
    ilocplex
    cplex
    concert
    dqrobotics
    )
