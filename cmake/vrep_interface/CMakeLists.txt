cmake_minimum_required(VERSION 3.1)

project(vrep_interface)

find_package (Threads REQUIRED)

set (CMAKE_CXX_STANDARD 11)
#This needs to be set to your CPLEX library PATH
#set( CPLEX_PATH /opt/ibm/ILOG/CPLEX_Studio1271 )

# Find the path to CPLEX Studio.
set(CPLEX_ILOG_DIRS /opt/ibm/ILOG /opt/IBM/ILOG)
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CPLEX_ARCH x86-64)
else ()
    set(CPLEX_ARCH x86)
endif ()
if (NOT CPLEX_STUDIO_DIR)
  foreach (dir ${CPLEX_ILOG_DIRS})
    file(GLOB CPLEX_STUDIO_DIRS "${dir}/CPLEX_Studio*")
    list(SORT CPLEX_STUDIO_DIRS)
    list(REVERSE CPLEX_STUDIO_DIRS)
    if (CPLEX_STUDIO_DIRS)
      list(GET CPLEX_STUDIO_DIRS 0 CPLEX_STUDIO_DIR_)
      message(STATUS "Found CPLEX Studio: ${CPLEX_STUDIO_DIR_}")
      break ()
    endif ()
  endforeach ()
  if (NOT CPLEX_STUDIO_DIR_)
    set(CPLEX_STUDIO_DIR_ CPLEX_STUDIO_DIR-NOTFOUND)
  endif ()
  set(CPLEX_STUDIO_DIR ${CPLEX_STUDIO_DIR_} CACHE PATH
    "Path to the CPLEX Studio directory")
endif ()

set(CPLEX_PATH ${CPLEX_STUDIO_DIR})

add_definitions(-DIL_STD)
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)

link_directories(
    ${CPLEX_PATH}/cplex/lib/x86-64_linux/static_pic
    ${CPLEX_PATH}/concert/lib/x86-64_linux/static_pic
    ${CPLEX_PATH}/bin/glnxa64
    )

include_directories(
    ${CPLEX_PATH}/concert/include
    ${CPLEX_PATH}/cplex/include
    )

add_executable(vrep_interface_move_kuka
    vrep_interface_move_kuka.cpp
    )


TARGET_LINK_LIBRARIES(vrep_interface_move_kuka
    pthread
    dqrobotics
    dqrobotics-interface-vrep
)


add_executable(vrep_interface_move_kuka_and_youbot
    vrep_interface_move_kuka_and_youbot.cpp
    )


TARGET_LINK_LIBRARIES(vrep_interface_move_kuka_and_youbot
    Threads::Threads
    ilocplex
    cplex
    concert
    dqrobotics
    dqrobotics-interface-vrep
)
