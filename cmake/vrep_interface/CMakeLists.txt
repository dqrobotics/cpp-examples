cmake_minimum_required(VERSION 3.1)

project(vrep_interface)

find_package (Threads REQUIRED)

set (CMAKE_CXX_STANDARD 11)

# By default, the AUTO_CPLEX_PATH variable is set to true. This means that your CPLEX library
# will founded automatically. However, your CPLEX library must be located in /opt/ibm/ILOG CPLEX.

#If you want to set your custom CPLEX libraty path, set to "false" the following line:
set(AUTO_CPLEX_PATH true)  # Ex: set(AUTO_CPLEX_PATH false)

if (NOT AUTO_CPLEX_PATH) # Add manually your CPLEX library path:
    set( CPLEX_PATH /opt/ibm/ILOG/CPLEX_Studio201 )

else ()    #Add automatically your libraty path located in /opt/ibm/ILOG CPLEX

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

endif () #Endif

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
    dl
)
