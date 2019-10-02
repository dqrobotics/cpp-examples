cmake_minimum_required(VERSION 2.8)

project(vrep_interface)

find_package (Threads REQUIRED)

set (CMAKE_CXX_STANDARD 11)
#This needs to be set to your CPLEX library PATH
set( CPLEX_PATH /opt/ibm/ILOG/CPLEX_Studio1271 )

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
    ../../cpp/include
    ../../interfaces/cpp-interface-vrep/include
    ../../interfaces/cpp-interface-vrep/coppeliarobotics/include
    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi
    ${CPLEX_PATH}/concert/include
    ${CPLEX_PATH}/cplex/include
    )

add_executable(vrep_interface_move_kuka
    vrep_interface_move_kuka.cpp

    ../../cpp/src/DQ.cpp

    ../../cpp/src/robot_modeling/DQ_Kinematics.cpp
    ../../cpp/src/robot_modeling/DQ_SerialManipulator.cpp

    ../../cpp/src/utils/DQ_LinearAlgebra.cpp

    ../../cpp/src/robots/KukaLw4Robot.cpp

    ../../interfaces/cpp-interface-vrep/src/DQ_VrepInterface.cpp
    ../../interfaces/cpp-interface-vrep/src/DQ_VrepInterfaceMapElement.cpp

    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApi.c
    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.h
    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.c
    )


TARGET_LINK_LIBRARIES(vrep_interface_move_kuka
pthread
)


add_executable(vrep_interface_move_kuka_and_youbot
    vrep_interface_move_kuka_and_youbot.cpp

    ../../cpp/src/DQ.cpp

    ../../cpp/src/robot_modeling/DQ_Kinematics.cpp
    ../../cpp/src/robot_modeling/DQ_SerialManipulator.cpp
    ../../cpp/src/robot_modeling/DQ_MobileBase.cpp
    ../../cpp/src/robot_modeling/DQ_HolonomicBase.cpp
    ../../cpp/src/robot_modeling/DQ_WholeBody.cpp

    ../../cpp/src/utils/DQ_LinearAlgebra.cpp
    ../../cpp/src/utils/DQ_Geometry.cpp

    ../../cpp/src/robots/KukaLw4Robot.cpp

    ../../cpp/src/robot_control/DQ_ClassicQPController.cpp
    ../../cpp/src/robot_control/DQ_KinematicConstrainedController.cpp
    ../../cpp/src/robot_control/DQ_KinematicController.cpp
    ../../cpp/src/robot_control/DQ_PseudoinverseController.cpp
    ../../cpp/src/robot_control/DQ_QuadraticProgrammingController.cpp

    ../../interfaces/cpp-interface-vrep/src/DQ_VrepInterface.cpp
    ../../interfaces/cpp-interface-vrep/src/DQ_VrepInterfaceMapElement.cpp
    ../../interfaces/cpp-interface-vrep/src/DQ_VrepRobot.cpp
    ../../interfaces/cpp-interface-vrep/src/LBR4pVrepRobot.cpp
    ../../interfaces/cpp-interface-vrep/src/YouBotVrepRobot.cpp

    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApi.c
    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.h
    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.c
    )


TARGET_LINK_LIBRARIES(vrep_interface_move_kuka_and_youbot
    Threads::Threads
    ilocplex
    cplex
    concert
)
