cmake_minimum_required(VERSION 2.8)

project(vrep_interface_move_kuka)
set (CMAKE_CXX_STANDARD 11)

add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)

include_directories(
    ../../cpp/include
    ../../interfaces/cpp-interface-vrep/include
    ../../interfaces/cpp-interface-vrep/coppeliarobotics/include
    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi
    )

add_executable(${PROJECT_NAME}
    vrep_interface_move_kuka.cpp
    ../../cpp/src/DQ.cpp

    ../../cpp/src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
    ../../cpp/src/robot_modeling/DQ_Kinematics.cpp
    ../../cpp/src/robot_modeling/DQ_SerialManipulator.cpp
    ../../cpp/src/robot_modeling/DQ_MobileBase.cpp
    ../../cpp/src/robot_modeling/DQ_HolonomicBase.cpp
    ../../cpp/src/robot_modeling/DQ_DifferentialDriveRobot.cpp
    ../../cpp/src/robot_modeling/DQ_WholeBody.cpp

    ../../cpp/src/utils/DQ_LinearAlgebra.cpp

    ../../cpp/src/robots/Ax18ManipulatorRobot.cpp
    ../../cpp/src/robots/BarrettWamArmRobot.cpp
    ../../cpp/src/robots/ComauSmartSixRobot.cpp
    ../../cpp/src/robots/KukaLw4Robot.cpp
    ../../cpp/src/robots/KukaYoubotRobot.cpp

    ../../interfaces/cpp-interface-vrep/src/vrep_interface.cpp
    ../../interfaces/cpp-interface-vrep/src/VrepInterfaceMapElement.cpp
    ../../interfaces/cpp-interface-vrep/src/DQ_VrepRobot.cpp
    ../../interfaces/cpp-interface-vrep/src/LBR4pVrepRobot.cpp
    ../../interfaces/cpp-interface-vrep/src/YouBotVrepRobot.cpp

    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApi.c
    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.h
    ../../interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.c
    )


TARGET_LINK_LIBRARIES(${PROJECT_NAME}
pthread
)
