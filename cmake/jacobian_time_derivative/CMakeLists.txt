cmake_minimum_required(VERSION 2.8)

project(jacobian_time_derivative)
set (CMAKE_CXX_STANDARD 11)

include_directories(../../cpp/include)
add_executable(${PROJECT_NAME}
    jacobian_time_derivative.cpp
    ../../cpp/src/DQ.cpp
    ../../cpp/src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
    ../../cpp/src/robot_modeling/DQ_Kinematics.cpp
    ../../cpp/src/robot_modeling/DQ_SerialManipulator.cpp
    ../../cpp/src/robot_modeling/DQ_MobileBase.cpp
    ../../cpp/src/robot_modeling/DQ_HolonomicBase.cpp
    ../../cpp/src/robot_modeling/DQ_DifferentialDriveRobot.cpp
    ../../cpp/src/robot_modeling/DQ_WholeBody.cpp

    ../../cpp/src/robots/Ax18ManipulatorRobot.cpp
    ../../cpp/src/robots/BarrettWamArmRobot.cpp
    ../../cpp/src/robots/ComauSmartSixRobot.cpp
    ../../cpp/src/robots/KukaLw4Robot.cpp
    )
