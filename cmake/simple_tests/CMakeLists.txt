cmake_minimum_required(VERSION 2.8)

project(simple_tests)
set (CMAKE_CXX_STANDARD 11)

include_directories(../../cpp/include)
add_executable(${PROJECT_NAME}
    simple_tests.cpp
    test_issue_24.cpp
    test_issue_25.cpp
    test_issue_26.cpp

    ../../cpp/src/DQ.cpp

    ../../cpp/src/utils/DQ_Geometry.cpp
    ../../cpp/src/utils/DQ_LinearAlgebra.cpp

    ../../cpp/src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
    ../../cpp/src/robot_modeling/DQ_Kinematics.cpp
    ../../cpp/src/robot_modeling/DQ_SerialManipulator.cpp
    ../../cpp/src/robot_modeling/DQ_MobileBase.cpp
    ../../cpp/src/robot_modeling/DQ_HolonomicBase.cpp
    ../../cpp/src/robot_modeling/DQ_DifferentialDriveRobot.cpp
    ../../cpp/src/robot_modeling/DQ_WholeBody.cpp

    ../../cpp/src/robot_control/DQ_KinematicController.cpp
    ../../cpp/src/robot_control/DQ_TaskSpacePseudoInverseController.cpp
    ../../cpp/src/robot_control/DQ_KinematicConstrainedController.cpp
    ../../cpp/src/robot_control/DQ_TaskspaceQuadraticProgrammingController.cpp
    ../../cpp/src/robot_control/DQ_ClassicQPController.cpp

    ../../cpp/src/robots/Ax18ManipulatorRobot.cpp
    ../../cpp/src/robots/BarrettWamArmRobot.cpp
    ../../cpp/src/robots/ComauSmartSixRobot.cpp
    ../../cpp/src/robots/KukaLw4Robot.cpp
    ../../cpp/src/robots/KukaYoubotRobot.cpp

    )
