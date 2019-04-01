cmake_minimum_required(VERSION 2.8)

project(performance_evaluation)
set (CMAKE_CXX_STANDARD 11)

include_directories(../../cpp/include)
add_executable(${PROJECT_NAME} "performance_evaluation.cpp" "../../cpp/src/DQ.cpp" "../../cpp/src/DQ_kinematics.cpp" "../../cpp/src/DQ_CooperativeDualTaskSpace.cpp")
