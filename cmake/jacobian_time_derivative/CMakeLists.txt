cmake_minimum_required(VERSION 2.8)

project(jacobian_time_derivative)

include_directories(../../cpp/include)
add_executable(${PROJECT_NAME} "jacobian_time_derivative.cpp" "../../cpp/src/DQ.cpp" "../../cpp/src/DQ_kinematics.cpp")
