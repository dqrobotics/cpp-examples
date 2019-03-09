cmake_minimum_required(VERSION 2.8)

project(jacobian_time_derivative)
#add_executable(${PROJECT_NAME} "main.cpp")


## Test
include_directories(../../../cpp/include)
add_executable(${PROJECT_NAME} "main.cpp" "../../../cpp/src/DQ.cpp" "../../../cpp/src/DQ_kinematics.cpp")
