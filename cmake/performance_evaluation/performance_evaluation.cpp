#include <iostream>
#include <dqrobotics/DQ.h>
#include <dqrobotics/DQ_kinematics.h>
#include <dqrobotics/robot_dh/Kukka.h>
#include <cmath>
#include <chrono>

using namespace DQ_robotics;
int main()
{
    const int RUN_COUNT = 1000;

    DQ_kinematics robot = KukkaKinematics();

    VectorXd theta;
    auto start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta = VectorXd::Random(7);
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end-start;
    std::cout << "Time to calculate a " << theta.size() << " DOF random vector " << RUN_COUNT << " times was " << diff.count() << " s." <<std::endl;

    MatrixXd pose_jacobian;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta = VectorXd::Random(7);
        pose_jacobian = robot.pose_jacobian(theta);
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate a " << pose_jacobian.cols() << " DOF Jacobian " << RUN_COUNT << " times was " << diff.count() << " s." <<std::endl;

    DQ pose;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta = VectorXd::Random(7);
        pose  = robot.fkm(theta);
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate a " << pose_jacobian.cols() << " DOF fkm " << RUN_COUNT << " times was " << diff.count() << " s." <<std::endl;

    MatrixXd translation_jacobian;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta                = VectorXd::Random(7);
        pose                 = robot.fkm(theta);
        pose_jacobian        = robot.pose_jacobian(theta);
        translation_jacobian = DQ_robotics::translation_jacobian(pose_jacobian,pose);
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate a " << translation_jacobian.cols() << " DOF translation Jacobian (including requirements) " << RUN_COUNT << " times was " << diff.count() << " s." <<std::endl;

    MatrixXd rotation_jacobian;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta             = VectorXd::Random(7);
        pose_jacobian     = robot.pose_jacobian(theta);
        rotation_jacobian = DQ_robotics::rotation_jacobian(pose_jacobian);
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate a " << rotation_jacobian.cols() << " DOF rotation Jacobian (including requirements) " << RUN_COUNT << " times was " << diff.count() << " s." <<std::endl;

    DQ a;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        a = DQ(VectorXd::Random(8));
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to create a dual quaternion from a random VectorXd " << RUN_COUNT << " times was " << diff.count() << " s." <<std::endl;

    DQ b;
    DQ c;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        a = DQ(VectorXd::Random(8));
        b = DQ(VectorXd::Random(8));
        c = a*b;
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate the dual quaternion product " << RUN_COUNT << " times was " << diff.count() << " s." <<std::endl;

    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        a = DQ(VectorXd::Random(8));
        b = DQ(VectorXd::Random(8));
        c = a+b;
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate the dual quaternion sum " << RUN_COUNT << " times was " << diff.count() << " s." <<std::endl;


    return 0;
}
