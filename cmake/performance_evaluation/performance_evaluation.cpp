#include <iostream>
#include <dqrobotics/DQ.h>
#include <dqrobotics/DQ_kinematics.h>
#include <dqrobotics/robot_dh/Kukka.h>
#include <cmath>
#include <chrono>

using namespace DQ_robotics;
int main()
{
    const int RUN_COUNT = 10000;

    DQ_kinematics robot = KukkaKinematics();

    VectorXd theta;
    auto start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta = VectorXd::Random(7);
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end-start;
    std::cout << "Time to calculate a " << theta.size() << " DOF random vector " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;

    MatrixXd pose_jacobian;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta = VectorXd::Random(7);
        pose_jacobian = robot.pose_jacobian(theta);
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate a " << pose_jacobian.cols() << " DOF Jacobian " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;

    DQ pose;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta = VectorXd::Random(7);
        pose  = robot.fkm(theta);
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate a " << pose_jacobian.cols() << " DOF fkm " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;

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
    std::cout << "Time to calculate a " << translation_jacobian.cols() << " DOF translation Jacobian (including requirements) " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;

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
    std::cout << "Time to calculate a " << rotation_jacobian.cols() << " DOF rotation Jacobian (including requirements) " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;

    MatrixXd point_to_point_distance_jacobian;
    DQ point;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta                = VectorXd::Random(7);
        pose_jacobian        = robot.pose_jacobian(theta);
        pose                 = robot.fkm(theta);
        translation_jacobian = DQ_robotics::translation_jacobian(pose_jacobian,pose);
        point                = DQ(VectorXd::Random(8));
        point                = normalize(point);

        point_to_point_distance_jacobian = DQ_robotics::point_to_point_distance_jacobian(translation_jacobian,translation(pose),translation(point));
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate a " << point_to_point_distance_jacobian.cols() << " DOF point to point distance Jacobian (including requirements) " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;

    MatrixXd point_to_line_distance_jacobian;
    DQ line;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta                = VectorXd::Random(7);
        pose_jacobian        = robot.pose_jacobian(theta);
        pose                 = robot.fkm(theta);
        translation_jacobian = DQ_robotics::translation_jacobian(pose_jacobian,pose);
        //Create random plucker line
        DQ l = DQ(VectorXd::Random(4));
        l.q(0)=0.0;
        l=normalize(l);
        DQ pl = DQ(VectorXd::Random(4));
        pl.q(0)=0.0;
        line = l+E_*cross(l,pl);

        point_to_line_distance_jacobian = DQ_robotics::point_to_line_distance_jacobian(translation_jacobian,translation(pose),line);
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate a " << point_to_line_distance_jacobian.cols() << " DOF point to line distance Jacobian (including requirements) " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;

    MatrixXd point_to_plane_distance_jacobian;
    DQ plane;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        theta                = VectorXd::Random(7);
        pose_jacobian        = robot.pose_jacobian(theta);
        pose                 = robot.fkm(theta);
        translation_jacobian = DQ_robotics::translation_jacobian(pose_jacobian,pose);
        //Create random plane
        DQ n = DQ(VectorXd::Random(4));
        n.q(0)=0.0;
        n = normalize(n);
        plane = n+E_*VectorXd::Random(1)(0);

        point_to_plane_distance_jacobian = DQ_robotics::point_to_plane_distance_jacobian(translation_jacobian,translation(pose),plane);
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate a " << point_to_plane_distance_jacobian.cols() << " DOF point to plane distance Jacobian (including requirements) " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;


    DQ a;
    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        a = DQ(VectorXd::Random(8));
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to create a dual quaternion from a random VectorXd " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;

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
    std::cout << "Time to calculate the dual quaternion product " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;

    start = std::chrono::system_clock::now();
    for(int i=0;i<RUN_COUNT;i++)
    {
        a = DQ(VectorXd::Random(8));
        b = DQ(VectorXd::Random(8));
        c = a+b;
    }
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to calculate the dual quaternion sum " << RUN_COUNT << " times was " << diff.count() << " s. Or per time [s]: " << diff.count()/double(RUN_COUNT) << std::endl;


    return 0;
}
