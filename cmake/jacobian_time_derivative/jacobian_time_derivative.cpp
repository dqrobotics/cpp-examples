#include <iostream>
#include <dqrobotics/DQ.h>
#include <dqrobotics/DQ_kinematics.h>
#include <dqrobotics/robot_dh/Kukka.h>
#include <cmath>

using namespace DQ_robotics;
int main()
{

    //Create a new DQ_kinematics object with KUKA LWR parameters
    DQ_kinematics kuka = KukkaKinematics();

    //Integration step for the numerical calculations
    double T = 1e-3;
    //Final time
    double T_end = 2*M_PI;

    VectorXd theta = VectorXd::Zero(7);

    for(double t=0.0;t<T_end;t+=T)
    {
        //For simplicity, all joint trajectories are the same. All joints
        //rotate at a frequency of T rad/s.
        theta = sin(T*t)*VectorXd::Ones(7);
        //This is the analytical time derivative of the joint trajectories.
        VectorXd theta_dot = T*cos(t)*VectorXd::Ones(7);
        //Calculation of the analytical Jacobian time derivative.
        MatrixXd jacob_dot = kuka.pose_jacobian_derivative(theta,theta_dot,7);
        //First-order numerical approximation of the Jacobian time derivative
        MatrixXd jacob_diff = (1.0/T)*(kuka.raw_pose_jacobian(theta+theta_dot*T,7)-kuka.raw_pose_jacobian(theta,7));

        std::cout << "Coefficient with largest error = " << jacob_diff.maxCoeff() << std::endl;
    }

    return 0;
}
