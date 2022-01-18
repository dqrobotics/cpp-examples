/**
(C) Copyright 2019 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#include <iostream>
#include <dqrobotics/robots/KukaLw4Robot.h>
#include <cmath>

using namespace DQ_robotics;
int main()
{

    //Create a new DQ_kinematics object with KUKA LWR parameters
    DQ_SerialManipulatorDH kuka = KukaLw4Robot::kinematics();

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
        MatrixXd jacob_dot = kuka.pose_jacobian_derivative(theta,theta_dot,6);
        //First-order numerical approximation of the Jacobian time derivative
        MatrixXd jacob_diff = (1.0/T)*(kuka.raw_pose_jacobian(theta+theta_dot*T,6)-kuka.raw_pose_jacobian(theta,6));

        std::cout << "Coefficient with largest error = " << jacob_diff.maxCoeff() << std::endl;
    }

    return 0;
}
