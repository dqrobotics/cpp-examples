/**
(C) Copyright 2022-2023 DQ Robotics Developers
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
- Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
    - Initial implementation

- Murilo M. Marinho (murilomarinho@ieee.org)
    - Removed includes for thread and DQ_VrepInterface.
    - Added include for vector.
    - Forward declared methods to fix the compilation warnings.
*/

#include <vector>

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
#include <dqrobotics/robots/KukaLw4Robot.h>
#include <dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/utils/DQ_Constants.h>


using namespace Eigen;
using namespace DQ_robotics;


std::vector<MatrixXd> numerical_differentiation(const std::vector<Eigen::MatrixXd>& J,
                                                const double& T);


bool check_pose_jacobian_derivative(const std::shared_ptr<DQ_Kinematics>& robot,
                                    int& iterations,
                                    double& T,
                                    double& threshold);

std::string map_return(const bool& value);


int main()
{
    double T       = 1e-4;
    int iterations = 1500;
    double threshold = 1e-10;

    //------------test the numerical differentiation implementation----//
    double t = 0;
    double w = 2*pi;
    std::vector<Eigen::MatrixXd> M;
    std::vector<Eigen::MatrixXd> M_dot;
    std::vector<Eigen::MatrixXd> Numerical_M_dot;
    bool accurate_computation = true;
    for(int i=0;i<iterations;i++)
    {
        t = i*T;
        double theta = sin(w*t);
        double theta_dot = w*cos(w*t);
        M.push_back((MatrixXd(1,1) << theta).finished());
        M_dot.push_back((MatrixXd(1,1) << theta_dot).finished());
    }
    Numerical_M_dot = numerical_differentiation(M, T);
    for(int i=0;i<iterations;i++)
    {
       auto numerical_error = M_dot[i] - Numerical_M_dot[i];
       double     max_coeff = numerical_error.maxCoeff();
       //std::cout<<"Error: "<<numerical_error.maxCoeff()<<std::endl;
       if (i>2 && i<iterations-2) //discard the first two and last two values.
       {
           if(std::abs(max_coeff) > threshold)
           {
                accurate_computation = false;
           }
       }
    }
    std::cout<<"is numerical_differentiation() working? "
            <<map_return(accurate_computation)<<std::endl;


    //------------Test for DQ_DifferentialDriveRobot()-----------------//
    auto diff_base = std::make_shared<DQ_DifferentialDriveRobot>(DQ_DifferentialDriveRobot(0.3, 0.01));
    bool resultdr = check_pose_jacobian_derivative(diff_base,iterations, T, threshold);
    std::cout<<"is pose_jacobian_derivative() working for the "
               "DQ_DifferentialDriveRobot()? "<<map_return(resultdr)<<std::endl;

    //------------Test for DQ_HolonomicBase()------------------------//
    auto base = std::make_shared<DQ_HolonomicBase>(DQ_HolonomicBase());
    bool result = check_pose_jacobian_derivative(base,iterations, T, threshold);
    std::cout<<"is pose_jacobian_derivative() working for the "
               "DQ_HolonomicBase()?        "<<map_return(result)<<std::endl;

    //------------Test for DQ_SerialManipulatorDH()------------------------//
    auto kuka = std::make_shared<DQ_SerialManipulatorDH>(KukaLw4Robot::kinematics());
    bool resultdh = check_pose_jacobian_derivative(kuka,iterations, T, threshold);
    std::cout<<"is pose_jacobian_derivative() working for the "
               "DQ_SerialManipulatorDH()?  "<<map_return(resultdh)<<std::endl;


    //------------Test for DQ_SerialManipulatorMDH()------------------------//
    auto franka = std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::kinematics());
    bool resultmdh = check_pose_jacobian_derivative(franka,iterations, T, threshold);
    std::cout<<"is pose_jacobian_derivative() working for the "
               "DQ_SerialManipulatorMDH()? "<<map_return(resultmdh)<<std::endl;


    return 0;
}


/**
 * @brief numerical_differentiation() returns the numerical differentiation of a
 *           vector of matrices using the four-point central difference method.
 * @param std::vector<Eigen::MatrixXd> J
 * @param double T
 * @return std::vector<MatrixXd> J_dot
 */
std::vector<MatrixXd> numerical_differentiation(const std::vector<Eigen::MatrixXd>& J,
                                                const double& T)
{
    int s = J.size();
    std::vector<MatrixXd> J_dot(s, MatrixXd::Zero(J[0].rows(), J[0].cols()));
    for(int i=2; i<s-2;i++)
    {
        J_dot[i] = ((J[i-2]-8*J[i-1]+8*J[i+1]-J[i+2])/(12*T));
    }
    return J_dot;
}


/**
 * @brief check_pose_jacobian_derivative() checks the pose_jacobian_derivative method.
 * @param std::shared_ptr<DQ_Kinematics> robot
 * @param int iterations
 * @param double T
 * @param double threshold
 * @return bool accurate_computation
 */
bool check_pose_jacobian_derivative(const std::shared_ptr<DQ_Kinematics>& robot,
                                    int& iterations,
                                    double& T,
                                    double& threshold)
{
    bool accurate_computation = true;
    int njoints    = robot->get_dim_configuration_space();
    VectorXd q     = VectorXd::Zero(njoints);
    VectorXd q_dot = VectorXd::Zero(njoints);

    std::vector<Eigen::MatrixXd> J;
    std::vector<Eigen::MatrixXd> J_dot;
    std::vector<Eigen::MatrixXd> numerical_J_dot;

    double t = 0;
    double w = 2*pi;

    for(int i=0;i<iterations;i++)
    {
        t = i*T;
        double theta = sin(w*t);
        double theta_dot = w*cos(w*t);

        for(int j=0;j<njoints;j++)
        {
            q(j) = theta;
            q_dot(j) = theta_dot;
        }
        J.push_back(robot->pose_jacobian(q));
        J_dot.push_back(robot->pose_jacobian_derivative(q, q_dot));
    }
    numerical_J_dot = numerical_differentiation(J, T);

    for(int i=0;i<iterations;i++)
    {
       auto numerical_error = J_dot[i] - numerical_J_dot[i];
       double     max_coeff = numerical_error.maxCoeff();
       //std::cout<<"Error: "<<numerical_error.maxCoeff()<<std::endl;
       if (i>2 && i<iterations-2) //discard the first two and last two values.
       {
           if(std::abs(max_coeff) > threshold)
           {
               //throw std::runtime_error(std::string("Wrong pose Jacobian derivative computation"));
               accurate_computation = false;
           }
       }

    }
    return accurate_computation;
}

/**
 * @brief map_return() maps a bool to string.
 * @param bool value
 * @return std::string
 */
std::string map_return(const bool& value)
{
    std::string msg;
    if (value == true)
    {
        msg = "Yes!";
    }else
    {
        msg = "No.";
    }
    return msg;
}
