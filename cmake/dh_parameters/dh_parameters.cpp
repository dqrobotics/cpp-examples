/**
(C) Copyright 2011-2025 DQ Robotics Developers

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
- Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
*/

#include <iostream>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
using namespace DQ_robotics;

/**
 * @brief perform_tests This function tests the following methods: get_parameter, get_parameters,
 *                      set_parameter, and set_parameters in the DQ_SerialManipulatorDH and
 *                      DQ_SerialManipulatorMDH classes.
 * @param robot    A smart pointer of a DQ_SerialManipulatorDH or DQ_SerialManipulatorMDH robot.
 * @param dh_matrix The DH matrix used to create the smart pointer of the robot.
 * @param msg The desired message to be displayed.
 */
template<typename T>
void perform_tests(const std::shared_ptr<T>& robot, const MatrixXd& dh_matrix, const std::string& msg);


int main()
{
    Matrix<double,5,7> dh_matrix(5,7);
    dh_matrix <<11, 12, 13, 14, 15, 16, 17, // theta
        21, 22, 23, 24, 25, 26, 27, // d
        31, 32, 33, 34, 35, 36, 37, // a
        41, 42, 43, 44, 45, 46, 47, // alpha
        0,  0,  0,  0,  0,  0,  0;

    auto dh_robot  = std::make_shared<DQ_SerialManipulatorDH>(dh_matrix);
    auto mdh_robot = std::make_shared<DQ_SerialManipulatorMDH>(dh_matrix);

    perform_tests(dh_robot,  dh_matrix, "DQ_SerialManipulatorDH");
    perform_tests(mdh_robot, dh_matrix, "DQ_SerialManipulatorMDH");

    return 0;
}



template<typename T>
void perform_tests(const std::shared_ptr<T>& robot, const MatrixXd& dh_matrix, const std::string& msg)
{
    for (int i=0;i<robot->get_dim_configuration_space();i++)
    {
        if (dh_matrix(0,i) != robot->get_parameter(DQ_SerialManipulator::DQ_ParameterDH::THETA, i))
            throw std::runtime_error(msg + " Error in get_parameter THETA");
        if (dh_matrix(1,i) != robot->get_parameter(DQ_SerialManipulator::DQ_ParameterDH::D, i))
            throw std::runtime_error(msg +" Error in get_parameter D");
        if (dh_matrix(2,i) != robot->get_parameter(DQ_SerialManipulator::DQ_ParameterDH::A, i))
            throw std::runtime_error(msg + " Error in get_parameter A");
        if (dh_matrix(3,i) != robot->get_parameter(DQ_SerialManipulator::DQ_ParameterDH::ALPHA, i))
            throw std::runtime_error(msg + " Error in get_parameter ALPHA");
    }

    if (dh_matrix.row(0).transpose() != robot->get_parameters(DQ_SerialManipulator::DQ_ParameterDH::THETA))
        throw std::runtime_error(msg + " Error in get_parameters THETA");
    if (dh_matrix.row(1).transpose() != robot->get_parameters(DQ_SerialManipulator::DQ_ParameterDH::D))
        throw std::runtime_error(msg + " Error in get_parameters D");
    if (dh_matrix.row(2).transpose() != robot->get_parameters(DQ_SerialManipulator::DQ_ParameterDH::A))
        throw std::runtime_error(msg + " Error in get_parameters A");
    if (dh_matrix.row(3).transpose() != robot->get_parameters(DQ_SerialManipulator::DQ_ParameterDH::ALPHA))
        throw std::runtime_error(msg + " Error in get_parameters ALPHA");

    robot->set_parameters(DQ_SerialManipulator::DQ_ParameterDH::THETA, dh_matrix.row(0)/10);
    robot->set_parameters(DQ_SerialManipulator::DQ_ParameterDH::D, dh_matrix.row(1)/10);
    robot->set_parameters(DQ_SerialManipulator::DQ_ParameterDH::A, dh_matrix.row(2)/10);
    robot->set_parameters(DQ_SerialManipulator::DQ_ParameterDH::ALPHA, dh_matrix.row(3)/10);

    for (int i=0;i<robot->get_dim_configuration_space();i++)
    {
        if (dh_matrix(0,i)/10 != robot->get_parameter(DQ_SerialManipulator::DQ_ParameterDH::THETA, i))
            throw std::runtime_error(msg + " Error in set_parameter THETA");
        if (dh_matrix(1,i)/10 != robot->get_parameter(DQ_SerialManipulator::DQ_ParameterDH::D, i))
            throw std::runtime_error(msg + " Error in set_parameter D");
        if (dh_matrix(2,i)/10 != robot->get_parameter(DQ_SerialManipulator::DQ_ParameterDH::A, i))
            throw std::runtime_error(msg + " Error in set_parameter A");
        if (dh_matrix(3,i)/10 != robot->get_parameter(DQ_SerialManipulator::DQ_ParameterDH::ALPHA, i))
            throw std::runtime_error(msg + "Error in set_parameter ALPHA");
    }

    if (dh_matrix.row(0).transpose()/10 != robot->get_parameters(DQ_SerialManipulator::DQ_ParameterDH::THETA))
        throw std::runtime_error(msg + " Error in get_parameters THETA");
    if (dh_matrix.row(1).transpose()/10 != robot->get_parameters(DQ_SerialManipulator::DQ_ParameterDH::D))
        throw std::runtime_error(msg + " Error in get_parameters D");
    if (dh_matrix.row(2).transpose()/10 != robot->get_parameters(DQ_SerialManipulator::DQ_ParameterDH::A))
        throw std::runtime_error(msg + " Error in get_parameters A");
    if (dh_matrix.row(3).transpose()/10 != robot->get_parameters(DQ_SerialManipulator::DQ_ParameterDH::ALPHA))
        throw std::runtime_error(msg + " Error in get_parameters ALPHA");

    std::cout<<msg + ": setters and getters of the DH parameters are working as expected!"<<std::endl;
}
