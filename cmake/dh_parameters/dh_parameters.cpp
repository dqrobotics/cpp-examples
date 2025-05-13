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
    std::vector<DQ_ParameterDH> parameters =
        {DQ_ParameterDH::THETA, DQ_ParameterDH::D, DQ_ParameterDH::A, DQ_ParameterDH::ALPHA};

    std::vector<std::string> string_parameters =
        {"THETA", "D", "A", "ALPHA"};

    std::vector<const char*> char_parameters =
        {"THETA", "D", "A", "ALPHA"};

    // Test get_parameter
    for (int i=0;i<robot->get_dim_configuration_space();i++)
        for(int j=0;j<=3;j++)
            assert(dh_matrix(j,i) == robot->get_parameter(parameters.at(j), i));

    // Test get_parameters
    for (int i=0;i<robot->get_dim_configuration_space();i++)
        for(int j=0;j<=3;j++)
            assert(dh_matrix.row(j).transpose() == robot->get_parameters(parameters.at(j)));

    // Set the new parameters
    for(int j=0;j<=3;j++)
        robot->set_parameters(string_parameters.at(j), dh_matrix.row(j)/10);

    // Test if the new parameters were set correctly using get_parameter
    for (int i=0;i<robot->get_dim_configuration_space();i++)
        for(int j=0;j<=3;j++)
            assert(dh_matrix(j,i)/10 == robot->get_parameter(string_parameters.at(j), i));

    // Test if the new parameters were set correctly using get_parameters
    for(int j=0;j<=3;j++)
        assert(dh_matrix.row(j).transpose()/10  == robot->get_parameters(char_parameters.at(j)));

    std::cout<<msg + ": setters and getters of the DH parameters are working as expected!"<<std::endl;
}
