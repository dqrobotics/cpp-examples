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
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDenso.h>


using namespace DQ_robotics;

/**
 * @brief perform_tests This function tests the following methods: set_joint_type, set_joint_types,
 *                      get_joint_type, and get_joint_types.
 * @param robot  A serial manipulator robot.
 * @param msg    A message to be displayed.
 * @param is_denso Flag used to indicate if the robot is an object of the DQ_SerialManipulatorDenso class.
 */
template<typename T>
void perform_tests(const std::shared_ptr<T>& robot,
                   const std::string& msg,
                   const bool& is_denso = false
                   );


int main()
{
    DQ_JointType R = DQ_JointType::REVOLUTE;

    Matrix<double,5,7> dh_matrix(5,7);
    dh_matrix <<11, 12, 13, 14, 15, 16, 17, // theta
        21, 22, 23, 24, 25, 26, 27, // d
        31, 32, 33, 34, 35, 36, 37, // a
        41, 42, 43, 44, 45, 46, 47, // alpha
        R,  R,  R,  R,  R,  R,  R;

    auto dh_robot = std::make_shared<DQ_SerialManipulatorDH>(dh_matrix);
    auto mdh_robot = std::make_shared<DQ_SerialManipulatorMDH>(dh_matrix);
    auto denso_robot = std::make_shared<DQ_SerialManipulatorDenso>(MatrixXd::Ones(6,6));

    perform_tests(dh_robot,  "DQ_SerialManipulatorDH");
    perform_tests(mdh_robot, "DQ_SerialManipulatorMDH");
    perform_tests(mdh_robot, "DQ_SerialManipulatorDenso", true);

    return 0;
}

template<typename T>
void perform_tests(const std::shared_ptr<T>& robot, const std::string& msg, const bool& is_denso){

    // Test get_joint_type
    for (int i=0;i<robot->get_dim_configuration_space();i++)
             assert(robot->get_joint_type(i) == DQ_JointType::REVOLUTE);

    // Test set_joint_type and get_joint_types
    DQ_JointType target_joint_type = DQ_JointType::PRISMATIC;
    if (is_denso)
        target_joint_type = DQ_JointType::REVOLUTE;
    for (int i=0;i<robot->get_dim_configuration_space();i++)
        robot->set_joint_type(target_joint_type, i);

    DQ_JointType expected_joint_type = DQ_JointType::PRISMATIC;
    if (is_denso)
        expected_joint_type = DQ_JointType::REVOLUTE;
    assert(robot->get_joint_types() ==
           std::vector<DQ_JointType>(robot->get_dim_configuration_space(), expected_joint_type));

    // Test set_joint_types
    robot->set_joint_types( std::vector<DQ_JointType>(robot->get_dim_configuration_space(), DQ_JointType::REVOLUTE));
    assert(robot->get_joint_types() ==
           std::vector<DQ_JointType>(robot->get_dim_configuration_space(), DQ_JointType::REVOLUTE));

    std::cout<<msg + ": setters and getters of the joint types are working as expected!"<<std::endl;
}
