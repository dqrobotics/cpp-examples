//Contributed by @juanjqo
//https://github.com/dqrobotics/cpp/issues/44

#include <iostream>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialWholeBody.h>
#include <dqrobotics/robots/KukaLw4Robot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <memory>
#include <Eigen/Dense>

using namespace DQ_robotics;

void test_issue_44()
{
    //-----------Robot definition------------------------------------------------------
    auto arm1 = std::make_shared<DQ_SerialManipulatorDH>(KukaLw4Robot::kinematics());
    auto arm2 = std::make_shared<DQ_SerialManipulatorDH>(KukaLw4Robot::kinematics());
    DQ_SerialWholeBody super_arm(std::static_pointer_cast<DQ_Kinematics>(arm1));
    super_arm.add(std::static_pointer_cast<DQ_Kinematics>(arm2));
    //---------------------------------------------------------------------------------
    int n = arm1->get_dim_configuration_space()+arm2->get_dim_configuration_space();
    VectorXd q = VectorXd::Zero(n);

    for (int i=0; i< n; i++)
    {
            std::cout << "x"<<i<<": "<<super_arm.fkm(q,i)<<std::endl;
    }

}

