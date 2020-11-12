//Contributed by @marcos-pereira
//https://github.com/dqrobotics/python/issues/22

#include "test_issue_py_22.h"

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h>
#include <iostream>
#include <cmath>

using namespace DQ_robotics;

void test_issue_py_22()
{
    std::cout << "Issue 22 Python" << std::endl;
    VectorXd q(3);
    q << 0.12, 0.0, 0.0;
    DQ_DifferentialDriveRobot diff_robot(1,1);
    MatrixXd J = diff_robot.pose_jacobian(q);
    std::cout << "J" << std::endl;
}
