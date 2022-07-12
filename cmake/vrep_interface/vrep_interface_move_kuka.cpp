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
#include <string>
#include <thread>

#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/KukaLw4Robot.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/utils/DQ_Constants.h>

int main(void)
{
    DQ_VrepInterface vi;

    vi.connect(19997,100,10);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();

    std::vector<std::string> joint_names = {"LBR_iiwa_14_R820_joint1",
                                            "LBR_iiwa_14_R820_joint2",
                                            "LBR_iiwa_14_R820_joint3",
                                            "LBR_iiwa_14_R820_joint4",
                                            "LBR_iiwa_14_R820_joint5",
                                            "LBR_iiwa_14_R820_joint6",
                                            "LBR_iiwa_14_R820_joint7"};

    DQ_SerialManipulatorDH robot = KukaLw4Robot::kinematics();

    VectorXd theta_d(7);
    theta_d << 0.,pi/2.,0.,pi/2.,0.,pi/2.,0.;
    DQ xd = robot.fkm(theta_d);

    VectorXd theta;

    VectorXd e(8);
    e(0)=1.0;

    std::cout << "Starting control loop..." << std::endl;
    while(e.norm()>0.05)
    {
        theta      = vi.get_joint_positions(joint_names);
        DQ x       = robot.fkm(theta);
        e          = vec8(x-xd);
        MatrixXd J = robot.pose_jacobian(theta);

        VectorXd u = -0.01*pinv(J)*e;
        theta      = theta+u;
        vi.set_joint_target_positions(joint_names,theta);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Control finished..." << std::endl;

    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();

    vi.disconnect();


    return 0;
}
