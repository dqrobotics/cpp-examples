/**
(C) Copyright 2022 DQ Robotics Developers
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
- Murilo Marques Marinho (murilo@g.ecc.u-tokyo.ac.jp)

Instructions:
Prerequisites:
- dqrobotics
- dqrobotics-vrep-interface

1) Open the CoppeliaSim scene joint_velocity_commands.ttt
2) Be sure that the Lua script attached to the object DQRoboticsApiCommandServer is updated.
   (Updated version: vrep_interface_tests/DQRoboticsApiCommandServer.lua)
3) Compile, run and enjoy!
*/

#include <iostream>
#include <string>
#include <thread>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <Eigen/Dense>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <dqrobotics/robot_control/DQ_KinematicController.h>

using namespace Eigen;

int main(void)
{
    DQ_VrepInterface vi;
    vi.connect(19997,100,10);
    vi.set_synchronous(true);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // robot definition
    Matrix<double, 5, 7> robot_dh;
    robot_dh << 0, 0, 0, 0, 0, 0, 0,                           // theta
                0.333, 0, 3.16e-1, 0, 3.84e-1, 0, 1.07e-1,     // d
                0, 0, 8.25e-2, -8.25e-2, 0, 8.8e-2, 0,         // a
                -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, M_PI_2, M_PI_2, 0, // alpha
                 0,0,0,0,0,0,0;
    DQ_SerialManipulatorDH franka(robot_dh);
    DQ robot_base = 1 + E_ * 0.5 * DQ(0, 0.0413, 0, 0);
    franka.set_base_frame(robot_base);
    franka.set_reference_frame(robot_base);

    std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2",
                                            "Franka_joint3", "Franka_joint4",
                                            "Franka_joint5", "Franka_joint6",
                                            "Franka_joint7"};

    DQ_PseudoinverseController controller(&franka);
    controller.set_gain(0.5);
    controller.set_damping(0.05);
    controller.set_control_objective(DQ_robotics::Translation);
    controller.set_stability_threshold(0.00001);

    DQ xdesired = 1 + E_*0.5*DQ(0, 0.2, 0.3, 0.3);
    vi.set_object_pose("DesiredFrame", xdesired);

    int i=0;
    while (not controller.system_reached_stable_region())
    {

        VectorXd q = vi.get_joint_positions(jointnames);
        vi.set_object_pose("ReferenceFrame", franka.fkm(q));
        VectorXd u = controller.compute_setpoint_control_signal(q, vec4(xdesired.translation()));
        std::cout << "error: " <<controller.get_last_error_signal().norm()<<" Iteration: "<<i<<std::endl;
        std::cout<< "Is stable?: "<<controller.system_reached_stable_region()<<std::endl;

        vi.set_joint_target_velocities(jointnames, u);
        vi.trigger_next_simulation_step();

        std::cout<< "q_dot: "<<vi.get_joint_velocities(jointnames)<<std::endl;
        i++;

    }
    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();
    return 0;
}
