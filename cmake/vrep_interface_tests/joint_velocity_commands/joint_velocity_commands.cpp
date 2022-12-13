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
- dqrobotics-interface-vrep

1) Open the CoppeliaSim scene joint_velocity_commands.ttt
2) Be sure that the Lua script attached to the object DQRoboticsApiCommandServer is updated.
   (Updated version: vrep_interface_tests/DQRoboticsApiCommandServer.lua)
3) Compile, run and enjoy!
*/

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>


using namespace Eigen;

int main(void)
{
    DQ_VrepInterface vi;
    vi.connect(19997,100,10);
    vi.set_synchronous(true);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::vector<std::string>jointnames = {"Franka_joint1", "Franka_joint2",
                                           "Franka_joint3", "Franka_joint4",
                                           "Franka_joint5", "Franka_joint6",
                                           "Franka_joint7"};

    // robot definition
    auto robot = std::make_shared<DQ_SerialManipulatorMDH>
            (FrankaEmikaPandaRobot::kinematics());

    //Update the base of the robot from CoppeliaSim
    DQ new_base_robot = (robot->get_base_frame())*vi.get_object_pose("Franka")*(1+0.5*E_*(-0.07*k_));
    robot->set_reference_frame(new_base_robot);


    DQ_PseudoinverseController controller(robot);
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
        vi.set_object_pose("ReferenceFrame", robot->fkm(q));
        VectorXd u = controller.compute_setpoint_control_signal(q, vec4(xdesired.translation()));
        std::cout << "task error: " <<controller.get_last_error_signal().norm()
                  <<" Iteration: "<<i<<std::endl;
        vi.set_joint_target_velocities(jointnames, u);
        vi.trigger_next_simulation_step();
        i++;

    }
    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();
    return 0;
}
