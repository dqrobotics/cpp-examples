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



- Instructions:
    1. Open the DQ_Robotics_lab.ttt scene (https://github.com/dqrobotics/coppeliasim-scenes)
      in CoppeliaSim before running this code.
*/

#include <iostream>
#include <dqrobotics/DQ.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>


using namespace DQ_robotics;
using namespace Eigen;

int main()
{
    DQ_CoppeliaSimInterfaceZMQ cs{};

    try{
        // Establishes the connection with CoppeliaSim.
        cs.connect("localhost", 23000, 1000);

        // Set the stepping mode.
        // Check more in https://manual.coppeliarobotics.com/en/simulation.htm
        cs.set_stepping_mode(true);

        cs.start_simulation();

        // Define some parameters to compute varying-time trajectories
        double a{1};
        double freq{0.1};
        double time_simulation_step{0.05};

        for (size_t i=0;i<300;i++)
        {
            double t = i*time_simulation_step;

            // Define a varying-time position based on the Lemniscate of Bernoulli
            DQ p = (a*cos(t)/(1 + pow(sin(t),2)))*i_ + (a*sin(t)*cos(t)/(1 + pow(sin(t),2)))*j_;

            // Define a varying-time orientation
            DQ r = cos(2*pi*freq*t) + k_*sin(2*pi*freq*t);

            // Built a varying-time unit dual quaternion
            DQ x = r + 0.5*E_*p*r;

            // Set the object pose in CoppeliaSim
            cs.set_object_pose("/coffee_drone", x);

            // Read the pose of an object in CoppeliaSim to set the pose of
            // another object with a constant offset.
            DQ xread = cs.get_object_pose("/coffee_drone");
            DQ xoffset = 1 + 0.5*E_*(0.5*i_);
            DQ xnew = xread*xoffset;
            cs.set_object_pose("/Frame_x", xnew);


            // Set the target position of the first joint of the UMIRobot arm
            VectorXd target_position = (VectorXd(1) <<sin(2*pi*freq*t)).finished();

            // Get the UMIRobot joint names
            std::vector<std::string> umi_joints = cs.get_jointnames_from_object("UMIRobot");

            // Set the joint target position of the first joint
            cs.set_joint_target_positions({umi_joints.at(0)}, target_position);

            // Get the UR5 joint names
            std::vector<std::string> ur5_joints = cs.get_jointnames_from_object("UR5");

            // Set the target position of the third joint of the UR5 robot
            cs.set_joint_target_positions({ur5_joints.at(2)}, target_position);

            // Get the UR5 joint names
            std::vector<std::string> franka_joints = cs.get_jointnames_from_object("Franka");
            //std::vector<std::string> franka_joint = {"Franka/joint"};
            // Set the target velocity of the first joint of the Franka Emika Panda
            VectorXd target_velocity = (VectorXd(1) <<2*pi*freq*cos(2*pi*freq*t)).finished();
            cs.set_joint_target_velocities({franka_joints.at(0)}, target_velocity);

            // Set the torque of the Dummy Robot
            VectorXd force = (VectorXd(1) <<sin(2*pi*freq*t)).finished();
            std::vector<std::string> dummy_joint = {"Revolute_joint"};
            cs.set_joint_target_forces(dummy_joint, force);

            // Set the target velocities of the Pioneer wheels
            std::vector<std::string> pioneer_rjoint = {"PioneerP3DX/rightMotor"};
            std::vector<std::string> pioneer_ljoint = {"PioneerP3DX/leftMotor"};
            cs.set_joint_target_velocities(pioneer_rjoint, (VectorXd(1) << 0.1).finished());
            cs.set_joint_target_velocities(pioneer_ljoint, (VectorXd(1) << 0.2).finished());

            // Trigger a simulation step in CoppeliaSim
            cs.trigger_next_simulation_step();

            auto force_read = cs.get_joint_forces(dummy_joint );
            std::cout<<force<<"  , "<<force_read<<std::endl;
        }
        cs.stop_simulation();

    } catch (std::exception& e) {

        std::cout<<e.what()<<std::endl;
        cs.stop_simulation();
    }
}
