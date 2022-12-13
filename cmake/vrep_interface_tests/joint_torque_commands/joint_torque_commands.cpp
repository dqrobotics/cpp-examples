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



Instructions:

This example performs a torque control in the joint configuration space using
a serial robot manipulator (FrankaEmikaPanda) from CoppeliaSim. In this specific example, the gravitational
acceleration is not taken into account. Because of that, the gravity on the CoppeliaSim scene is disabled.


Instructions:
Prerequisites:
- dqrobotics
- dqrobotics-interface-vrep

1) Open the CoppeliaSim scene joint_torque_commands_no_gravity.ttt
2) Be sure that the Lua script attached to the object DQRoboticsApiCommandServer is updated.
   (Updated version: vrep_interface_tests/DQRoboticsApiCommandServer.lua)
3) Compile, run and enjoy!

Note:
This example saves in the build folder two CSV files containing the torque references and the torque readings.
You do not need to create manually both CVS files in the build folder, since they are created automatically
when this example is executed.

You can see the results using the following Matlab code:

//------------------------Matlab code-----------------------------
clear all
close all
clc

torques_ref = readmatrix('list_torques_ref.csv')';
torques_read = readmatrix('list_torques_read.csv')';

h1 = figure;
set(h1, 'DefaultTextFontSize', 10);
set(h1, 'DefaultAxesFontSize', 10); % [pt]
set(h1, 'DefaultAxesFontName', 'mwa_cmr10');
set(h1, 'DefaultTextFontName', 'mwa_cmr10');
set(h1, 'Units', 'centimeters');
pos = get(h1, 'Position');
pos(3) = 2*20; % Select the width of the figure in [cm] 17
pos(4) = 2*10; % Select the height of the figure in [cm] 6
set(h1, 'Position', pos);
set(h1, 'PaperType', 'a4letter');
set(h1,'PaperPositionMode','auto')
set(h1, 'Renderer', 'Painters');
a = 2;
b = 4;
w = 4;
fontsize = 20;
for i=1:7
subplot(a,b,i);
plot(torques_ref(i,:),'b', 'LineWidth',w);
hold on
plot(torques_read(i,:),':r', 'LineWidth',3);
set(gca, 'FontSize', fontsize );
fig = gcf;
fig.Color = [1 1 1];
box('off');
%title('$\tau$', 'Interpreter','latex', 'Rotation', 0)
title(['\tau_',num2str(i)])
end
legend('Reference', 'measurement')
//------------------------------------------------------------------------------------



*/

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>
#include <fstream>

using namespace Eigen;




int main(void)
{
    DQ_VrepInterface vi;
    vi.connect(19997,100,10);
    vi.set_synchronous(true);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int iterations = 10000;

    //------------------- Robot definition--------------------------
    //---------- Franka Emika Panda serial manipulator
    DQ_SerialManipulatorMDH franka = FrankaEmikaPandaRobot::kinematics();

    //Update the base of the robot from CoppeliaSim
    DQ new_base_robot = (franka.get_base_frame())*vi.get_object_pose("Franka")*(1+0.5*E_*(-0.07*k_));
    franka.set_reference_frame(new_base_robot);
    //---------------------------------------------------------------

    std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2",
                                            "Franka_joint3", "Franka_joint4",
                                            "Franka_joint5", "Franka_joint6",
                                            "Franka_joint7"};

    VectorXd vec_torques(7);

    double Kp = 0.01; //4.5
    double Kv = 3*sqrt(Kp);
    VectorXd qd = VectorXd(7);
    qd <<-0.70, -0.10, 1.66, -2.34,0.40, 1.26, 0.070;
    vi.set_object_pose("DesiredFrame", franka.fkm(qd));

    VectorXd g_qd = VectorXd(7);
    Matrix<double, 7,3> list_torques;

    std::ofstream list_torques_ref;
    list_torques_ref.open("list_torques_ref.csv");

    std::ofstream list_torques_read;
    list_torques_read.open("list_torques_read.csv");

    for (int i=0;i<iterations;i++)
    {
        VectorXd q = vi.get_joint_positions(jointnames);
        VectorXd qerror = qd-q;
        VectorXd q_dot = vi.get_joint_velocities(jointnames);
        VectorXd qerror_dot = -q_dot;

        vi.set_object_pose("ReferenceFrame", franka.fkm(q));

        vec_torques = Kp*qerror + Kv*qerror_dot ;
        vi.set_joint_torques(jointnames, vec_torques);
        vi.trigger_next_simulation_step();

        VectorXd vec_torques_read = vi.get_joint_torques(jointnames);

        list_torques.col(0) = vec_torques;
        list_torques.col(1) = vec_torques_read;
        list_torques.col(2) = list_torques.col(0)-list_torques.col(1);


        list_torques_ref <<vec_torques(0)<<","<<vec_torques(1)<<","<<vec_torques(2)<<
                           ","<<vec_torques(3)<<","<<vec_torques(4)<<","<<vec_torques(5)<<
                           ","<<vec_torques(6)<<","<<'\n';


        list_torques_read <<vec_torques_read(0)<<","<<vec_torques_read(1)<<","<<vec_torques_read(2)<<
                           ","<<vec_torques_read(3)<<","<<vec_torques_read(4)<<","<<vec_torques_read(5)<<
                           ","<<vec_torques_read(6)<<","<<'\n';


        std::cout<< "torques ref:   torques read:    error:"<<std::endl;
        std::cout<<list_torques<<std::endl;

        std::cout<< "  "<<std::endl;
        std::cout<< "Applying torques..."<<iterations-i<<std::endl;
        std::cout<< "Error..."<<qerror.norm()<<std::endl;
    }
    list_torques_ref.close();
    list_torques_read.close();

    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();
    return 0;
}
