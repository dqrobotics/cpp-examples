
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
Prerequisites:
- dqrobotics
- dqrobotics-interface-vrep
- dqrobotics-interface-qpoases

1) Open the CoppeliaSim scene test_dynamic_conic_constraint.ttt
2) Compile, run and enjoy!
*/

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <thread>


using namespace Eigen;
using namespace DQ_robotics;

int main()
{
    const bool USE_RESIDUAL = true;

    DQ_VrepInterface vi;
    vi.connect(19997,100,10);
    vi.set_synchronous(true);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int iterations = 15000; //+7.0000e-02
    std::vector<std::string> linknames ={"Franka_link2_resp","Franka_link3_resp","Franka_link4_resp",
                                             "Franka_link5_resp","Franka_link6_resp","Franka_link7_resp",
                                             "Franka_link8_resp"};
    std::vector<std::string> jointnames ={"Franka_joint1", "Franka_joint2","Franka_joint3",
                                              "Franka_joint4", "Franka_joint5", "Franka_joint6",
                                              "Franka_joint7"};


    //------------------- Robot definition--------------------------
    auto robot = std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::kinematics());
    //Update the base of the robot from CoppeliaSim
    DQ new_base_robot = (robot->get_base_frame())*vi.get_object_pose("Franka")*(1+0.5*E_*(-0.07*k_));
    robot->set_reference_frame(new_base_robot);
    //--------------------------------------------------------------
    auto solver = std::make_shared<DQ_QPOASESSolver>(DQ_QPOASESSolver());
    //--------------------------------------------------------------
    //------------------- Controller definition---------------------
    DQ_ClassicQPController controller(robot, solver);

    controller.set_gain(0.5);
    controller.set_damping(0.1);
    controller.set_control_objective(DQ_robotics::Translation);
    controller.set_stability_threshold(0.001);
    //--------------------------------------------------------------
    const double vfi_gain = 0.5;
    const double safe_angle = 15*(pi/180);
    const double T = 0.005;
    const double w = 0.2;
    const double alpha = 20*(pi/180);
    double t = 0;


    for (int i=0;i<iterations;i++)
    {
        t = i*T;

        double phi_t = alpha*sin(w*t);
        double phi_t_dot = alpha*w*cos(w*t);
        DQ r_dyn = cos(phi_t/2) + i_*sin(phi_t/2); //
        DQ r_dyn_dot = (-sin(phi_t/2)  + i_*cos(phi_t/2))*(phi_t_dot/2);

        VectorXd q = vi.get_joint_positions(jointnames);
        DQ x = robot->fkm(q);
        MatrixXd J = robot->pose_jacobian(q);

        // DQ workspace_pose is a unit dual quaternion that represent the position and orientation of a frame rigidly attached
        // to the dynamic workspace line.
        DQ workspace_line_pose  = r_dyn + 0.5*E_*x.translation()*r_dyn;

        //----Dynamic Workspace line-------------------------
        DQ workspace_attached_direction = k_;
        DQ workspace_line = r_dyn * workspace_attached_direction *(r_dyn.conj());
        VectorXd vec_workspace_line_dot = (haminus4(k_*r_dyn.conj())+ hamiplus4(r_dyn*k_)*C4())*vec4(r_dyn_dot);

        //----Robot Workspace line-------------------------
        const DQ robot_attached_direction = -k_;
        DQ robot_line = (x.P())*(robot_attached_direction)*(x.P().conj());

        //----line-to-line-angle-Jacobian-------------------------//////
        //MatrixXd Jl = DQ_Kinematics::line_jacobian(J, x, robot_attached_direction);
        MatrixXd Jphi =  DQ_Kinematics::
                         line_to_line_angle_jacobian(DQ_Kinematics::line_jacobian(J, x, robot_attached_direction),
                                                     robot_attached_direction,
                                                     workspace_line);

        double residual_phi = DQ_Kinematics::line_to_line_angle_residual(robot_line,
                                                                         workspace_line,
                                                                         DQ(vec_workspace_line_dot));

        double phi = DQ_Geometry::line_to_line_angle(robot_line, workspace_line);
        double f = 2-2*cos(phi);
        double fsafe = 2-2*cos(safe_angle);
        double ferror = f-fsafe;
        if (-1*ferror < 0)
        {
            std::cout<<"-----RLINE_TO_LINE_ANGLE Constraint violated!!!!!!!!-------------------------"<<std::endl;
        }

        if (USE_RESIDUAL == false)
        {
         residual_phi = 0;
        }
        VectorXd b(1);
        b(0) = vfi_gain*ferror + residual_phi;

        vi.set_object_pose("x", x);
        vi.set_object_pose("cone", workspace_line_pose);
        vi.set_joint_position("Revolute_joint_master", safe_angle);

        DQ xdesired = x;
        vi.set_object_pose("xd", xdesired);
        controller.set_inequality_constraint(Jphi, -b);
        VectorXd u = controller.compute_setpoint_control_signal(q, vec4(xdesired.translation()));
        std::cout << "Ending simulation at : " <<iterations-i<<std::endl;
        vi.set_joint_target_velocities(jointnames, u);
        vi.trigger_next_simulation_step();

    }
    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();
    return 0;
}

