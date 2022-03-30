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

#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/KukaLw4Robot.h>
#include <dqrobotics/solvers/DQ_CPLEXSolver.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <dqrobotics/utils/DQ_Constants.h>

using namespace DQ_robotics;

int main(void)
{
    DQ_CPLEXSolver cplex_solver;
    DQ_SerialManipulatorDH robot = KukaLw4Robot::kinematics();

    DQ_ClassicQPController classic_qp_controller(&robot,&cplex_solver);
    classic_qp_controller.set_control_objective(ControlObjective::Pose);
    classic_qp_controller.set_gain(0.01);
    classic_qp_controller.set_damping(0.01);

    VectorXd theta_init(7);
    theta_init << 0.,pi/4.,0.,0.,pi/4.,0.,0.;

    VectorXd theta_xd(7);
    theta_xd << 0.,pi/2.,0.,0.,pi/2.,0.,0.;
    DQ xd = robot.fkm(theta_xd);

    classic_qp_controller.compute_setpoint_control_signal(theta_init,vec8(xd));

    return 0;
}
