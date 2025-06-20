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

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/utils/DQ_Math.h>
#include <memory>


/**
 * @brief move_to_configuration This method performs a kinematic control at the joint-space level.
 * @param target_configuration The desired robot configuration.
 * @param coppeliasim_robot The coppeliasim robot.
 * @param proportional_gain The proportional controller. This parameter affects the convergence rate.
 * @param error_tolerance The error norm tolerance.
 */
void move_to_configuration(const VectorXd& target_configuration,
                           const std::shared_ptr<DQ_CoppeliaSimRobotZMQ>& coppeliasim_robot,
                           const double& proportional_gain,
                           const double& error_tolerance);


int main()
{
    auto cs = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
    try {
        cs->connect();
        auto panda = std::make_shared<FrankaEmikaPandaCoppeliaSimZMQRobot>("Franka", cs);
        cs->start_simulation();

        VectorXd target_pos_1 = DQ_robotics::deg2rad((VectorXd(7)<<90,90,135,-45,90,180,0).finished());
        VectorXd target_pos_2 = DQ_robotics::deg2rad((VectorXd(7)<<-90,90,135,-45,90,180,0).finished());
        VectorXd target_pos_3 = DQ_robotics::deg2rad((VectorXd(7)<<0,0,0,-90,0,90,0).finished());

        move_to_configuration(target_pos_1, panda, 1, 0.01);
        move_to_configuration(target_pos_2, panda, 1, 0.01);
        move_to_configuration(target_pos_3, panda, 1, 0.01);

        cs->stop_simulation();
    }
    catch (const std::runtime_error& e)
    {
        std::cerr<<e.what()<<std::endl;
    }
}


void move_to_configuration(const VectorXd& target_configuration,
                           const std::shared_ptr<DQ_CoppeliaSimRobotZMQ>& coppeliasim_robot,
                           const double& proportional_gain,
                           const double& error_tolerance)
{
    const VectorXd& qd = target_configuration;
    VectorXd error = VectorXd::Ones(target_configuration.size());
    while (error.norm() > error_tolerance)
    {
        VectorXd q = coppeliasim_robot->get_configuration();
        error = q-qd;
        VectorXd u = -proportional_gain*error;
        coppeliasim_robot->set_target_configuration_velocities(u);
    }

}
