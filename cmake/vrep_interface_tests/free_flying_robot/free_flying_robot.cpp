/**
(C) Copyright 2023 DQ Robotics Developers
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


Prerequisites:
- dqrobotics
- dqrobotics-interface-vrep
- dqrobotics-interface-qpoases

Instructions:
1) Open the CoppeliaSim scene free_flying_robot.ttt
2) Compile, run and enjoy!
*/

#include <iostream>
#include <dqrobotics/robot_modeling/DQ_FreeFlyingRobot.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/DQ.h>
#include <thread>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics/utils/DQ_Geometry.h>

using namespace DQ_robotics;
using namespace Eigen;


/**
 * @brief This function computes the VFI constraints required to prevent collisions between the robot and
 *        the environment.
 * @param vi The client object.
 * @param robot The free-flying robot object
 * @param obstacle_names The name of the spheric obstacles in the scene
 * @param safe_distances The vector containing the safe distances.
 * @param robot_pose The unit dual quaternion that represents
 *                   the free-flying robot configuration..
 * @param robot_jacobian The Jacobian of the free-flying robot.
 * @return The desired constraints.
 */
std::tuple<MatrixXd, VectorXd> compute_constraints(const std::shared_ptr<DQ_VrepInterface>& vi,
                                                   const DQ_FreeFlyingRobot& robot,
                                                   const std::vector<std::string>& obstacle_names,
                                                   const std::vector<double>& safe_distances,
                                                   const DQ& robot_pose, const MatrixXd& robot_jacobian);

int main()
{
    auto vi = std::make_shared<DQ_VrepInterface>();
    try
    {
        vi->connect(19997,100,10);
        vi->set_synchronous(true);
        std::cout << "Starting the CoppeliaSim simulation..." << std::endl;
        vi->start_simulation();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto robot = DQ_FreeFlyingRobot();
        double T = 0.01;
        double lambda = 1;
        DQ xd = vi->get_object_pose("xdesired");
        DQ x  = vi->get_object_pose("coffee_drone");
        DQ xerror = x.conj()*xd;

        MatrixXd IM = MatrixXd::Zero(8,6);
        for (auto i=0;i<3;i++){
            IM(i+1,i) = 1.0;
            IM(i+5,i+3) = 1.0;
        }

        auto solver = std::make_shared<DQ_QPOASESSolver>();

        std::vector<std::string> obstacle_names = {"obstacle1", "obstacle2", "obstacle3",
                                                   "obstacle4", "obstacle5", "obstacle6",
                                                   "obstacle7", "obstacle8", "obstacle9",
                                                   "obstacle10"};
        std::vector<double> safe_distances(obstacle_names.size(), 0.25);

        MatrixXd A;
        VectorXd b;
        MatrixXd Aeq;
        VectorXd beq;

        while (log(xerror).vec6().norm() > 0.0001)
        {

            MatrixXd J = robot.pose_jacobian(x)*IM;
            MatrixXd M = pinv(Q8(xerror))*haminus8(xd)*C8()*J;
            MatrixXd H = M.transpose()*M;
            VectorXd yerror = vec6(log(xerror));
            VectorXd f = 2*lambda*yerror.transpose()*M;

            std::tie(A, b) = compute_constraints(vi, robot, obstacle_names, safe_distances,x, J);
            auto uvec = solver->solve_quadratic_program(H, f,  A,  b, Aeq, beq);

            x = exp(T/2*DQ(uvec))*x;
            xerror =  x.conj()*xd;
            std::cout<<"error: "<<yerror.norm()<<std::endl;
            vi->set_object_pose("coffee_drone", x);
            vi->trigger_next_simulation_step();
        }

    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
        vi->stop_simulation();
        vi->disconnect();
        return 0;
    }
    vi->wait_for_simulation_step_to_end();
    vi->stop_simulation();
    vi->disconnect();
    return 0;
}


std::tuple<MatrixXd, VectorXd> compute_constraints(const std::shared_ptr<DQ_VrepInterface>& vi,
                                                   const DQ_FreeFlyingRobot& robot,
                                                   const std::vector<std::string>& obstacle_names,
                                                   const std::vector<double>& safe_distances,
                                                   const DQ& robot_pose, const MatrixXd& robot_jacobian)
{
    DQ t = robot_pose.translation();
    MatrixXd J_t = robot.translation_jacobian(robot_jacobian, robot_pose);
    int n = obstacle_names.size();
    MatrixXd A = MatrixXd::Zero(n+1, 6);
    VectorXd b = VectorXd::Zero(n+1);


    for (auto i=0;i<n;i++)
    {
        DQ obstacle_translation = vi->get_object_pose(obstacle_names[i]).translation();
        A.row(i) = -1*robot.point_to_point_distance_jacobian(J_t, t, obstacle_translation);
        b(i)     = DQ_Geometry::point_to_point_squared_distance(t, obstacle_translation)
                   - std::pow(safe_distances[i],2);
    }
    A.row(n) = -1*robot.point_to_plane_distance_jacobian(J_t, t, k_);
    b(n)     = DQ_Geometry::point_to_plane_distance(t,k_);
    return {A, b};
}
