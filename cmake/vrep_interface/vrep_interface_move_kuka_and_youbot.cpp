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

#include <memory>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_CPLEXSolver.h>
#include <dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.h>
#include <dqrobotics/interfaces/vrep/robots/YouBotVrepRobot.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/utils/DQ_Constants.h>

struct SimulationParameters
{
    bool move_manipulator;
    bool first_iteration;
    double wd;
    double wn;
    double total_time;
    double dispz;
    double tc;
    DQ rcircle;
};

DQ get_plane_from_vrep(DQ_VrepInterface& vrep_interface,
                       const std::string& plane_name,
                       const DQ& normal);

DQ get_line_from_vrep(DQ_VrepInterface& vrep_interface,
                      const std::string& line_name,
                      const DQ& direction);

void compute_lbr4p_reference(DQ& xd,
                             DQ& xd_dot,
                             const DQ_SerialManipulator& lbr4p,
                             const SimulationParameters& simulation_parameters,
                             const DQ& x0,
                             const double &t);

void compute_youbot_reference(DQ &youbot_xd, DQ &youbot_ff, SimulationParameters& simulation_parameters,
                              const DQ_ClassicQPController& controller,
                              const DQ& lbr4p_xd,
                              const DQ& lbr4p_ff);

void compute_constraints(MatrixXd& Jconstraint,
                          VectorXd& bconstraint,
                          DQ_WholeBody& youbot,

                          const VectorXd& youbot_q,
                          const DQ& plane,
                          const DQ& cylinder1,
                          const DQ& cylinder2);

int main(void)
{
    DQ_VrepInterface vi;
    try
    {
        SimulationParameters simulation_parameters;
        simulation_parameters.first_iteration = true;
        simulation_parameters.tc = 0;
        simulation_parameters.rcircle = DQ(1);
        simulation_parameters.move_manipulator = true;
        simulation_parameters.wd = 0.5;
        simulation_parameters.wn = 0.1;
        simulation_parameters.total_time = 40.0;
        simulation_parameters.dispz = 0.1;

        if(!vi.connect(19998,100,5))
        {
            throw std::runtime_error("Unable to connect to vrep!");
        }
        std::cout << "Starting V-REP simulation..." << std::endl;
        vi.start_simulation();

        //Initialize VREP robots
        LBR4pVrepRobot lbr4p_vreprobot("LBR4p",&vi);
        YouBotVrepRobot youbot_vreprobot("youBot",&vi);

        //Load DQ Robotics Kinematics
        DQ_SerialManipulator lbr4p = lbr4p_vreprobot.kinematics();
        DQ_WholeBody youbot = youbot_vreprobot.kinematics();

        //Initialize controllers
        DQ_PseudoinverseController lbr4p_controller(&lbr4p);
        lbr4p_controller.set_control_objective(ControlObjective::Pose);
        lbr4p_controller.set_gain(10.0);

        DQ_CPLEXSolver solver;
        DQ_ClassicQPController youbot_controller(&youbot, &solver);
        youbot_controller.set_control_objective(ControlObjective::Pose);
        youbot_controller.set_gain(10.0);
        youbot_controller.set_damping(0.01);

        //Control loop parameters and initial configuration
        double sampling_time = 0.05;

        //Get initial robot information
        VectorXd lbr4p_q(7); lbr4p_q << 0, 1.7453e-01, 0, 1.5708, 0, 2.6273e-01, 0;
        lbr4p_vreprobot.send_q_to_vrep(lbr4p_q);
        DQ lbr4p_x0 = conj(lbr4p.get_reference_frame())*lbr4p.fkm(lbr4p_q);
        VectorXd youbot_q = youbot_vreprobot.get_q_from_vrep();

        DQ lbr4p_xd;
        DQ lbr4p_ff;

        DQ youbot_xd;
        DQ youbot_ff;

        for(double t=0;t<simulation_parameters.total_time;t+=sampling_time)
        {
            //Get Obstacles from VREP
            DQ plane = get_plane_from_vrep(vi, "ObstaclePlane", k_);
            DQ cylinder1 = get_line_from_vrep(vi, "ObstacleCylinder1", k_);
            DQ cylinder2 = get_line_from_vrep(vi, "ObstacleCylinder2", k_);

            // Set reference for the manipulator and the mobile manipulator
            compute_lbr4p_reference(lbr4p_xd, lbr4p_ff,
                                    lbr4p, simulation_parameters, lbr4p_x0, t);

            compute_youbot_reference(youbot_xd, youbot_ff,
                                     simulation_parameters, youbot_controller, lbr4p_xd, lbr4p_ff);

            //Compute control signal for the arm
            VectorXd lbr4p_u = lbr4p_controller.compute_tracking_control_signal(lbr4p_q,
                                                                                vec8(lbr4p_xd),
                                                                                vec8(lbr4p_ff));

            //Computer control signal for the youbot
            MatrixXd Jconstraint; VectorXd bconstraint;
            compute_constraints(Jconstraint, bconstraint,
                                 youbot, youbot_q,
                                 plane, cylinder1, cylinder2);
            youbot_controller.set_inequality_constraint(-Jconstraint,1*bconstraint);

            VectorXd youbot_u = youbot_controller.compute_tracking_control_signal(
                        youbot_q,
                        vec8(youbot_xd),
                        vec8(youbot_ff));


            lbr4p_q = lbr4p_q + lbr4p_u*sampling_time;
            youbot_q = youbot_q + youbot_u*sampling_time;


            //Send desired values
            lbr4p_vreprobot.send_q_to_vrep(lbr4p_q);
            youbot_vreprobot.send_q_to_vrep(youbot_q);

            std::this_thread::sleep_for(std::chrono::milliseconds(int(sampling_time*1000.0)));
        }


        vi.stop_simulation();
        vi.disconnect();

    } catch (std::runtime_error& e)
    {
        std::cout << "There was an error connecting to V-REP, please check that it is open and that the Kuka Robot is in the scene." << std::endl;
        std::cout << e.what() << std::endl;
        vi.stop_simulation();
        vi.disconnect();
        vi.disconnect_all();
    }
    catch(...)
    {
        vi.disconnect_all();
    }

    return 0;
}

DQ get_plane_from_vrep(DQ_VrepInterface& vrep_interface, const std::string& plane_name, const DQ& normal)
{
    DQ plane_object_pose = vrep_interface.get_object_pose(plane_name);
    DQ p = translation(plane_object_pose);
    DQ r = rotation(plane_object_pose);
    DQ n = Ad(r, normal);
    DQ d = dot(p, n);
    return n + E_*d;
}

DQ get_line_from_vrep(DQ_VrepInterface& vrep_interface, const std::string& line_name, const DQ& direction)
{
    DQ line_object_pose = vrep_interface.get_object_pose(line_name);
    DQ p = translation(line_object_pose);
    DQ r = rotation(line_object_pose);
    DQ l = Ad(r, direction);
    DQ m = cross(p, l);
    return l + E_*m;
}

void compute_lbr4p_reference(DQ& xd, DQ& xd_dot, const DQ_SerialManipulator& lbr4p, const SimulationParameters& simulation_parameters, const DQ& x0, const double& t)
{
    const double& dispz = simulation_parameters.dispz;
    const double& wd = simulation_parameters.wd;
    const double& wn = simulation_parameters.wn;

    double phi = (pi/2.0)*sin(wn*t);
    DQ r = cos(phi/2.0) + k_*sin(phi/2.0);

    DQ z = dispz*cos(wd*t)*k_;
    DQ p = 1 + E_*0.5*z;

    //Return pose
    xd = r*x0*p;

    //Return time derivative
    double phidot = (pi/2.0)*cos(wn*t)*wn;
    DQ rdot = 0.5*(-sin(phi/2.0) + k_*cos(phi/2.0))*phidot;
    DQ pdot = -E_*0.5*dispz*wd*sin(wd*t)*k_;
    xd_dot = rdot*x0*p + r*x0*pdot;

    // The trajectory,including the feedforward term, has been calculated with
    // respect to the manipulator base. Therefore, we need to calculate them
    // with respect to the global reference frame.
    xd = lbr4p.get_reference_frame()*xd;
    xd_dot = lbr4p.get_reference_frame()*xd_dot;
}

void compute_youbot_reference(DQ& youbot_xd,
                              DQ& youbot_ff,
                              SimulationParameters& simulation_parameters,
                              const DQ_ClassicQPController& controller,
                              const DQ& lbr4p_xd,
                              const DQ& lbr4p_ff)
{
    const double circle_radius = 0.1;
    const DQ tcircle = 1 + E_ * 0.5 * circle_radius * j_;

    // Youbot trajectory
    // Those are the trajectory components to track the whiteboard
    youbot_xd = lbr4p_xd * (1 + 0.5*E_*0.015*k_) * j_;
    youbot_ff = lbr4p_ff * (1 + 0.5*E_*0.015*k_) * j_;
    // Now we modify the trajectory in order to draw a circle, but we do it
    // only if the whiteboard pen tip is on the whiteboard surface.
    if(simulation_parameters.first_iteration)
    {
        simulation_parameters.first_iteration = false;
        simulation_parameters.tc = 0;
        simulation_parameters.rcircle = DQ(1);
    }
    else if(controller.get_last_error_signal().norm() < 0.002)
    {
        simulation_parameters.tc += 0.1; // Increment around 0.5 deg.
        simulation_parameters.rcircle = cos(simulation_parameters.tc/2.0) + k_ * sin(simulation_parameters.tc/2.0);
    }
    youbot_xd = youbot_xd * simulation_parameters.rcircle * tcircle;
    youbot_ff = youbot_ff * simulation_parameters.rcircle * tcircle;
}

void compute_constraints(MatrixXd& Jconstraint, VectorXd& bconstraint,
                          DQ_WholeBody& youbot,
                          const VectorXd& youbot_q,
                          const DQ& plane,
                          const DQ& cylinder1,
                          const DQ& cylinder2)
{
    double robot_radius = 0.35;
    double radius_cylinder1 = 0.1;
    double radius_cylinder2 = 0.1;

    DQ_HolonomicBase youbot_base = youbot.get_chain_as_holonomic_base(0);
    DQ youbot_base_pose = youbot_base.raw_fkm(youbot_q);
    MatrixXd Jx = youbot_base.raw_pose_jacobian(youbot_q);
    DQ t = translation(youbot_base_pose);
    MatrixXd base_Jt = youbot.translation_jacobian(Jx,youbot_base_pose);
    MatrixXd Jt(4,8);
    Jt << base_Jt, MatrixXd::Zero(4,5);

    MatrixXd Jdist_plane = youbot.point_to_plane_distance_jacobian(Jt, t, plane);
    double dist_plane = DQ_Geometry::point_to_plane_distance(t, plane) - robot_radius;

    MatrixXd Jdist_cylinder1 = youbot.point_to_line_distance_jacobian(Jt, t, cylinder1);
    double dist_cylinder1 = DQ_Geometry::point_to_line_squared_distance(t, cylinder1) - pow(radius_cylinder1 + robot_radius, 2);

    MatrixXd Jdist_cylinder2 = youbot.point_to_line_distance_jacobian(Jt, t, cylinder2);
    double dist_cylinder2 = DQ_Geometry::point_to_line_squared_distance(t, cylinder2) - pow(radius_cylinder2 + robot_radius, 2);

    Jconstraint.resize(3,8);
    Jconstraint << Jdist_plane, Jdist_cylinder1, Jdist_cylinder2;
    bconstraint.resize(3);
    bconstraint << dist_plane, dist_cylinder1, dist_cylinder2;
}
