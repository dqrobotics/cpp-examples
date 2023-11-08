/**
(C) Copyright 2022-2023 DQ Robotics Developers

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
- Murilo M. Marinho (murilo@g.ecc.u-tokyo.ac.jp)
    - initial implementation
- Quentin Lin (qlin1806@g.ecc.u-tokyo.ac.jp)
    - added check for equality constraints
*/

#include <iostream>

#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/KukaLw4Robot.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <dqrobotics/utils/DQ_Constants.h>

using namespace DQ_robotics;

/**
 * @brief Test solver
 */
void test_solver(void) {
    DQ_QPOASESSolver qpoases_solver;
    DQ_SerialManipulatorDH robot = KukaLw4Robot::kinematics();

    DQ_ClassicQPController classic_qp_controller(&robot,&qpoases_solver);
    classic_qp_controller.set_control_objective(ControlObjective::Pose);
    classic_qp_controller.set_gain(0.01);
    classic_qp_controller.set_damping(0.01);

    VectorXd theta_init(7);
    theta_init << 0., pi/4., 0., 0., pi/4., 0., 0.;

    VectorXd theta_xd(7);
    theta_xd << 0., pi/2., 0., 0., pi/2., 0., 0.;
    DQ xd = robot.fkm(theta_xd);

    VectorXd qd = classic_qp_controller.compute_setpoint_control_signal(theta_init,vec8(xd));

    std::cout << "q_dot is " << qd.transpose() << std::endl;
}

/**
 * @brief Check if two vectors are equal within a tolerance
 * @param a Vector 1
 * @param b Vector 2
 * @param tolerance default to 1e-3
 * @return true if pass
 */
bool check_if_vectors_equal(const Eigen::VectorXd &a, const Eigen::VectorXd &b, double tolerance = 1e-3) {
    if (a.size() != b.size()) {
        return false; // Vectors have different dimensions
    }
    for (int i = 0; i < a.size(); ++i) {
        if (std::abs(a(i) - b(i)) > tolerance) {
            std::cout << "CHECK::index: " << i << " exceed error: " << std::abs(a(i) - b(i)) << std::endl;
            return false; // Elements at (i) differ by more than the tolerance
        }
    }
    return true;
}

/**
 * @brief Test equality constraints
 */
void test_equality(void) {
    DQ_QPOASESSolver qpoases_solver;
    auto tolerance = qpoases_solver.get_equality_constraints_tolerance();
    tolerance = qpoases_solver.get_equality_constraints_tolerance();
    std::cout << "current equality tolerance: " << tolerance << std::endl;
    qpoases_solver.set_equality_constraints_tolerance(tolerance);

    for (int problem_size = 5; problem_size <= 10; problem_size += 1) {
        qpoases_solver = DQ_QPOASESSolver();

        MatrixXd H = MatrixXd::Identity(problem_size, problem_size);
        VectorXd f = VectorXd::Ones(problem_size);
        std::cout << "Checking Problem Size: " << problem_size << std::endl;
        for (int iter = 0; iter < 10; iter += 1) {
            srand(iter);
            VectorXd a_diag = VectorXd::Random(problem_size);
            MatrixXd Aeq = a_diag.asDiagonal();
            VectorXd beq = VectorXd::Random(problem_size);
            auto out = qpoases_solver.solve_quadratic_program(H, f, MatrixXd(), VectorXd(), Aeq, beq);
            // just plain tolerance is too tight for the check
            if (!check_if_vectors_equal(beq, Aeq * out, tolerance+DQ_threshold)) {
                std::cout << "Solver Failed equality check on a_diag: " << a_diag.transpose() <<
                          std::endl << "beq: " << beq.transpose() <<
                          std::endl << "Solution: " << out.transpose() <<
                          std::endl << "check " << (Aeq * out).transpose() << std::endl;
                throw std::runtime_error("solver equality check failed");
            }
        }
    }
}


int main(void) {
    test_solver();
    test_equality();

    return 0;
}


