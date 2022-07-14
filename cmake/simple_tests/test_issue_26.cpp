//Contributed by @maripaf
//https://github.com/dqrobotics/cpp/issues/26

#include "test_issue_26.h"

#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <iostream>
#include <cmath>

using namespace DQ_robotics;

void test_issue_26()
{
    // robot definition
    Matrix<double, 5, 7> kuka_mdh;
    kuka_mdh << 0, 0, 0, 0, 0, 0, 0, // theta
               0.3105, 0, 0.4, 0, 0.39, 0, 0, // d
               0, 0, 0, 0, 0, 0, 0, // a
               0, M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, // alpha
               0,      0,       0,       0,      0,      0,       0; // joint_type
    DQ_SerialManipulatorMDH kuka(kuka_mdh);

    int num_dof = kuka.get_dim_configuration_space();
    VectorXd q(num_dof), q_dot(num_dof);
    q << 0., 0.5, 0., -1.2, 0., 0., 0.;
    q_dot << 0, 0, 0, 0, 0, 0, 0;

    Eigen::MatrixXd J = kuka.pose_jacobian(q);
    Eigen::MatrixXd N(8,num_dof);
    DQ x = kuka.fkm(q);

    VectorXd q_desired(num_dof);
    q_desired << 0., 0.5, 0.5, -1.2, 0., 0., 0.;
    DQ xd = kuka.fkm(q_desired);

    DQ x_til = x.conj()*xd;

    double k = 5;
    double sampling_time = 0.005;
    int i = 0;

    // initialize rotation angle
    DQ test;

    while (i < 2000) {

        ++i;

        x = kuka.fkm(q);
        J = kuka.pose_jacobian(q);

        // invariant error
        x_til = x.conj()*xd;

        // check log
        test = log(x_til);
        if (std::isnan(test.q(0))) {
            std::cout << test << std::endl;
            throw std::runtime_error("log is nan!");
        }

        // first order kinematic control
        N = haminus8(xd)*C8()*J;
        q_dot = pinv(N)*(k*vec8(1-x_til));
        q = sampling_time*q_dot + q;

        // error
        std::cout << (1 - x.conj()*xd).vec8().norm() << std::endl;
    }

}
