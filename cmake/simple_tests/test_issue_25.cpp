//Contributed by @maripaf
//https://github.com/dqrobotics/cpp/issues/25

#include "test_issue_25.h"

#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <iostream>
#include <cmath>
#include <exception>

using namespace DQ_robotics;

void test_issue_25()
{
    // robot definition
    Matrix<double, 5, 7> kuka_mdh;
    kuka_mdh << 0, 0, 0, 0, 0, 0, 0, // theta
            0.3105, 0, 0.4, 0, 0.39, 0, 0, // d
            0, 0, 0, 0, 0, 0, 0, // a
            0, M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, M_PI_2, -M_PI_2,// alpha
            0,      0,       0,       0,      0,      0,       0;//joint_types
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
    double phi = 0.;

    while (i < 2000) {

        ++i;

        x = kuka.fkm(q);
        J = kuka.pose_jacobian(q);

        // invariant error
        x_til = x.conj()*xd;

        // check rotation angle
        phi = x_til.rotation_angle();
        if (std::isnan(phi)) {
            std::cout << "phi is not real" << std::endl;
            std::cout << "phi = " << phi << std::endl;
            std::cout << "is unit: " << is_unit(x_til) << std::endl;
            std::cout << "real: " << Re(P(x_til)) << std::endl;
            throw std::runtime_error("phi is not real!");
        }

        // first order kinematic control
        N = haminus8(xd)*C8()*J;
        q_dot = pinv(N)*(k*vec8(1-x_til));
        q = sampling_time*q_dot + q;

        // error
        std::cout << (1 - x.conj()*xd).vec8().norm() << std::endl;
    }

}
