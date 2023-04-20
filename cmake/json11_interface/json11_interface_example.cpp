#include <iostream>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <dqrobotics/utils/DQ_Math.h>

using namespace DQ_robotics;

void load_dq_serial_manipulator_dh();
void load_dq_serial_manipulator_denso();

int main(void)
{

    load_dq_serial_manipulator_dh();
    load_dq_serial_manipulator_denso();

    return 0;
}

static VectorXd deg2rad_with_mask(const VectorXd& v, const VectorXi& mask)
{
    if(v.size()!=mask.size())
        throw std::runtime_error("DQ_JsonReader::deg2rad_with_mask::Invalid mask size.");

    VectorXd v_with_mask(v);
    for(int i=0;i<v.size();i++)
    {
        if(mask(i)==0)
            v_with_mask(i)=deg2rad(v(i));
    }
    return v_with_mask;
}

void test_dq_serial_manipulator(DQ_SerialManipulator* dqsm, VectorXi angle_mask)
{
    //"lower_q_limit":[-1 ,-2 ,-3 ,-4 ,-5 ,-6 ,-7 ,-8 ,-9 ],
    VectorXd lower_q_limit_check(9); lower_q_limit_check<<-1 ,-2 ,-3 ,-4 ,-5 ,-6 ,-7 ,-8 ,-9;
    assert((dqsm->get_lower_q_limit()==deg2rad_with_mask(lower_q_limit_check.head(dqsm->get_dim_configuration_space()),angle_mask)));
    //"upper_q_limit":[9 ,8 ,7 ,6 ,5 ,4 ,3 ,2 ,1 ],
    VectorXd upper_q_limit_check(9); upper_q_limit_check<<9 ,8 ,7 ,6 ,5 ,4 ,3 ,2 ,1;
    assert((dqsm->get_upper_q_limit()==deg2rad_with_mask(upper_q_limit_check.head(dqsm->get_dim_configuration_space()),angle_mask)));
    //"lower_q_dot_limit":[-9, -8, -7, -6, -5, -4, -3, -2, -1],
    VectorXd lower_q_dot_limit_check(9); lower_q_dot_limit_check<<-9, -8, -7, -6, -5, -4, -3, -2, -1;
    assert((dqsm->get_lower_q_dot_limit()==deg2rad_with_mask(lower_q_dot_limit_check.head(dqsm->get_dim_configuration_space()),angle_mask)));
    //"upper_q_dot_limit":[1, 2, 3, 4, 5, 6, 7, 8, 9],
    VectorXd upper_q_dot_limit_check(9); upper_q_dot_limit_check<<1, 2, 3, 4, 5, 6, 7, 8, 9;
    assert((dqsm->get_upper_q_dot_limit()==deg2rad_with_mask(upper_q_dot_limit_check.head(dqsm->get_dim_configuration_space()),angle_mask)));
    //"effector":[0.0,1.0,0.0,0.0],
    DQ effector_check = i_;
    assert(dqsm->get_effector()==effector_check);
    //"reference_frame":[0.0,0.0,1.0,0.0]
    DQ reference_frame_check = j_;
    assert(dqsm->get_reference_frame()==reference_frame_check);
}

void load_dq_serial_manipulator_dh()
{
    std::cout << "Loading DQ_SerialManipulatorDH..." << std::endl;

    DQ_SerialManipulatorDH robot_dh = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>("../dq_serial_manipulator_dh.json");

    //"theta":[-90, 90, -90, 0, 90, -90,  0,  0,  0],
    VectorXd thetas_check(9); thetas_check << -90, 90, -90, 0, 90, -90,  0,  0,  0;
    assert(robot_dh.get_thetas()==deg2rad(thetas_check));
    //"d":[0.345, 0, 0, 0.255, 0, 0, 0.1198375,  0.27, 0],
    VectorXd d_check(9); d_check << 0.345, 0, 0, 0.255, 0, 0, 0.1198375,  0.27, 0;
    assert(robot_dh.get_ds()==d_check);
    //"a":[0.0, 0.255, 0.01, 0, 0, 0, 0, 0, 0],
    VectorXd a_check(9); a_check << 0.0, 0.255, 0.01, 0, 0, 0, 0, 0, 0;
    assert(robot_dh.get_as()==a_check);
    //"alpha":[90, 0, -90, 90, 90, 0, -90, 90, 0],
    VectorXd alpha_check(9); alpha_check << 90, 0, -90, 90, 90, 0, -90, 90, 0;
    assert(robot_dh.get_alphas()==deg2rad(alpha_check));
    //"types":[0, 0, 1, 0, 0, 1, 0, 0, 0],
    VectorXd types_check(9); types_check << 0, 0, 1, 0, 0, 1, 0, 0, 0;
    assert(robot_dh.get_types()==types_check);

    test_dq_serial_manipulator(static_cast<DQ_SerialManipulator*>(&robot_dh), types_check.cast<int>());

    std::cout << "Loaded and checked DQ_SerialManipulatorDH." << std::endl;
}

void load_dq_serial_manipulator_denso()
{
    std::cout << "Loading DQ_SerialManipulatorDenso..." << std::endl;

    DQ_SerialManipulatorDenso robot_denso = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDenso>("../dq_serial_manipulator_denso.json");

    //"a":    [0,     0,      -0.009935, 0.000182,  -0.00002,  0],
    VectorXd a_check(6); a_check << 0,     0,      -0.009935, 0.000182,  -0.00002,  0;
    //"b":    [0,     -0.2501, 0,      0,      0,      0],
    VectorXd b_check(6); b_check << 0,     -0.2501, 0,      0,      0,      0;
    //"d":    [0.345,   0,      0,      0.25509, -0.00017,  0.069967],
    VectorXd d_check(6); d_check << 0.345,   0,      0,      0.25509, -0.00017,  0.069967;
    //"alpha":[-89.94,0.039,  90.018, -89.95, 89.986, 0],
    VectorXd alpha_check(6); alpha_check << -89.94,0.039,  90.018, -89.95, 89.986, 0;
    assert(robot_denso.get_alphas()==deg2rad(alpha_check));
    //"beta": [0,     -0.009, 0,      0,      0,      0],
    VectorXd beta_check(6); beta_check << 0,     -0.009, 0,      0,      0,      0;
    assert(robot_denso.get_betas()==deg2rad(beta_check));
    //"gamma":[0,     0.017,  -0.015, -0.115, -0.013, 0],
    VectorXd gamma_check(6); gamma_check << 0,     0.017,  -0.015, -0.115, -0.013, 0;
    assert(robot_denso.get_gammas()==deg2rad(gamma_check));

    test_dq_serial_manipulator(static_cast<DQ_SerialManipulator*>(&robot_denso), VectorXi::Zero(6));

    std::cout << "Loaded and checked DQ_SerialManipulatorDenso." << std::endl;
}
