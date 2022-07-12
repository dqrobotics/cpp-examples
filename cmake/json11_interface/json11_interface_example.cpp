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

static void print_kinematics_info(DQ_Kinematics* kinematics)
{
    std::cout << "	reference_frame = " << kinematics->get_reference_frame() << std::endl;
}

static void print_serialmanipulator_info(DQ_SerialManipulator* serial_manipulator)
{
    std::cout << "	lower_q_limit = " << serial_manipulator->get_lower_q_limit().transpose() << std::endl;
    std::cout << "	upper_q_limit = " << serial_manipulator->get_upper_q_limit().transpose() << std::endl;
    std::cout << "	lower_q_dot_limit = " << serial_manipulator->get_lower_q_dot_limit().transpose() << std::endl;
    std::cout << "	upper_q_dot_limit = " << serial_manipulator->get_upper_q_dot_limit().transpose() << std::endl;
    std::cout << "	effector = " << serial_manipulator->get_effector() << std::endl;
}

static VectorXd deg2rad_with_mask(const VectorXd& v, const VectorXi& mask=VectorXi())
{
    if(mask.size() == 0)
        return v;
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

void load_dq_serial_manipulator_dh()
{
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
//"lower_q_limit":[-1 ,-2 ,-3 ,-4 ,-5 ,-6 ,-7 ,-8 ,-9 ],
    VectorXd lower_q_limit_check(9); lower_q_limit_check<<-1 ,-2 ,-3 ,-4 ,-5 ,-6 ,-7 ,-8 ,-9;
    assert((robot_dh.get_lower_q_limit()==deg2rad_with_mask(lower_q_limit_check,types_check.cast<int>())));
//"upper_q_limit":[9 ,8 ,7 ,6 ,5 ,4 ,3 ,2 ,1 ],
    VectorXd upper_q_limit_check(9); upper_q_limit_check<<9 ,8 ,7 ,6 ,5 ,4 ,3 ,2 ,1;
    assert((robot_dh.get_upper_q_limit()==deg2rad_with_mask(upper_q_limit_check,types_check.cast<int>())));
//"lower_q_dot_limit":[-9, -8, -7, -6, -5, -4, -3, -2, -1],
    VectorXd lower_q_dot_limit_check(9); lower_q_dot_limit_check<<-9, -8, -7, -6, -5, -4, -3, -2, -1;
    assert((robot_dh.get_lower_q_dot_limit()==deg2rad_with_mask(lower_q_dot_limit_check,types_check.cast<int>())));
//"upper_q_dot_limit":[1, 2, 3, 4, 5, 6, 7, 8, 9],
    VectorXd upper_q_dot_limit_check(9); upper_q_dot_limit_check<<1, 2, 3, 4, 5, 6, 7, 8, 9;
    assert((robot_dh.get_upper_q_dot_limit()==deg2rad_with_mask(upper_q_dot_limit_check,types_check.cast<int>())));
//"effector":[0.0,1.0,0.0,0.0],
    DQ effector_check = i_;
    assert(robot_dh.get_effector()==effector_check);
//"reference_frame":[0.0,0.0,1.0,0.0]
    DQ reference_frame_check = j_;
    assert(robot_dh.get_reference_frame()==reference_frame_check);

    std::cout << "Loaded and checked DQ_SerialManipulatorDH." << std::endl;
}

void load_dq_serial_manipulator_denso()
{
    DQ_SerialManipulatorDenso robot_denso = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDenso>("../dq_serial_manipulator_denso.json");

    std::cout << "Loaded DQ_SerialManipulatorDH with" << std::endl;
    std::cout << "	alphas = " << robot_denso.get_alphas().transpose() << std::endl;
    std::cout << "	betas = " << robot_denso.get_betas().transpose() << std::endl;
    std::cout << "	thetas = " << robot_denso.get_gammas().transpose() << std::endl;
    std::cout << "	as = " << robot_denso.get_as().transpose() << std::endl;
    std::cout << "	bs = " << robot_denso.get_bs().transpose() << std::endl;
    std::cout << "	ds = " << robot_denso.get_ds().transpose() << std::endl;

    print_kinematics_info(static_cast<DQ_Kinematics*>(&robot_denso));
    print_serialmanipulator_info(static_cast<DQ_SerialManipulator*>(&robot_denso));
}
