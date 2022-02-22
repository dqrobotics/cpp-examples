#include <iostream>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

using namespace DQ_robotics;

void load_dq_serial_manipulator_dh();
void load_dq_serial_manipulator_denso();

int main(void)
{

    load_dq_serial_manipulator_dh();
    load_dq_serial_manipulator_denso();

    return 0;
}

void print_kinematics_info(DQ_Kinematics* kinematics)
{
    std::cout << "	reference_frame = " << kinematics->get_reference_frame() << std::endl;
}

void print_serialmanipulator_info(DQ_SerialManipulator* serial_manipulator)
{
    std::cout << "	lower_q_limit = " << serial_manipulator->get_lower_q_limit().transpose() << std::endl;
    std::cout << "	upper_q_limit = " << serial_manipulator->get_upper_q_limit().transpose() << std::endl;
    std::cout << "	lower_q_dot_limit = " << serial_manipulator->get_lower_q_dot_limit().transpose() << std::endl;
    std::cout << "	upper_q_dot_limit = " << serial_manipulator->get_upper_q_dot_limit().transpose() << std::endl;
    std::cout << "	effector = " << serial_manipulator->get_effector() << std::endl;
}

void load_dq_serial_manipulator_dh()
{
    DQ_SerialManipulatorDH robot_dh = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>("../dq_serial_manipulator_dh.json");

    std::cout << "Loaded DQ_SerialManipulatorDH with" << std::endl;
    std::cout << "	thetas = " << robot_dh.get_thetas().transpose() << std::endl;
    std::cout << "	ds = " << robot_dh.get_ds().transpose() << std::endl;
    std::cout << "	as = " << robot_dh.get_as().transpose() << std::endl;
    std::cout << "	alphas = " << robot_dh.get_alphas().transpose() << std::endl;
    std::cout << "	types = " << robot_dh.get_types().transpose() << std::endl;

    print_kinematics_info(static_cast<DQ_Kinematics*>(&robot_dh));
    print_serialmanipulator_info(static_cast<DQ_SerialManipulator*>(&robot_dh));
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
