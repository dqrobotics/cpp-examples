#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

using namespace DQ_robotics;

int main(void)
{
    DQ_SerialManipulatorDH robot_dh = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>("../dq_serial_manipulator_dh.json");
    
    std::cout << "Loaded DQ_SerialManipulatorDH with" << std::endl;
    std::cout << "	thetas = " << robot_dh.get_thetas().transpose() << std::endl;
    std::cout << "	ds = " << robot_dh.get_ds().transpose() << std::endl;
    std::cout << "	as = " << robot_dh.get_as().transpose() << std::endl;
    std::cout << "	alphas = " << robot_dh.get_alphas().transpose() << std::endl;
    std::cout << "	types = " << robot_dh.get_types().transpose() << std::endl;
    std::cout << "	lower_q_limit = " << robot_dh.get_lower_q_limit().transpose() << std::endl;
    std::cout << "	upper_q_limit = " << robot_dh.get_upper_q_limit().transpose() << std::endl;
    std::cout << "	lower_q_dot_limit = " << robot_dh.get_lower_q_dot_limit().transpose() << std::endl;
    std::cout << "	upper_q_dot_limit = " << robot_dh.get_upper_q_dot_limit().transpose() << std::endl;
    std::cout << "	effector = " << robot_dh.get_effector() << std::endl;
    std::cout << "	reference_frame = " << robot_dh.get_reference_frame() << std::endl;
    
    return 0;
}
