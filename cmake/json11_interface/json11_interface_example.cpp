#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

using namespace DQ_robotics;

int main(void)
{
    DQ_SerialManipulatorDH robot_dh = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>("dq_serial_manipulator_dh.json");
    return 0;
}
