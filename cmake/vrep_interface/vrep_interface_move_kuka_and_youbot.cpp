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

#include <iostream>
#include <string>
#include <thread>

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep_interface.h>
#include <dqrobotics/interfaces/LBR4pVrepRobot.h>
#include <dqrobotics/interfaces/YouBotVrepRobot.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/utils/DQ_Constants.h>

int main(void)
{
    VrepInterface vi;
    try
    {
        vi.connect(19997,10,10);
        std::cout << "Starting V-REP simulation..." << std::endl;
        vi.start_simulation();

        //Initialize VREP robots
        LBR4pVrepRobot lbr4p_vreprobot("LBR4p",&vi);
        YouBotVrepRobot youbot_vreprobot("youBot",&vi);

        //Load DQ Robotics Kinematics
        DQ_SerialManipulator lbr4p = lbr4p_vreprobot.kinematics();
        DQ_WholeBody youbot = youbot_vreprobot.kinematics();


        vi.stop_simulation();
        vi.disconnect();

    } catch (...)
    {
        std::cout << "There was an error connecting to V-REP, please check that it is open and that the Kuka Robot is in the scene." << std::endl;
        vi.disconnect_all();
    }

    return 0;
}
