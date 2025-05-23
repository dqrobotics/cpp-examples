/**
(C) Copyright 2011-2025 DQ Robotics Developers

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
- Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)



- Instructions:
    1. Open the DQ_Robotics_lab.ttt scene (https://github.com/dqrobotics/coppeliasim-scenes)
      in CoppeliaSim before running this code.
*/

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>



int main()
{
    DQ_CoppeliaSimInterfaceZMQ cs{};
    try {
        cs.connect();
        cs.start_simulation();
        std::vector<std::string> robotnames = {"UR5", "Franka", "UMIRobot"};
        for(auto& robotname : robotnames)
        {
            cs.get_joint_positions(cs.get_jointnames_from_object(robotname));
            std::cout<<robotname<<" configuration: "
                      <<cs.get_joint_positions(cs.get_jointnames_from_object(robotname)).transpose()<<std::endl;
        }
        cs.stop_simulation();
    }
    catch (const std::runtime_error& e)
    {
        std::cerr<<e.what()<<std::endl;
    }
}
