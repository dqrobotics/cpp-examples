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

#include "test_issue_24.h"
#include "test_issue_25.h"
#include "test_issue_26.h"
#include "test_issue_44.h"
#include "test_issue_py_18.h"
#include "test_issue_py_22.h"

int main()
{
    //Evaluate DQ constructors
    //https://github.com/dqrobotics/cpp/issues/24
    test_issue_24();

    //Evaluate rotation_angle()
    //https://github.com/dqrobotics/cpp/issues/25
    test_issue_25();

    //Evaluate log()
    //https://github.com/dqrobotics/cpp/issues/26
    test_issue_26();

    //Evaluate DQ_SerialWholeBody.fkm
    //https://github.com/dqrobotics/cpp/issues/44
    test_issue_44();

    //Evaluate to_string()
    //https://github.com/dqrobotics/python/issues/18
    test_issue_py_18();

    //Evaluate DQ_DifferentialDriveRobot::pose_jacobian()
    //https://github.com/dqrobotics/python/issues/22
    test_issue_py_22();

    return 0;
}


