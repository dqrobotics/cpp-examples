//Contributed by @juanjqo
//https://github.com/dqrobotics/python/issues/18

#include "test_issue_py_18.h"

#include <dqrobotics/DQ.h>
#include <iostream>
#include <cmath>

using namespace DQ_robotics;

void test_issue_py_18()
{
    std::cout << "Issue 18 Python" << std::endl;
    DQ r = DQ(0.5068154, -0.8591303, 0.0555768, -0.0440969);
    r = normalize(r);
    std::cout << r << std::endl;
    std::cout << r.rotation_axis() << std::endl;
    std::cout << r.rotation_angle() << std::endl;
}
