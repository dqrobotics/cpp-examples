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

#include<iostream>
#include<vector>
#include<chrono>

#include <dqrobotics/DQ.h>

using namespace DQ_robotics;

double get_average(const std::vector<double>&);
double get_variance(const std::vector<double>&);

int main(void)
{
    const int RUN_COUNT = 1000000;

    //Initialize vectors with random values
    std::vector<DQ> random_dq_a(RUN_COUNT);
    std::vector<DQ> random_dq_b(RUN_COUNT);
    std::vector<DQ> result_dq_c(RUN_COUNT);
    std::vector<double> required_time(RUN_COUNT);

    for(unsigned int i=0;i<RUN_COUNT;i++)
    {
        random_dq_a[i] = DQ(VectorXd::Random(8));
        random_dq_b[i] = DQ(VectorXd::Random(8));
    }

    for(unsigned int i=0;i<RUN_COUNT;i++)
    {
        auto start = std::chrono::system_clock::now();
        result_dq_c[i] = random_dq_a[i]*random_dq_b[i];
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        required_time[i]=diff.count()/double(RUN_COUNT);
    }

    std::cout << "Average is: " << get_average(required_time) << std::endl;
    std::cout << "Variance is: " << get_variance(required_time) << std::endl;

    return 0;
}

double get_average(const std::vector<double>& a)
{
    const unsigned long N = a.size();
    double acc = 0;
    for(const double& point : a)
    {
        acc+=point;
    }
    return acc/double(N);
}

double get_variance(const std::vector<double>& a)
{
    const unsigned long N = a.size();
    const double m = get_average(a);

    double acc=0;
    for(const double& point : a)
    {
        acc+=pow(point-m,2);
    }
    return acc/double(N);
}
