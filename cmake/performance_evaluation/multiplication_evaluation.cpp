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
#include<iomanip>
#include<sstream>
#include<fstream>
#include<vector>
#include<chrono>

#include <dqrobotics/DQ.h>


using namespace DQ_robotics;

const int NUMBER_OF_RANDOM = 1000;
const int NUMBER_OF_RUNS = 1000;

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 12)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

double get_average(const std::vector<double>&);
double get_variance(const std::vector<double>&);

int main(void)
{
    std::vector<double> required_time;
    std::vector<double> variance(NUMBER_OF_RUNS);

    std::ofstream time_out;
    time_out.precision(12);
    std::ofstream variance_out;
    variance_out.precision(12);

    time_out.open("time_out.csv");
    variance_out.open("variance_out.csv");

    for (int j = 0; j < NUMBER_OF_RUNS; j++) {

        //Initialize vectors with random values
        std::vector<DQ> random_dq_a(NUMBER_OF_RANDOM);
        std::vector<DQ> random_dq_b(NUMBER_OF_RANDOM);
        std::vector<DQ> result_dq_c(NUMBER_OF_RANDOM);

        for(unsigned int i=0;i<NUMBER_OF_RANDOM;i++)
        {
            random_dq_a[i] = DQ(VectorXd::Random(8));
            random_dq_b[i] = DQ(VectorXd::Random(8));
        }

        auto start = std::chrono::system_clock::now();
        for(unsigned int i=0;i<NUMBER_OF_RANDOM;i++)
        {
            result_dq_c[i] = random_dq_a[i]*random_dq_b[i];
        }
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;

        required_time.push_back(diff.count()*double(1000.0));
        variance[j] = get_variance(required_time);

        time_out << std::fixed << to_string_with_precision<double>(required_time[j]) << std::endl;
        variance_out << std::fixed << to_string_with_precision<double>(variance[j]) << std::endl;
    }

    time_out.close();
    variance_out.close();

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
