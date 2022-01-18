# dq-robotics-cpp-examples
DQ Robotics C++ examples

## Running examples

To try out the examples from this repository, do the following:

1. Install DQRobotics C++11.
Refer to the [website](https://dqrobotics.github.io/).

2. Clone the examples repository

```
git clone https://github.com/dqrobotics/cpp-examples.git --recursive
```

3. Browse to the example folder and compile. For example,

```
cd cmake/performance_evaluation
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

4. Run the example. For example,

```
./performance_evaluation
```

5. If you get something similar to what is shown below, your example ran properly

```
Time to calculate a 7 DOF random vector 1000000 times was 0.0336847 s. Or per time [s]: 3.36847e-08
Time to calculate a 7 DOF Jacobian 1000000 times was 27.3508 s. Or per time [s]: 2.73508e-05
Time to calculate a 7 DOF fkm 1000000 times was 11.6242 s. Or per time [s]: 1.16242e-05
Time to calculate a 7 DOF translation Jacobian (including requirements) 1000000 times was 39.487 s. Or per time [s]: 3.9487e-05
Time to calculate a 7 DOF rotation Jacobian (including requirements) 1000000 times was 27.4134 s. Or per time [s]: 2.74134e-05
Time to create a dual quaternion from a random VectorXd 1000000 times was 0.0671938 s. Or per time [s]: 6.71938e-08
Time to calculate the dual quaternion product 1000000 times was 1.17405 s. Or per time [s]: 1.17405e-06
Time to calculate the dual quaternion sum 1000000 times was 0.166378 s. Or per time [s]: 1.66378e-07
```
