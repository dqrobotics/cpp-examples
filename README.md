# dq-robotics-cpp-examples [![Build Status](https://travis-ci.com/dqrobotics/cpp-examples.svg?branch=master)](https://travis-ci.com/dqrobotics/cpp-examples)
DQ Robotics C++ examples

## Running examples

To try out the examples from this repository, do the following:

1. Clone the repository

```
git clone https://github.com/dqrobotics/cpp-examples.git --recursive
```

2. Browse to the example folder and compile. For example,

```
cd cmake/performance_evaluation
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

3. Run the example. For example,

```
./performance_evaluation
```

4. If you get something similar to what is shown below, your example ran properly

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
## Opening examples as QTCreator Projects

1. Download and install QT from https://www.qt.io/download or your favorite source. Installing a gcc64 compatible kit, such as Qt 5.12 is also recommended

2. File >> Open File or Project 

3. Choose the example's CMakeLists.txt

4. Configure the QTCreator kit and the source code should show up and be compilable.
