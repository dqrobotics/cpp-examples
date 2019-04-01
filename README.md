# dq-robotics-cpp-examples
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
cmake ..
make
```

3. Run the example. For example,

```
./performance_evaluation
```
## Opening examples as QTCreator Projects

1. Download and install QT from https://www.qt.io/download or your perferred source. Installing a gcc64 compatible kit, such as Qt 5.12 is also recommended

2. File >> Open File or Project 

3. Choose the example's CMakeLists.txt

4. Configure the QTCreator kit and the source code should show up and be compilable.
