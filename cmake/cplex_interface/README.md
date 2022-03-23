## Additional requirements

1. Install [CPLEX.](https://www.ibm.com/products/ilog-cplex-optimization-studio)
2. Install the interface between DQ Robotics and CPLEX:

```shell
sudo apt-get install libdqrobotics-interface-cplex
```

3. Set your CPLEX library PATH on the CMakeLists.txt. 

```txt
set( CPLEX_PATH /opt/ibm/ILOG/CPLEX_Studio_your_version)
```
