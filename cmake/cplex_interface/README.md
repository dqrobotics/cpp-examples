## Additional requirements

1. Install [CPLEX.](https://www.ibm.com/products/ilog-cplex-optimization-studio)
2. Install the interface between DQ Robotics and CPLEX:

```shell
sudo apt-get install libdqrobotics-interface-cplex
```

3. Set your CPLEX library PATH on the CMakeLists.txt. If your CPLEX library is located in /opt/ibm/ILOG will be founded automatically. Otherwise, your must set your CPLEX library manually. To do so, change the line set(AUTO_CPLEX_PATH true) to set(AUTO_CPLEX_PATH false). Then, add your CPLEX directory:


```txt
if (NOT AUTO_CPLEX_PATH) # Add manually your CPLEX library path:
    set( CPLEX_PATH /opt/ibm/ILOG/CPLEX_Studio201 )
```
