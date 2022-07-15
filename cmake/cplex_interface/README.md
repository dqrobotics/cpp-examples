## Additional requirements

1. Install [CPLEX.](https://www.ibm.com/products/ilog-cplex-optimization-studio)
2. Install the interface between DQ Robotics and CPLEX:

```shell
sudo apt-get install libdqrobotics-interface-cplex
```

3. Set your CPLEX library PATH on the CMakeLists.txt. If your CPLEX library is located in /opt/ibm/ILOG it will be found automatically. Otherwise, your must set your CPLEX library manually. To do so, change the line 

```CMAKE
set(AUTO_CPLEX_PATH true) 
```
to 

```CMAKE
set(AUTO_CPLEX_PATH false)
```

Then, add your CPLEX directory:

```CMAKE
if (NOT AUTO_CPLEX_PATH) # Add manually your CPLEX library path:
    set( CPLEX_PATH /my_cplex_directory)   
```

4. Build and enjoy!

![cplex](https://user-images.githubusercontent.com/23158313/179157323-01619074-bdf7-477f-9d45-7bef582df9f5.gif)
