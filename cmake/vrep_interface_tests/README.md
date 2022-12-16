## Additional requirements

1. Download [V-REP (CoppeliaSim).](https://www.coppeliarobotics.com/)
2. Install the interface between DQ Robotics and V-REP:

```shell
sudo apt-get install libdqrobotics-interface-vrep
```
3. Check out the [documentation](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/python.html) about the interface with CoppeliaSim. Some known issues (**not related to DQ Robotics**) are addressed.

![req](https://user-images.githubusercontent.com/23158313/158897518-a61bf680-9836-4aa8-9f66-95c05fd35f90.gif)

## Compile, run and enjoy!

Example: Consider the example vrep_interface/joint_torque_commands.
1. Open the joint_torque_commands_no_gravity.ttt VREP scene. 
2. Compile
 
```shell
cd cmake/vrep_interface_tests/joint_torque_commands
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

4. Run
 
```shell
./joint_torque_commands 
```


![ezgif-1-c04be41431](https://user-images.githubusercontent.com/23158313/158247464-945cb68b-d1ef-4d57-a6f9-601d92f91aaf.gif)
