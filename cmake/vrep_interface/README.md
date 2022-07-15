## Additional requirements

1. Download [V-REP (CoppeliaSim).](https://www.coppeliarobotics.com/)
2. Install the interface between DQ Robotics and V-REP:

```shell
sudo apt-get install libdqrobotics-interface-vrep
```
3. Install the [qpoases library](https://github.com/dqrobotics/cpp-interface-qpoases)

4. Install the [cpp-interface-qpoases](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/cpp.html#interface-packages)

```shell
sudo apt-get install libdqrobotics-interface-qpoases
```

5. Open the correspoding VREP scene

- To run the example _vrep_interface_move_kuka_and_youbot_ download and open the [vrep_scene_felt_pen_official_scene.ttt](https://github.com/dqrobotics/matlab-examples/tree/master/vrep/simulation-ram-paper) VREP scene.

- To run the example _vrep_interface_move_kuka_ open a new scene on VREP using the _KUKA LBR iiwa 14 R820_. Don't forget to delete the code on the script to prevent undesired movements.


6. Build the example and enjoy!

- _vrep_interface_move_kuka_and_youbot_

![vrep_interface](https://user-images.githubusercontent.com/23158313/179151228-34ce7655-acd7-4068-b302-e39755f4fc82.gif)

- _vrep_interface_move_kuka_
![kuka](https://user-images.githubusercontent.com/23158313/179156283-1e4fb3c5-9222-4b6a-91f4-0b3e01b23644.gif)
