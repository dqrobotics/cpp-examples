#!/bin/bash

cd cmake/vrep_interface_tests

declare -a vrep_examples_array=("extracting_inertial_parameters" "joint_torque_commands" "joint_velocity_commands")
 
for vrep_example in ${vrep_examples_array[@]}; do
    cd $vrep_example
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    
    make
    # Guaranteeing a error when the build fails.
    if [ $? -eq 0 ]
    then
      echo "Successfully built $vrep_example"
    else
      echo "Failed building $vrep_example"
      exit 1
    fi
    
    cd ..
    cd ..
done
