#!/bin/bash

cd cmake

# An array containing the "pure" examples, i.e. those that can be compiled and ran without any extra package.
declare -a pure_examples_array=("jacobian_time_derivative" "performance_evaluation" "simple_tests")
 
# Build all "pure" examples
for pure_example in ${pure_examples_array[@]}; do
    cd $pure_example
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    
    make
    # Guaranteeing a error when the build fails.
    if [ $? -eq 0 ]
    then
      echo "Successfully built $pure_example"
    else
      echo "Failed building $pure_example"
      exit 1
    fi
    
    cd ..
    cd ..
done
