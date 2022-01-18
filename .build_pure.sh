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
    cd ..
    cd ..
done
