#!/bin/bash

cd cmake/

declare -a qpoases_examples_array=("qpoases_interface")
 
for qpoases_example in ${qpoases_examples_array[@]}; do
    cd $qpoases_example
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    
    make
    # Guaranteeing a error when the build fails.
    if [ $? -eq 0 ]
    then
      echo "Successfully built $qpoases_example"
    else
      echo "Failed building $qpoases_example"
      exit 1
    fi
    
    cd ..
    cd ..
done
