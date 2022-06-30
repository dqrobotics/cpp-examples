#!/bin/bash

cd cmake/

declare -a qpoases_examples_array=("qpoases_interface")
 
for qpoases_example in ${qpoases_examples_array[@]}; do
    cd $qpoases_example
    cd build
    ./qpoases_example
    # Guaranteeing a error when the build fails.
    if [ $? -eq 0 ]
    then
      echo "Successfully ran $qpoases_example"
    else
      echo "Failed running $qpoases_example"
      exit 1
    fi
    
    cd ..
    cd ..
done
