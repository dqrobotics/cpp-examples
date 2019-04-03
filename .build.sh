#!/bin/bash

cd cmake

for d in */; do
cd $d
mkdir -p build
cd build
cmake -DCMAKE_CXX_FLAGS=${CXX_FLAGS} -DCMAKE_BUILD_TYPE=Release .. 
make
cd ..
cd ..
done
