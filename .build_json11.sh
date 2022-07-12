#!/bin/bash
set -e

cd cmake

cd json11_interface
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..

make
# Guaranteeing a error when the build fails.
if [ $? -eq 0 ]
then
echo "Successfully built json11_interface"
./json11_interface_example
else
echo "Failed building json11_interface"
exit 1
fi

cd ..
cd ..
