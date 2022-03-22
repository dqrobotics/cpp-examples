This example shows how to read json files containing the DH parameters of a serial manipulator.

## Compile

```shell
cd cmake/json11_interface
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
## Run
```shell
./json11_interface_example 
```
![json](https://user-images.githubusercontent.com/23158313/159579167-b1833d0d-4323-450b-883c-3a4f0698ab27.gif)

# Tips for QT Creator

1. Set the build directory inside the json11_interface folder. Otherwise, you may experience the error
```shell
terminate called after throwing an instance of 'std::runtime_error'
  what():  Json parse error: unexpected end of input
```



  





