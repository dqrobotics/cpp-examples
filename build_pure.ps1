cd cmake
$pure_examples_array = @("jacobian_time_derivative","performance_evaluation","multiplication_evaluation","simple_tests")

foreach ( $pure_example in $pure_examples_array )
{
    cd $pure_example
    mkdir build
    cd build
	
    cmake ..
    cmake --build . --config Release
    
    cd ..
    cd ..
}