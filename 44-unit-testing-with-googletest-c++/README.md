# Unit Testing with GoogleTest (GTest) C++
In this ros2 tutorial, I will show you how to do unit testing with GoogleTest (aka GTest) for your C++ ROS packages.

- [C++ Google Test Package](#c-google-test-package)
- [C++ Google Test Example](#c-google-test-example)
- [Update package.xml](#update-packagexml)
- [Update CMakeLists.txt](#update-cmakeliststxt)
- [Build Package](#build-package)
- [Run Unit Test with GTest](#run-unit-test-with-gtest)

## C++ Google Test Package
Move `tutorial_test` package to your `ros2_ws/src`. Package was created using
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake tutorial_test
```

## C++ Google Test Example
Review new test file `.../test/tutorial_test.cpp`

## Update package.xml
See changes for `package.xml`.

> **Side Note:** 
> In case you see the error message `std_msgs` was not found before: 
> ```bash
> CMake Error at /opt/ros/humble/share/ament_cmake_target_dependencies/cmake/ament_target_dependencies.cmake:77 (message):
>   ament_target_dependencies() the passed package name 'std_msgs' was not
>   found before
> Call Stack (most recent call first):
>   CMakeLists.txt:21 (ament_target_dependencies)
> ```
> 
> Solution
> 1. Either you need to install the `std-msgs` package
> ```bash
> sudo apt-get install ros-humble-std-msgs
> ```
> 
> 2. And/or you need to add this to your `CMakeLists.txt` file 
> ```bash
> find_package(std_msgs REQUIRED)
> ```
> 
> Now you should be able to build without error! 

## Update CMakeLists.txt
See changes for `CMakeLists.txt`

## Build package
```bash
cd ~/ros2_ws
colcon build --packages-select tutorial_test
```

## Run Unit Test with GTest
Note, don't need to do the source install step 
```bash
cd ~/ros2_ws
colcon test --packages-select tutorial_test
```

Options to see more print out. Will show details of what failed when using verbose
```bash
colcon test-result --all
colcon test-result --all --verbose
```

# Next Video:<br>Unit Testing with Pytest Python