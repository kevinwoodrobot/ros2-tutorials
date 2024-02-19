# Unit Testing with Pytest Python
In this ros2 tutorial, I will go over how to do unit testing with pytest for your python ROS package.

- [Python Test Package](#python-test-package)
- [Example of Python Test File](#example-of-python-test-file)
- [Build Package](#build-package)
- [Run Python Unit Test](#run-python-unit-test)

## Python Test Package
Move package `tutorial_test_python` to `~/ros2_ws/src`. Package was created using
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python tutorial_test_python
```

## Example of Python Test File
Review new test file `test_nameOfYourTest.py` in the `test` folder

## Build Package
Build package
```bash
cd ~/ros2_ws
colcon build --packages-select tutorial_test_python
```

## Run Python Unit Test
To run the python test, we can run the following
```bash
# Run All Test
colcon test --packages-select tutorial_test_python

# Run Specific Test
colcon test --packages-select tutorial_test_python --pytest-args -k test_math
```

Options to see more print out. Will show details of what failed when using verbose
```bash
colcon test-result --all
colcon test-result --all --verbose
```

# Next Video:<br>URDF Tutorial - Describe Any Robot