# Integrate Launch File in Package
In this ros2 tutorial, I will show you how to create and use a launch file from a package. I will go through the steps from creating the package, making the launch file, go over the code for the launch file, build the package and run the launch file. 

1. Create package 
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 py_launch_example
```

2. Create `launch` directory
```bash
mkdir ~/ros2_ws/src/py_launch_example/launch
```

3. Update `setup.py`

4. Copy launch file from `34-.../my_script_launch.py` to `~/ros2_ws/src/py_launch_example/launch`

5. Build package 
```bash
cd ~/ros2_ws
colcon build --packages-select py_launch_example
```

6. Run launch file 
```bash
source install/setup.bash
ros2 launch py_launch_example my_script_launch.py
```

# Next Video:<br>Launch File Substitution