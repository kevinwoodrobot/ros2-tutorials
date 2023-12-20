# Parameters Package C++ 
In this ros2 tutorial, I will show you how to create your own node that uses parameters by creating your own package. I will show you how you can change the parameters from the terminal and also from a launch file. 

## Create and Build Parameter Package
1. Create package 
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
```

2. Create param node by moving `ros-tutorials/20-.../cpp_parameters_node.cpp` to `~/ros2_ws/src/cpp_parameters/src`

- C++ Classes: 
https://youtu.be/qI5h1MYXiyI?si=VEmDyk9nIHJpOt0r
- C++ Vectors: 
https://youtu.be/2nwd0xnR0rI?si=RGjFIiZQk4nVn9Xf
- C++ Inheritance 
https://youtu.be/QZpHYxR348I?si=xAOh1mISe3-TutS1
- C++ Function Pointer
https://www.youtube.com/watch?v=Mvni51kg8zY
- C++ Shared Pointer 
https://youtu.be/6vP8RPEDe6A?si=rdLqqlJHOmomXBAU

3. Update `CmakeLists.txt` (see modified lines)

4. Build with colcon 
```bash
cd ~/ros2_ws 
colcon build --packages-select cpp_parameters
```

## Change Parameter in Terminal
1. Test param node 
```bash
cd ~/ros2_ws 
source install/setup.bash
ros2 run cpp_parameters minimal_param_node
```

2. Modify parameter in another terminal
```bash
cd ~/ros2_ws 
source install/setup.bash

# See parameters
ros2 param list 

# Modify parameter 
ros2 param set <node_name> <parameter_name> <value>
ros2 param set /minimal_param_node my_parameter earth
```

## Change Parameter with Launch File
1. Add launch file. Move `23-.../cpp_parameters_launch.py` to `~/ros2_ws/src/cpp_parameters/launch`

2. Update `CmakeLists.txt`

3. Rebuild package
```bash
cd ~/ros2_ws 
colcon build --packages-select cpp_parameters
```

4. Run the launch file 
```bash
ros2 launch cpp_parameters cpp_parameters_launch.py 
```

# Next Video:<br>Parameters Package Python