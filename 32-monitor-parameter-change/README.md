# Monitor Parameter Change
In this ros2 tutorial, I will go over how to monitor a parameter from a node in the same executable and also from a remote node (not in the same executable). We will create the package, review the source code, update the CmakeLists.txt, build the package, and test out the two cases. 

## Monitor Own Node Parameter Change
1. Create Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_parameter_event_handler --dependencies rclcpp
```

2. Move `32-.../parameter_event_handler.cpp` to `~/ros2_ws/src/cpp_parameter_event_handler/src`. Review code changes for `Monitor Remote Node` later. 

3. Update `CmakeList.txt`

4. Build package
```bash
cd ~/ros2_ws
colcon build --packages-select cpp_parameter_event_handler
```

5. In one terminal, run event handler
```bash
source install/setup.bash
ros2 run cpp_parameter_event_handler parameter_event_handler
```

6. In another terminal, set the param
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 param set node_with_parameters an_int_param 43
```

## Monitor Another Node's Parameter Change
1. Review codes change for remote node monitoring

2. Build package
```bash
cd ~/ros2_ws/
colcon build --packages-select cpp_parameter_event_handler
```

3. In terminal 1, start the event handler
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run cpp_parameter_event_handler parameter_event_handler
```

4. In terminal 2, start the node
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run demo_nodes_cpp parameter_blackboard
```

5. In terminal 3, set the parameter
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 param set parameter_blackboard a_double_param 3.45
```

# Next Video:<br>Create a Launch File