# Service and Client C++ 
In this ROS tutorial, we will create a service and client package in C++. 

0. Install CMake extension to view CMakeLists.txt files with syntax coloring.  

1. Create package from `~/ros2_ws/src`. We will make a `cpp_srvcli` package folder with the following generated `include`, `src`, `CMakeLists.txt`, and `package.xml`
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
```

- Dependencies will be added to the cmake and xml file. The `example_interface` contains the request and response format: 
```bash
int64 a
int64 b
---
int64 sum
```

2. For the srvcli src, copy `ros2-tutorials/19-.../add_two_ints_server.cpp` and `ros2-tutorials/17-.../add_two_ints_client.cpp` into `~/ros2_ws/src/cpp_srvcli/src`

- C++ Function Pointer
https://www.youtube.com/watch?v=Mvni51kg8zY
- C++ Shared Pointer 
https://youtu.be/6vP8RPEDe6A?si=rdLqqlJHOmomXBAU

3. Don't need to update `package.xml` because we used `--dependencies rclcpp example_interfaces` when we made our package

4. Update `CMakeLists.txt` (see file for added lines)

5. (Optional) Check for missing dependencies from `~/ros2_ws`
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

6. Build with colcon from `~/ros2_ws`
```bash
colcon build --packages-select cpp_srvcli
```

7. In one terminal, run the talker from `~/ros2_ws`
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run cpp_srvcli server
```

8. In another terminal, run the listener from `~/ros2_ws`
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run cpp_srvcli client 3 7 
```

# Next Video:<br>Server and Client Python