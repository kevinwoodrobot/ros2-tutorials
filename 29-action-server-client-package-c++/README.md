# Action Server Client Package C++
In this ros2 tutorial, I will show you how to make your own action server client package in C++. We will create our own package, go over the visibility_control.h file needed for Windows build, walk through the server and client cpp files, update the CMakeLists.txt file, build the package, and see the server and client package in action. 

## C++ References
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

## Create and Build Package
1. Create package 
```bash
cd ~/ros2_ws/src
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp
```

2. Move `29-.../visibility_control.h` into `~/ros2_ws/src/action_tutorials_cpp/include/action_tutorials_cpp`. This is needed to makes packages compile on Windows. 

3. Move `29-.../fibonacci_action_server.cpp` into `~/ros2_ws/src/action_tutorials_cpp/src`

4. Move `29-.../fibonacci_action_client.cpp` into `~/ros2_ws/src/action_tutorials_cpp/src`

5. Update `CMakeLists.txt`

6. Build 
```bash
cd ~/ros2_ws
colcon build --packages-select action_tutorials_cpp
```

7. In one terminal, run the following
```bash
cd ~/ros_ws 
source install/setup.bash
ros2 run action_tutorials_cpp fibonacci_action_server
```

8. In another terminal, run the following
```bash
cd ~/ros_ws 
source install/setup.bash
ros2 run action_tutorials_cpp fibonacci_action_client
```

# Next Video:<br>Action Server Client Package Python