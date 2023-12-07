# Publisher and Subscriber C++ 
In this ROS tutorial, we will create a pub and sub package in C++. 

0. Install CMake extension to view CMakeLists.txt files with syntax coloring.  

1. Create package from `~/ros2_ws/src`. We will make a `cpp_pubsub` package folder with the following generated `include`, `src`, `CMakeLists.txt`, and `package.xml`
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake cpp_pubsub
```

2. For the pubsub src, copy `ros2-tutorial/17-.../publisher_member_function.cpp` and `ros2-tutorial/17-.../subscriber_member_function.cpp`into `~/ros2_ws/src/cpp_pubsub/src`

- C++ Classes: 
https://youtu.be/qI5h1MYXiyI?si=VEmDyk9nIHJpOt0r
- C++ Inheritance: 
https://youtu.be/QZpHYxR348I?si=xAOh1mISe3-TutS1
- C++ Shared Pointer: 
https://youtu.be/6vP8RPEDe6A?si=rdLqqlJHOmomXBAU

3. Update `package.xml` (see file for added lines)

4. Update `CMakeLists.txt` (see file for added lines)

5. (Optional) Check for missing dependencies from `~/ros2_ws`
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

6. Build with colcon from `~/ros2_ws`
```bash
colcon build --packages-select cpp_pubsub
```

7. In one terminal, run the talker from `~/ros2_ws`
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run cpp_pubsub talker
```

8. In another terminal, run the listener from `~/ros2_ws`
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run cpp_pubsub listener
```

# Next Video:<br>Publisher and Subscriber Python