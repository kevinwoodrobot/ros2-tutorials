# Action Package 
In this ROS2 tutorial, I will go over how to make a custom action interface package. Recall that action consists of a request and result as seen in client and server, but with a feedback. When we create an instance of an action, it's called a `goal`. 

1. Create package 
```bash
cd ~/ros2_ws/src
ros2 pkg create action_tutorials_interfaces
```

2. Create action folder 
```bash
cd ~/ros2_ws/src # verify
mkdir action_tutorials_interfaces/action
```

3. Create action file. Move `28-.../Fibonacci.action` to `ros2_ws/src/action_tutorials_interfaces/action`

4. Modify `CMakeLists.txt`

5. Modify `package.xml`

6. Build 
```bash
cd ~/ros2_ws
colcon build --packages-select action_tutorials_interfaces
```

7. Run 
```bash
source install/setup.bash
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```

# Next Video:<br>Action Server Client Package C++