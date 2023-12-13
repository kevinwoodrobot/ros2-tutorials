# Server and Client Python 
In this ROS tutorial, we will create a server and client package in python. 

1. Create package from `~/ros2_ws/src`. We will make a `py_srvcli` package folder with the following generated `py_srvcli`, `resource`, `test`, `package.xml`, `setup.cfg`, and `setup.py`
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```

2. For the srvcli src, copy `ros2-tutorial/20-.../service_member_function.py` and `ros2-tutorial/20-.../client_member_function.py` into `~/ros2_ws/src/py_srvcli/py_srvcli`

- Python Class: 
https://youtu.be/5u8aovsCQhI?si=DLt_SoctiY3iMg5i
- Pyhon Inheritance: 
https://youtu.be/fSPGTQubT9w?si=uKdHU2sIFqS5RA3o

3. Don't need to update `package.xml` because we used `--dependencies rclcpp example_interfaces` when we made our package

4. Update `setup.py` (see file for added lines)

5. (Optional) Check for missing dependencies from `~/ros2_ws`
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

6. Build with colcon from `~/ros2_ws`
```bash
colcon build --packages-select py_srvcli
```

7. In one terminal, run the service from `~/ros2_ws`
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_srvcli service
```

8. In another terminal, run the client from `~/ros2_ws`
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_srvcli client 3 7
```

# Next Video:<br>Custom Interface Package