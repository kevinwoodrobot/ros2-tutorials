# Publisher and Subscriber Python 
In this ROS tutorial, we will create a pub and sub package in python. 

1. Create package from `~/ros2_ws/src`. We will make a `py_pubsub` package folder with the following generated `py_pubsub`, `resource`, `test`, `package.xml`, `setup.cfg`, and `setup.py`
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub
```

2. For the pubsub src, copy `ros2-tutorial/17-.../publisher_member_function.py` and `ros2-tutorial/17-.../subscriber_member_function.py` into `~/ros2_ws/src/py_pubsub/py_pubsub`

- Python Class: 
https://youtu.be/5u8aovsCQhI?si=DLt_SoctiY3iMg5i
- Pyhon Inheritance: 
https://youtu.be/fSPGTQubT9w?si=uKdHU2sIFqS5RA3o

3. Update `package.xml` (see file for added lines)

4. Update `setup.py` (see file for added lines)

5. (Optional) Check for missing dependencies from `~/ros2_ws`
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

6. Build with colcon from `~/ros2_ws`
```bash
colcon build --packages-select py_pubsub
```

7. In one terminal, run the talker from `~/ros2_ws`
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_pubsub talker
```

8. In another terminal, run the listener from `~/ros2_ws`
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_pubsub listener
```

# Next Video:<br>Server and Client C++ 