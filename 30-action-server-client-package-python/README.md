# Action Server Client Package Python
In this ros2 tutorial, I will show you how to make your own action server client package in python. We will start off by making our own package, add in the server and client src files, update our setup.py file, build our package, and see the server and client in action. 

## Python References
- Python Class: 
https://youtu.be/5u8aovsCQhI?si=DLt_SoctiY3iMg5i
- Pyhon Inheritance: 
https://youtu.be/fSPGTQubT9w?si=uKdHU2sIFqS5RA3o
- Python 'if __name__ == '__main__'
https://youtu.be/yoAFfLf4GBA?si=Iu5Yw5SO-wEyXcA5

## Create and Build Package
1. Create package 
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python action_tutorials_py --dependencies rclpy action_tutorials_interfaces
```

2. Move `30-.../fibonacci_action_server.py` and `30-.../fibonacci_action_client.py` into `~/ros2_ws/src/action_tutorials_py/action_tutorials_py`

3. Update `setup.py`

4. Build 
```bash
cd ~/ros2_ws
colcon build --packages-select action_tutorials_py
```

5. In one terminal, run the following
```bash
cd ~/ros2_ws 
source install/setup.bash
ros2 run action_tutorials_py fibonacci_action_server
```

6. In another terminal, run the following
```bash
cd ~/ros2_ws 
source install/setup.bash
ros2 run action_tutorials_py fibonacci_action_client
```

# Next Video:<br>Components