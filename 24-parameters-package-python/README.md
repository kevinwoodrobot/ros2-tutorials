# Parameters Package Python 
In this ros2 tutorial, I will show you how to create your own node that uses parameters by creating your own package. I will show you how you can change the parameters from the terminal and also from a launch file. 

## Create and Build Parameter Package
1. Create package 
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_parameters --dependencies rclpy
```

2. Create param node by moving `ros-tutorials/20-.../py_parameters_node.py` to `~/ros2_ws/src/py_parameters/py_parameters`

- Python Class: 
https://youtu.be/5u8aovsCQhI?si=DLt_SoctiY3iMg5i
- Pyhon Inheritance: 
https://youtu.be/fSPGTQubT9w?si=uKdHU2sIFqS5RA3o
- Python 'if __name__ == '__main__'
https://youtu.be/yoAFfLf4GBA?si=Iu5Yw5SO-wEyXcA5

3. Update `setup.py` (see modified lines)

4. Build with colcon 
```bash
cd ~/ros2_ws 
colcon build --packages-select py_parameters
```

## Change Parameter in Terminal
1. Test param node 
```bash
cd ~/ros2_ws 
source install/setup.bash
ros2 run py_parameters minimal_param_node
```

2. Modify parameter in another terminal
```bash
cd ~/ros2_ws 
source install/setup.bash

# (Optional) See parameters
ros2 param list 

# Modify parameter 
ros2 param set <node_name> <parameter_name> <value>
ros2 param set /minimal_param_node my_parameter earth
```

## Change Parameter with Launch File
1. Add launch file. Move `23-.../py_parameters_launch.py` to `~/ros2_ws/src/py_parameters/launch`

2. Update `setup.py`

3. Rebuild package
```bash
cd ~/ros2_ws 
colcon build --packages-select py_parameters
```

4. Run the launch file 
```bash
ros2 launch py_parameters py_parameters_launch.py 
```

# Next Video:<br>Doctor