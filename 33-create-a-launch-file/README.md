# Create a Launch File 
In this ros2 tutorial, I will show you how to launch your nodes from a launch file. We will make a simple launch scripts and go over the code and general structure of the launch file. Then we will test it out by running the launch file using the `ros2 launch` command and see the turtles move. 

1. Make `Launch` directory in workspace 
```bash
cd ~/ros2_ws
mkdir launch
```

2. Move file `33-.../turtlesim_mimic_launch.py` into `~/ros2_ws/launch`

3. Run the launch file 
```bash
cd ~/ros2_ws/launch
# ros2 launch <package_name> <launch_file_name>
ros2 launch turtlesim_mimic_launch.py
```

4. Send some command to move turtles
```bash
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

# Next Video:<br>Integrate Launch File in Package