# Running Executables
```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

See list of packages
```bash
ros2 pkg list
```

See list of <packages, executables> 
```bash
ros2 pkg executables 
```

See list of executables from turtlesim package 
```bash
ros2 pkg executables turtlesim
```

Where is turtlesim? 
```bash
cd /opt/ros/humble/
code . 
Ctrl+P, turtlesim
```
Should see `turtlesim` is located in 
```bash
share/ament_index/resource_index/packages
```

Run turtlesim. Do `ros2 run -h` so see options
```bash
ros2 run <package_name> <executable_name>
ros2 run turtlesim turtlesim_node
```

In other terminal, run the teleop mode
```bash
ros2 run turtlesim turtle_teleop_key
```

# Next Video:<br>RQT