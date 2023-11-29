# ROS Parameters
Parameters are values you can change inside a node. 

1. In a new terminal, run 
```bash 
ros2 run turtlesim turtlesim_node
```

2. In another terminal, run 
```bash
ros2 run turtlesim turtle_teleop_key
```

3. See list of parameters
```bash
ros2 param list 
```

4. Get parameter value 
```bash
ros2 param get <node_name> <parameter_name> 
ros2 param get /turtlesim background_g
```

5. Set parameter value 
```bash
ros2 param set <node_name> <parameter_name> <value>
ros2 param set /turtlesim background_g 255
```

6. View all param for a node 
```bash
ros2 param dump <node_name>
ros2 param dump /turtlesim
```

7. Store param in yaml file 
```bash
ros2 param dump /turtlesim > turtlesim.yaml
```

8. Load param from yaml file 
```bash
ros2 param load /turtlesim turtlesim.yaml
```

9. Load param on startup
```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

# Next Video:<br>Actions