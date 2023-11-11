# ROS Node
ROS nodes are standalone executables used to send or receive data by using: 
- topics 
- services 
- actions 
- parameters  

# ROS2 Node Commands 
1. See ros2 node help
```bash 
ros2 node -h 
```

2. List current nodes running. Right now there should be nothing. 
```bash
ros2 node list 
```

3. Run a node from previous lessons
```bash 
ros2 run turtlesim turtlesim_node
```

4. Now list the nodes again and you should see `/turtlesim`
```bash
ros2 node list 
```

5. See node info 
```bash 
ros2 node info <node_name>
ros2 node info /turtlesim
```

# Next Video:<br>Topics 