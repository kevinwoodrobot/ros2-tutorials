# ROS Topic
A topic is one way data is moved between nodes, where 1 or more publisher nodes can connect to 1 or more subscriber nodes. 

1. Open new terminal and run
```bash
ros2 run <package_name> <executable_name>
ros2 run turtlesim turtlesim_node
```

2. Open new terminal and run. Move the turtle anywhere (needed for the rqt graph to show connection later). 
```bash
ros2 run turtlesim turtle_teleop_key
```

3. See list of topics 
```bash
ros2 topic list
# See Details 
ros2 topic list -t
```

4. Start rqt graph. Uncheck hide boxes to see hidden topics. 
```bash
rqt_graph
```

5. See topic output. Move with teleop to see output after running command. 
```bash
ros2 topic echo <topic_name>
ros2 topic echo /turtle1/cmd_vel 
```

6. View topic info 
```bash 
ros2 topic info <topic_name>
ros2 topic info /turtle1/cmd_vel
```

7. See interface definition
```bash 
ros2 interface show <type>
ros2 interface show geometry_msgs/msg/Twist
```

8. Publish data to a topic ("--once" means publish once and exit)
```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

9. Publish data to a topic with rate 1 hz 
```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

10. Echo the pose topic. Go to the rqt graph and refresh.  
```bash
ros2 topic echo /turtle1/pose
```

11. View topic freq
```bash 
ros2 topic hz <topic_name>
ros2 topic hz /turtle1/pose
```

# Next Video:<br>Services