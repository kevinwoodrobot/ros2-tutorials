# ROS Actions
Actions lets you communicate between nodes with a goal, feedback, and result in a client-server fashion. 

1. In a new terminal, run 
```bash 
ros2 run turtlesim turtlesim_node
```

2. In another terminal, run 
```bash
ros2 run turtlesim turtle_teleop_key
```

3. In the teleop window, try the following and observe 
a. Press one of the letters 'G' and see the status when complete
b. Press one of the letters and cancel early with 'F'
c. Press one of the letters 'G' and before it finishes press another 'F' 


4. See the action clients in node info on the bottom
```bash 
 ros2 node info /teleop_turtle 
```

5. See all actions
```bash
ros2 action list

# List with type 
ros2 action list -t
```

6. See action info 
```bash
ros2 action info <action_name>
ros2 action info /turtle1/rotate_absolute
```

7. See data type for action 
```bash
ros2 interface show turtlesim/action/RotateAbsolute
```

8. Send action goal 
```bash
ros2 action send_goal <action_name> <action_type> <goal>
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```

# Next Video:<br>View Logs