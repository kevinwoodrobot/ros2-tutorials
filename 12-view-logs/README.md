# View Logs 
Rqt_console is a GUI used to view log messages in ROS. 

1. Start rqt_console
```bash
ros2 run rqt_console rqt_console 
```

2. Run turtlesim node 
```bash
ros2 run turtlesim turtlesim_node
```

3. Run teleop node 
```bash
ros2 run turtlesim turtle_teleop_key
```

4. Crash into wall and see error message. Should see `info` and `warning` message 

5. Set log types to only show warning. Will no longer see `info` messages at very start of node. 
```bash
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```

# Next Video:<br>Launch Multiple Nodes 

