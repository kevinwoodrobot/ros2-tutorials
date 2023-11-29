# Record and Playback Data 
Ros topics can be recorded using ros bags and played back from the recorded bag file. 

1. In a new terminal, run 
```bash 
ros2 run turtlesim turtlesim_node
```

2. In another terminal, run 
```bash
ros2 run turtlesim turtle_teleop_key
```

4. See list of topic names 
```bash
ros2 topic list
```

5. Echo the topic and move the turtle with the teloep node to see it update the command velocity
```bash
ros2 topic echo <topic_name>
ros2 topic echo /turtle1/cmd_vel
```

6. Record topic. Run `ctrl+c` to stop recording. For more topics to record, just add more topic names to the arguments
```bash
ros2 bag record -o <folder_name> <topic_name1> <topic_name2> ...
ros2 bag record -o bagData /turtle1/cmd_vel
```

7. View bag info 
```bash
ros2 bag info <bag_path>
ros2 bag info bagData
```

8. Playback data 
```bash 
ros2 bag play <bag_path>
ros2 bag play bagData
```

# Next Video:<br>Build Packages with Colcon