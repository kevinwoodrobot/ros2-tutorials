# Doctor 
Ros2 doctor (aka "wtf - where's the fault") is a diagnostic tool that helps you check if your dependencies or packages are up to date and if there's any communication issues between your nodes. 

1. Run ros2 doctor 
```bash
ros2 doctor 

# OR 
ros2 wtf 
```

2. In one terminal, run turtlesim_node 
```bash
cd ~/ros_ws 
ros2 run turtlesim turtlesim_node
```

3. In another terminal, run the teleop node 
```bash 
cd ~/ros_ws 
ros2 run turtlesim turtle_teleop_key
```

4. Run ros2 doctor and see the warnings
```bash 
ros2 doctor
```

5. Subscribe to one of the topics 
```bash
ros2 topic echo /turtle1/color_sensor
```

6. Run ros2 doctor and should see one of the warnings go away 
```bash
ros2 doctor 
```

7. See full report 
```bash
ros2 doctor --report
```

# Next Video:<br>Plugins