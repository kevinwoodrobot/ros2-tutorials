# RQT
RQT is a GUI interface that let's you work with plugins for robot control and visualization. 

1. Install rqt 
```bash
sudo apt install ~nros-humble-rqt*
```

2. Run rqt 
```bash
rqt
```

3. Run TurtleSim 
- In one terminal, start the turtle_node
```bash
ros2 run turtlesim turtlesim_node
```
- In another terminal, start the teleop node 
```bash
ros2 run turtlesim turtle_teleop_key
```
4. Spawn a turtle in rqt 
- `Plugins > Services > Service Caller`
- Under the Service drop down, choose `/spawn`
- Enter some x, y position and hit the button `Call`

4. Edit turtle path color 
- Under Service drop down, choose `/turtle1/set_pen`
- Modify r value to `255` and press `Call`
- Now move the turtle

5. Change control to the new turtle
```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

# Next Video:<br>Nodes 