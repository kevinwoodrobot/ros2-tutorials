# Launch File Large Projects
In this ros2 tutorial, I will go over how to orgainize multiple launch files in a large project. We will be calling multiple launch files from one main launch file and use some tricks to handle changing parameters. We will go over the src code for each launch file, build the package, and run the launch file. 

0. Install package for tf2 used later 
```bash
sudo apt install ros-humble-turtle-tf2-py
```

1. Copy all the launch files from `37-...` into `~/ros2_ws/src/launch_tutorial/launch`. Note the main launch file is `37-.../launch_turtlesim_launch.py`, which includes all the other launch files

2. Create a the directory `~/ros2_ws/src/launch/tutorial/config` and copy the `37-.../turtlesim.yaml` over 

3. Update `setup.py`

4. Build package
```bash
cd ~/ros2_ws
colcon build --packages-select launch_tutorial
```

5. Run launch file
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch launch_tutorial launch_turtlesim_launch.py
```


# Next Video:<br>tf2 Coordinate Frame Transform