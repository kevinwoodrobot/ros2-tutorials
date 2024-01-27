# Launch File Event Handler
In this ros2 tutorial, I will go over how to use event handlers in your launch file. This is great if you want your launch actions to happen at a specific order or sequence. I will go over the launch file that uses the event handlers, build the package, and run the launch file. 

Note that we are using the `launch_tutorial` package from the previous tutorial. 

1. Move file `36-.../example_event_handlers_launch.py` into `~/ros2_ws/src/launch_tutorial/launch`

2. Build package
```bash
cd ~/ros2_ws
colcon build --packages-select launch_tutorial
```

3. Run launch file 
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch launch_tutorial example_event_handlers_launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

# Next Video:<br>Launch File Large Projects