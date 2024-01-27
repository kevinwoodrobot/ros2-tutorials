# Launch File Substitution
In this ros2 tutorial, I will show you how to use substitutions in your launch file. Substitution lets you use command line arguments to customize your launch settings when you run the launch file. I will show you how to create the package, make the launch files, go over the code, build the package and test out running the launch file with user defined input arguments. 

1. Create package 
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python launch_tutorial
```

2. Make `launch` directory
```bash
mkdir ~/ros2_ws/src/launch_tutorial/launch
```

3. Update `setup.py`

4. Copy the file `35-.../example_main_launch.py` and `35-.../example_substitutions_launch.py` to `~/ros2_ws/src/launch_tutorial/launch`

5. Build package 
```bash
cd ~/ros2_ws
colcon build --packages-select launch_tutorial
```

6. Run launch file 
```bash
source install/setup.bash
ros2 launch launch_tutorial example_main_launch.py
```

7. See arguments for launch file 
```bash
ros2 launch launch_tutorial example_substitutions_launch.py --show-args
```

8. Run launch file with args 
```bash
ros2 launch launch_tutorial example_substitutions_launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

# Next Video:<br>Launch File Event Handler