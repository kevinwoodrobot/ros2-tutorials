# Launch Multiple Nodes 
Using a launch file, we can startup two nodes with just one command. 

1. Launch two nodes 
```bash 
ros2 launch <package_name> <launch_arguments> 
ros2 launch turtlesim multisim.launch.py
```

2. Details of launch file
```bash
# turtlesim/launch/multisim.launch.py

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
    ])
```

# Next Video:<br>Record and Playback Data