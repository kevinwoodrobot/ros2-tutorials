# tf2 Static BroadCaster Python
In this ros2 tf2 tutorial, I will show you how to make your own tf2 static broadcaster in python. We will go over creating the package (updating the package.xml and setup.py files) and reviewing the src code, run our static broadcaster, and discuss how we publish static transforms in practice.

## Python References
If you need some python references, check out my python playlist: 
https://youtube.com/playlist?list=PLSK7NtBWwmpSUenWrmUh0ND_l023RPAXK&si=z1k563TJcVCJVJlc

- Python Class: 
https://youtu.be/5u8aovsCQhI?si=DLt_SoctiY3iMg5i
- Pyhon Inheritance: 
https://youtu.be/fSPGTQubT9w?si=uKdHU2sIFqS5RA3o
- Python 'if __name__ == '__main__'
https://youtu.be/yoAFfLf4GBA?si=Iu5Yw5SO-wEyXcA5

## Creating the Package and Reviewing Src Code
1. Copy the `learning_tf2_py` from this lesson to `~/ros2_ws/src`

2. Package was created using
```bash
cd ~/ros2_ws
ros2 pkg create --build-type ament_python --license Apache-2.0 -- learning_tf2_py
```

3. Added src file `static_turtle_tf2_broadcaster.py` into `learning_tf2_py/learning_tf2_py`

4. Modified `package.xml`

5. Modified `setup.py`

6. Build package
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select learning_tf2_py
```

## Running the Static Broadcaster
In one terminal, start the static broadcaster with the turtle at z = 1 (Data Format: x y z rx ry rz - 0 0 1 0 0 0 )
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run learning_tf2_py static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
```

In another terminal, echo the static transform
```bash
cd ~/ros2_ws
ros2 topic echo /tf_static
```


## Publishing Static Transforms in Practice
Using xyz and rpy
```bash
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id
```

Using quaternions
```bash
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id
```

Using launch files (ex with xyz and rpy)
```bash
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'mystaticturtle']
        ),
    ])
```

# Next Video:<br>tf2 Broadcaster Python
