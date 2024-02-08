# tf2 Broadcaster Python
In this ros2 tf2 tutorial, I will show you how to make your own tf2 broadcaster in python. We will modify our `learning_tf2_py` package from our previous video, add a new src file, add a launch file, modify the `package.xml` and `setup.py` files, rebuild, and print out the transforms. 

## Python References
If you need some python references, check out my python playlist: 
https://youtube.com/playlist?list=PLSK7NtBWwmpSUenWrmUh0ND_l023RPAXK&si=z1k563TJcVCJVJlc

- Python Class: 
https://youtu.be/5u8aovsCQhI?si=DLt_SoctiY3iMg5i
- Pyhon Inheritance: 
https://youtu.be/fSPGTQubT9w?si=uKdHU2sIFqS5RA3o
- Python 'if __name__ == '__main__'
https://youtu.be/yoAFfLf4GBA?si=Iu5Yw5SO-wEyXcA5

## Learning Tf2 Package
Copy the `learning_tf2_py` from this lesson to `~/ros2_ws/src`

## Review Tf2 Broadcaster Src Code
Added src file `turtle_tf2_broadcaster.py` into `learning_tf2_py/learning_tf2_py`

## Review tf2 Launch File
Created launch directory and launch file `turtle_tf2_demo.launch.py`

## Review package.xml
Modified `package.xml`

## Review setup.py
Modified `setup.py`

## Build package
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select learning_tf2_py
```

## Run the Launch File and Print out Transforms
In a terminal, run the launch file
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py
```

In a second terminal, run the telop
```bash
ros2 run turtlesim turtle_teleop_key
```

In a third terminal, run the tf2_echo to see the translation and rotation
```bash
ros2 run tf2_ros tf2_echo world turtle1
```

# Next Video:<br>tf2 Listener Python
