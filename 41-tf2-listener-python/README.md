# tf2 Listener Python
In this ros2 tf2 tutorial, I will show you how to make your own tf2 listener in python. We will recreate our demo from our first tf2 video and spawn a first turtle and make the second turtle follow the first one by listening to the frame transforms. We will modify our `learning_tf2_py` package from our previous video, add a new src file, modify the launch file, modify the `setup.py` file, rebuild, and observe the turtle following. 

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

## Review Tf2 Listener Src File
Added src file `turtle_tf2_listener.py` into `learning_tf2_py/learning_tf2_py`

## Update Launch File
Added two nodes to the launch file `turtle_tf2_demo.launch.py` and add argument for the `target_frame`

## Modify setup.py
Modified `setup.py`

## Build package
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select learning_tf2_py
```

## Run the Launch File
Run the launch file and see the listener message in the terminal
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py
```

In a second terminal, run the telop
```bash
ros2 run turtlesim turtle_teleop_key
```

# Next Video:<br>tf2 Adding a Frame Python
