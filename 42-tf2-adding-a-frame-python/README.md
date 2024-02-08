# tf2 Adding a Frame Python
In this ros2 tf2 tutorial, we will see how to create a new frame called carrot and make the turtle follow the carrot frame. 

- [Python References](#python-references)
- [Learning Tf2 Package](#learning-tf2-package)
- [Review tf2 Fixed Broadcaster and Dynamic Broadcaster](#review-tf2-fixed-broadcaster-and-dynamic-broadcaster)
- [Review Launch Files](#review-launch-files)
- [Modify setup.py](#modify-setuppy)
- [Build Package](#build-package)
- [Run Fixed and Dynamic Frame Examples](#run-fixed-and-dynamic-frame-examples)


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

## Review tf2 Fixed Broadcaster and Dynamic Broadcaster
Added src files `fixed_frame_tf2_broadcaster.py` and `dynamic_frame_tf2_broadcaster.py` to the package

## Review Launch Files 
Created 2 launch files `turtle_tf2_fixed_frame_demo.launch.py` and `dynamic_frame_tf2_broadcaster.py`

## Modify setup.py
Modified `setup.py`

## Build package
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select learning_tf2_py
```

## Run Fixed and Dynamic Frame Examples
Run fixed frame example to see turtle follow a fixed carrot frame
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py target_frame:=carrot1
```

> NOTE: Can also run fixed frame example by modifying the launch file to take in launch arguments for the `target_frame`
> ```bash
> def generate_launch_description():
>     demo_nodes = IncludeLaunchDescription(
>         ...,
>         launch_arguments={'target_frame': 'carrot1'}.items(),
>         )
> ```

Run dynamic frame example to see the turtle follow the moving carrot frame
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch learning_tf2_py turtle_tf2_dynamic_frame_demo.launch.py
```

# Next Video:<br>tf2 Time Travel Python