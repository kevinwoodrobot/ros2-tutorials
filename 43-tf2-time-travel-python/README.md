# tf2 Time Travel Python (following with delay)
If this ros2 tf2 tutorial, we will see how to make a turtle follow another turtle with delay.  

- [Python References](#python-references)
- [Turtle Tf2 Listener Update](#turtle-tf2-listener-update)
- [Turtle Following with Insufficient Data](#turtle-following-with-insufficient-data)
- [Turtle Following with Delay](#turtle-following-with-delay)

## Python References
If you need some python references, check out my python playlist: 
https://youtube.com/playlist?list=PLSK7NtBWwmpSUenWrmUh0ND_l023RPAXK&si=z1k563TJcVCJVJlc

- Python Class: 
https://youtu.be/5u8aovsCQhI?si=DLt_SoctiY3iMg5i
- Pyhon Inheritance: 
https://youtu.be/fSPGTQubT9w?si=uKdHU2sIFqS5RA3o
- Python 'if __name__ == '__main__'
https://youtu.be/yoAFfLf4GBA?si=Iu5Yw5SO-wEyXcA5

## Turtle Tf2 Listener Update
Update the `turtle_tf2_listener.py` file in the `learning_tf2_py` package from the last video.

## Turtle Following with Insufficient Data
Use `TEST 1` to see the turtle try to follow the carrot with a 5 second delay. You will see the turtle go crazy because there is no data to follow. Make sure to comment out code under `TEST 2` and uncomment code under `TEST 1`. 
```bash
cd ~/ros2_ws
colcon build --packages-select learning_tf2_py
source install/setup.bash
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py target_frame:=carrot1 
```
## Turtle Following with Delay
Use `TEST 2` to see the turtle follow the other turtle with a 5 second delay. Make sure to comment out code under `TEST 1` and uncomment code under `TEST 2`.
```bash
cd ~/ros2_ws
colcon build --packages-select learning_tf2_py
source install/setup.bash
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py
```

In another terminal, run the teleop to move the turtle
```bash
ros2 run turtlesim turtle_teleop_key
```

# Next Video:<br>Unit Testing with GoogleTest C++