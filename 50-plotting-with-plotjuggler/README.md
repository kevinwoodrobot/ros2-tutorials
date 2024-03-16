# ROS2 Plotting Tutorial with PlotJuggler - A Simple Way to Plot in ROS
In this ROS2 plotting tutorial, I will show you how to plot your signals in real-time using PlotJuggler.

I'll be using the Tesla Bot simulation that I built previously, so check it out. 

[![](https://img.youtube.com/vi/PM_1Nb9u-N0/0.jpg)](https://www.youtube.com/watch?v=PM_1Nb9u-N0)

- [Install PlotJuggler](#install-plotjuggler)
- [Run PlotJuggler](#run-plotjuggler)
- [Plotting Signal](#plotting-signal)
- [Saving the Plot Layout](#saving-the-plot-layout)
- [Loading the Plot Layout](#loading-the-plot-layout)

By the end of this video, we will see the joint trajectories for the elbows in a real-time plot. 

## Install PlotJuggler
```bash
sudo apt install ros-${ROS_DISTRO}-plotjuggler
sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros
```

## Run PlotJuggler 
1. Running in terminal
    ```bash
    ros2 run plotjuggler plotjuggler
    ```

2. Running from launch file 
    ```python
    run_plotjuggler = ExecuteProcess(
            cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
            output='screen',
            shell=True  
    )
    ```

Let's start up the Tesla Bot Simulation from our gazebo tutorial. 

In one terminal,
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch gazebo_tutorial gazebo.launch.py 
```

In another terminal,
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run gazebo_tutorial joint_publisher
```

## Plotting Signal 
1. Under Streaming on the left it should say `ROS2 Topic Subscriber`, hit `START`
2. Select the topic name that has the data that you want to plot. In my case, I will choose `/joint_trajectory_controller/joint_trajectory`
3. Find the signal in the list. 
    - joint_trajectory
        - points[]
            - positions[]
                - positions[0]
                - positions[1]
4. Drag the first signal `position[0]` to the plot. 
5. Hover mouse over the `x` and select icon to split the plots. 
6. Now, drag the second signal `position[1]` to the plot. 

## Saving the Plot Layout 
Now that you have a layout setup, you can save it so that you don't have to set it up each time.

1. On the left, go to `Layout` and choose the upload/save icon (up arrow). 
2. Choose folder and save using desired file name. 

## Loading the Plot Layout 
1. On the left, go to `Layout` and choose the load icon (down arrow). 
2. Choose the layout that you saved. 
3. Confirm the topic selection and hit `Ok`. 
