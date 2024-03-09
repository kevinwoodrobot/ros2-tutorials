# ROS2 Rviz2 Robot Simulation Trajectory using State Publisher 
In this ROS2 Rviz2 tutorial, I will show you how to simulate your robot in RVIZ using the robot state publisher and our own state publisher. 

- [R2D2 ROS Package](#r2d2-ros-package)
- [RVIZ Configuration Summary](#rviz-configuration-summary)
- [R2D2 URDF Summary](#r2d2-urdf-summary)
- [Review Launch File](#review-launch-file)
- [Review State Publisher Src File](#review-state-publisher-src-file)
- [Update Setup.py File](#update-setuppy-file)
- [Build R2D2 URDF Package](#build-r2d2-urdf-package)
- [Run the R2D2 Example in Rviz](#run-the-r2d2-example-in-rviz)

By the end of this video, we will see the robot go around in circles!

## R2D2 ROS Package
Move `urdf_tutorial_r2d2` package to your workspace `~/ros2_ws/src`
> Package was created using: 
> ```bash
> cd ~/ros2_ws/src
> ros2 pkg create --build-type ament_python --license Apache-2.0 urdf_tutorial_r2d2 --dependencies rclpy
> ```

## RVIZ Configuration Summary 
The `.../urdf/r2d2.rviz` file can be used to configure your rviz settings so you don't have to re-add the plugin and update settings each time. Otherwise, if you wanted the `RobotModel` you need to do `Add > rviz_default_plugins > RobotModel`. 

- Global Options
  - Fixed Frame: odom
- RobotModel
  - Description Topic: /robot_description 
- TF


## R2D2 URDF Summary
If you are new to URDF, I have a URDF Tutorial video!

**ROS2 URDF Tutorial - Describe Any Robot (Links and Joints)**

[![](https://img.youtube.com/vi/LsKL8N5Iwkw/0.jpg)](https://www.youtube.com/watch?v=LsKL8N5Iwkw)

The `.../urdf/r2d2.urdf.xml` file can be summarized below: 

| Name of Link | Geometry                             |
| ------------ | ------------------------------------ |
| axis         | Cylinder (radius: 0.01, length: 0.5) |
| leg1         | Box (size: 0.20 x 0.10 x 0.8)        |
| leg2         | Box (size: 0.20 x 0.10 x 0.8)        |
| body         | Cylinder (radius: 0.20, length: 0.6) |
| head         | Sphere (radius: 0.4)                 |
| rod          | Cylinder (radius: 0.02, length: 0.2) |
| box          | Box (size: 0.05 x 0.05 x 0.05)       |

| Name of Joint | Parent Link | Child Link |
| ------------- | ----------- | ---------- |
| leg1connect   | axis        | leg1       |
| leg2connect   | axis        | leg2       |
| tilt          | axis        | body       |
| swivel        | body        | head       |
| periscope     | head        | rod        |
| boxconnect    | rod         | box        |

## Review Launch File
We will use the `.../launch/demo.launch.py` file to run the `robot_state_publisher` and the `state_publisher`

## Review State Publisher Src File 
We will review the `state_publisher.py` file which will be used to compute the kinematics for R2D2. 

## Update Setup.py File 
Review changes in the `setup.py` file required for building the package. 

## Build R2D2 URDF Package
```bash
cd ~/ros2_ws
colcon build --packages-select urdf_tutorial_r2d2
```

## Run the R2D2 Example in Rviz
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch urdf_tutorial_r2d2 demo.launch.py
```

```bash
cd ~/ros2_ws
source install/setup.bash
rviz2 -d install/urdf_tutorial_r2d2/share/urdf_tutorial_r2d2/r2d2.rviz
```

# Next Video:<br>Gazebo Tutorial - Simulate Any Robot