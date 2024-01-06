# Managing Dependencies with rosdep 
In this ROS2 tutorial, I will go over the rosdep command. 

## What is rosdep? 
The `rosdep` command is a tool used to manage dependencies in ROS. 

## How does rosdep work? 
`rosdep` will find the "rosdep keys" (the dependencies listed in the package.xml file) and check with a central index (https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml). Once found, they are installed. 

## Types of dependencies in package.xml files 
- `<depend>` is for dependencies in build time and run time, usually for C++ packages 
- `<build_depend` is for dependencies for building your package (not during execution)
- `<build_export_depend>` is for external packages that depend on this package
- `<exec_depend>` is for run time (shared libraries, executables, python modules, launch scripts)
- `<test_depend` is for running tests 

## What rosdep key to use? 
In the `distribution.yaml` file, search for the library name. Example below shows the library for `rclcpp` (used to make nodes for pub/sub and server/client in previous tutorials)

```bash
rclcpp:
    doc:
      type: git
      url: https://github.com/ros2/rclcpp.git
      version: humble
    release:
      packages:
      - rclcpp
      - rclcpp_action
      - rclcpp_components
      - rclcpp_lifecycle
      tags:
        release: release/humble/{package}/{version}
      url: https://github.com/ros2-gbp/rclcpp-release.git
      version: 16.0.7-1
    source:
      test_pull_requests: true
      type: git
      url: https://github.com/ros2/rclcpp.git
      version: humble
    status: maintained
```
## rosdep installation 
```bash
apt-get install python3-rosdep
sudo rosdep init
rosdep update
```

## Running the rosdep command
```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
```
Meaning: 
- `--from-paths src` checks `package.xml` files in the `src` folder 
- `-y` installs all answer yes to all prompts 
- `--ignore-src` ignores packages in the src folder 

# Next Video:<br>Action Package