# Build Packages with Colcon
A package contains your source, CmakeList.txt, headers, and package.xml (meta info) for your ROS programs. Colcon is the tool to build your package. 

1. Install colcon
```bash
sudo apt install python3-colcon-common-extensions
```

2. Create workspace and navigate to workspace 
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

3. Add some source code to test building 
```bash
git clone https://github.com/ros/ros_tutorials.git -b humble
```

4. Setup colcon tab completion
```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

5. Build with colcon from `~/ros2_ws`.  The ``--symlink-install`` creates symbolic links (symlinks) to the original files in the source or build directories instead of making copies
```bash
colcon build --symlink-install
```

6. See the folders that were created (build, install, log)
```bash
ls
```

7. Source overlay from `~/ros2_ws`
```bash
source install/local_setup.bash
```

8. Run node 
```bash
ros2 run turtlesim turtlesim_node
```

9. To verify you are modifying your package and not the previous one, let's edit the name of the window. Hit `Ctrl+P` and search `turtle_frame.cpp`. Modify `line 52` to the following
```bash
setWindowTitle("KevinTurtleSim");
```

7. Rebuild, source, then run. Should see the name change 
```bash
colcon build --symlink-install
source install/local_setup.bash
ros2 run turtlesim turtlesim_node
```

# Next Video:<br>Create Your Own Package