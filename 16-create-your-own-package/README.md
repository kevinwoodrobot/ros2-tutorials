# Create Your Own Package
Recall a package contains your source, CmakeList.txt, headers, and package.xml (meta info) for your ROS programs. Here we will create our own package from scratch. 

0. Prereq - Install colcon and create workspace. SKIP if did it already from my previous video!
```bash
sudo apt install python3-colcon-common-extensions
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

1. Create package from `~/ros2_ws/src`
```bash
ros2 pkg create --build-type ament_cmake --node-name <node_name> <package_name>
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

2. Check out the files and folders that it made. Open files and see what's inside. 
```bash
include/my_package
src/my_node.cpp
CMakeLists.txt
package.xml
```

3. Build that package from `~/ros_ws` 
```bash
colcon build --packages-select my_package
```

4. Source 
```bash
source install/local_setup.bash
```

5. Run node 
```bash
ros2 run my_package my_node
```

# Next Video:<br>Publisher and Subscriber C++