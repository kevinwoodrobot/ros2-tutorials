# Plugins 
In this ros2 tutorial video, I will show you how to make your own ros2 plugin. We will start off by making a base class package. Then we will create a plugin package that will use the base class package. We will make the plugin xml file required to use the plugin in ros. And finally, we will see the plugin in action. Plugins lets you dynamically load new functionality to your code without having to recompile the codes that's using it, so it may be quite useful! 

# C++ References 
- C++ Classes: 
https://youtu.be/qI5h1MYXiyI?si=VEmDyk9nIHJpOt0r
- C++ Inheritance: 
https://youtu.be/QZpHYxR348I?si=xAOh1mISe3-TutS1
- C++ Virtual Functions: 
https://youtu.be/-4-4bNvT61s?si=LN0veq9P46mzsJzn
- C++ Namespace: 
https://youtu.be/bU1EckIgBmo?si=XLEJDgyQgiEAu1dt
- C++ Exception Handling (Try/Catch): 
https://youtu.be/yclXmO6ToK4?si=OSxJmLFfxJ7U4C5C
- C++ Shared Pointer 
https://youtu.be/6vP8RPEDe6A?si=rdLqqlJHOmomXBAU

## Create Base Class Package 
1. Create base class package 
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies pluginlib --node-name area_node --license Apache-2.0 polygon_base
```

2. Create abstract base class. Move `26-.../polygon_base/regular_polygon.hpp` to `~/ros2_ws/src/polygon_base/include/polygon_base`

3. Modify `CMakeLists.txt` file 

## Create Plugin Package
1. Create `polygon_plugins` package 
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies polygon_base pluginlib --library-name polygon_plugins --license Apache-2.0 polygon_plugins
``` 

2. Replace `ros2_ws/src/polygon_plugins/src/polygon_plugins.cpp` with `26-.../polygon_plugins/polygon_plugins.cpp` 

3. Move `26-.../polygon_plugins/plugins.xml` to `ros2_ws/src/polygon_plugins`

4. Modify `CMakeLists.txt` file 


## Use the Plugin
KEY POINT: The `area_node.cpp` can use the triangle and and square implementation created in the `polygon_plugins` without having to include the polygon_plugin class! 

1. Replace the `~/ros2_ws/src/polygon_base/src/area_node.cpp` with the one in `26-.../polygon_base/area_node.cpp`

2. Build package 
```bash
cd ~/ros2_ws
colcon build --packages-select polygon_base polygon_plugins
```

3. Run node 
```bash
source install/setup.bash
ros2 run polygon_base area_node
```

# Next Video:<br>Rosdep