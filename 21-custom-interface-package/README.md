# Custom Interface Package
In this video, we will go over how to make a custom interface package. This will allow you to create you own interface to communicate between nodes. 

## Create Interface Package 
1. Create interface package 
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

3. Move our custom messages from `21-.../interface/msg` to `~/ros2_ws/src/tutorial_interfaces`

4. Move our custom service from `21-.../interface/srv` to `~/ros2_ws/src/tutorial_interfaces`

5. Replace the `CMakeLists.txt` and `package.xml` from `~/ros2_ws/src/tutorial_interfaces` with `21-.../interface`

6. Build the package 
```bash
cd ~/ros2_ws 
colcon build --packages-select tutorial_interfaces
```

7. Check that the following interfaces return what was defined in our files 
```bash
cd ~/ros2_ws 
source install/setup.bash
ros2 interface show tutorial_interfaces/msg/Num
ros2 interface show tutorial_interfaces/msg/Sphere
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```
## Test Pubsub 
1. Replace the `CMakesLists.txt` and `package.xml` from `~/ros2_ws/src/cpp_pubsub` with the new ones in `21-.../cpp_pubsub` 

2. Replace the `subscriber_member_function.cpp` and `publisher_member_function.cpp` from `~/ros2_ws/src/cpp_pubsub` with the new ones in `21-.../cpp_pubsub` 

3. Build package 
```bash
cd ~/ros2_ws
colcon build --packages-select cpp_pubsub
```

4. Test out pubsub using two terminals 
```bash
# Terminal 1 
cd ~/ros2_ws
source install/setup.bash
ros2 run cpp_pubsub talker

# Terminal 2 
cd ~/ros2_ws
source install/setup.bash
ros2 run cpp_pubsub listener
```

## Test Srvcli
1. Replace the `CMakesLists.txt` and `package.xml` from `~/ros2_ws/src/cpp_pubsub` with the new ones in `21-.../cpp_pubsub` 

2. Replace the `add_two_ints_client.cpp` and `add_two_ints_server.cpp` from `~/ros2_ws/src/cpp_srvcli` with the new ones in `21-.../cpp_srvcli` 

3. Build package 
```bash
cd ~/ros2_ws
colcon build --packages-select cpp_srvcli
```

4. Test out pubsub using two terminals 
```bash
# Terminal 1 
cd ~/ros2_ws
source install/setup.bash
ros2 run cpp_srvcli server

# Terminal 2 
cd ~/ros2_ws
source install/setup.bash
ros2 run cpp_srvcli client 2 3 1
```

# Next Video:<br>Integrated Interface Package