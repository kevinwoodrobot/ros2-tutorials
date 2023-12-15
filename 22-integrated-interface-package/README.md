# Integrated Interface Package
In this video, we will go over how to make an integrated interface package. This is good if you want to combine a custom interface with you package containing other things (i.e. pubsub). 

1. Create Package and Make Msg Folder 
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake more_interfaces
mkdir more_interfaces/msg
```

2. Create `AddressBook.msg` file inside `.../more_interfaces/msg`
```bash
uint8 PHONE_TYPE_HOME=0
uint8 PHONE_TYPE_WORK=1
uint8 PHONE_TYPE_MOBILE=2

string first_name
string last_name
string phone_number
uint8 phone_type
```

3. Replace the `CMakesLists.txt` and `package.xml` from `~/ros2_ws/src/more_interfaces` with the new ones in `22-...` 

4. Add the file `publish_address_book.cpp` to `~/ros2_ws/src/more_interfaces/src`

5. Test it out 
```bash
cd ~/ros2_ws
colcon build --packages-up-to more_interfaces
source install/local_setup.bash
ros2 run more_interfaces publish_address_book
```

6. Verify topic is published to the `address_book` topic in another terminal 
```bash
source install/setup.bash
ros2 topic echo /address_book
```

# Next Video:<br>Parameter Package C++ 