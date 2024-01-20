# Components
In this ros2 tutorial, I will go over ros components, which are used to load nodes at runtime. I'll go over runtime composition with pub/sub and server/client examples, compile-time composition, runtime composition with dlopen, composition using launch files, and special component operations like unloading and remapping. 

In ros1, this concept was called ros nodelets. 

```bash
ros2 component types
```

## Runtime Composition with Pub/Sub
1. Start `/ComponentManager`
```bash
ros2 run rclcpp_components component_container
```

2. Verify `/ComponentManager` started 
```bash
ros2 component list
```

3. Start the `talker` and `listner` components
```bash
ros2 component load /ComponentManager composition composition::Talker
ros2 component load /ComponentManager composition composition::Listener
```

4. See the `talker` and `listner` components show up
```bash
ros2 component list
```

## Runtime Composition with Server/Client
1. Start `/ComponentManager`
```bash
ros2 run rclcpp_components component_container
```

2. Start server and client
```bash
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```

## Compile-time Composition
```bash
ros2 run composition manual_composition
```

## Runtime Composition using dlopen
Here we use `.so` or `shared object` files, which are dynamically linked during runtime, to create our composition wihout using the `ros2 component load` method from earlier
```bash
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```

## Composition using Launch Files
```bash
ros2 launch composition composition_demo.launch.py
```

## Unload Components 
1. Start `/ComponentManager`
```bash
ros2 run rclcpp_components component_container
```

2. Load `talker` and `listener`
```bash
ros2 component load /ComponentManager composition composition::Talker
ros2 component load /ComponentManager composition composition::Listener
```

3. Now unload 
```bash
ros2 component unload /ComponentManager 1 2
```

## Remap Container Name and Namespace
1. Start container with custom name and namespace
```bash
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns
```

2. See new container name 
```bash
ros2 component list
```

# Reference 
https://github.com/ros2/demos/tree/humble/composition

# Next Video:<br>Monitor Parameter Change