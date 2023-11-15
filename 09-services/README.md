# ROS Services
Services are used to communicate between nodes using a client-server model, where the server respondes when the client make a request. 

1. In a new terminal, run 
```bash 
ros2 run turtlesim turtlesim_node
```

2. In another terminal, run 
```bash
ros2 run turtlesim turtle_teleop_key
```

3. List all service names 
```bash
ros2 service list

# See service type 
ros2 service list -t
```

4. View service type another way
```bash
ros2 service type <service_name>
ros2 service type /clear
```

5. Find service with specific type 
```bash
ros2 service find <service_type>
ros2 service find std_srvs/srv/Empty
```

6. See interface 
```bash
ros2 interface show <service_type>
ros2 interface show turtlesim/srv/Spawn  
```

7. Calling a service 
```bash
ros2 service call <service_name> <service_type> <arguments>

# Clear drawing
ros2 service call /clear std_srvs/srv/Empty

# Spawn turtle 
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

# Next Video:<br>Parameters