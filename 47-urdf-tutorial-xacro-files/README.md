# URDF Tutorial Xacro Files (Simplify URDF with Xacro Property and Xacro Macro)
In this URDF tutorial, I will show you how to use xacro properties and xacro macros in your `.urdf.xacro` files. 

- [Xacro Command](#xacro-command)
- [Robot State Publisher](#robot-state-publisher)
- [URDF_Launch](#urdf_launch)
- [Xacro Property](#xacro-property)
- [Xacro Macro](#xacro-macro)
- [URDF using Xacros in Same File](#urdf-using-xacros-in-same-file)
- [URDF using Xacros in Separate Files](#urdf-using-xacros-in-separate-files)


## How to use Xacro
### 1. Xacro Command
Xacro is used to convert a `.xacro` file to a `.urdf` file. 
```bash
xacro model.xacro > model.urdf
```

### 2. Robot State Publisher
Use the `robot_state_publisher` to read the `.urdf` file in a launch file and use it as a `robot_description` parameter. Also, run the `xacro` command to convert the `.xacro` to a `.urdf` file. 

```python
path_to_urdf = ...
robot_state_publisher_node = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': ParameterValue(
            Command(['xacro ', str(path_to_urdf)]), value_type=str
        )
    }]
)
```

### 3. URDF_Launch
Use `urdf_launch` package to load the `.xacro` or `.urdf` file
```python
def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'turtlebot3_description',
            'urdf_package_path': PathJoinSubstitution(['urdf', 'model_name.urdf'])}.items()
    ))
    return ld
```

## Xacro Property
Xacro properties are useful when the same property shows up at multiple places. This allows you to change the value of all instance at one location. 

Property Declaration: 
```xml
<xacro:property name="property_name" value="..." />
```

Property Usage:
```xml
<... variable="${property_name}" .../>
```

## Xacro Macro
Macro's are used to replace several lines of xml code with the option of having input parameters. This is useful when several lines of code are repeated. 

Macro Declaration:
```xml
<xacro:macro name="macro_name" params="param-1 param-2 ... param-n">

    ...

</xacro:macro>
```

Macro Usage:
```xml
<xacro:macro_name param-1="value"/>
```

## URDF using Xacros in Same File
In `04-robot-example-xacro-internal.urdf.xacro`, we will see how properties and macros are used within the same file. After this, we will see how to move them to a separate file. 
```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/04-robot-example-xacro-internal.urdf.xacro
```

## URDF using Xacros in Separate Files
Now we can see how to use the xacros in separate files. This is useful when you may want to reuse the xacro file in another file later or to keep your urdf file smaller. 

Separate Xacro File:
```xml
<!-- File Name: filename.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name=" " value =" ">
  
  <xacro:macro ...>

  </xacro:macro>
</robot>
```

Using the Xacro File: 
```xml
<!-- File Name: filename.urdf.xacro -->
<?xml version="1.0"?>
<robot name="robot_name" xmlns:xacro="http://ros.org/wiki/xacro">

  ...

  <xacro:include filename="filename.xacro" />

  ...
  
</robot>
```

We can take a look at the `robot-parts.xacro` file and then confirm that running `05-robot-example-xacro-external.urdf.xacro` produces the same results as the previous example when using the xacro in the same file. 
```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/05-robot-example-xacro-external.urdf.xacro
```

# Next Video:<br>RVIZ Robot Simulation
