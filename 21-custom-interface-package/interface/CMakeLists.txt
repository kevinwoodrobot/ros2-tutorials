cmake_minimum_required(VERSION 3.8)
project(tutorial_interfaces)


find_package(ament_cmake REQUIRED)

# ADDED --------------------------------------
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
# END ADDED-----------------------------------

ament_package()
