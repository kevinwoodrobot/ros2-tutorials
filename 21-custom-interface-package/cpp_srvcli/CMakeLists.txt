cmake_minimum_required(VERSION 3.8)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED) # NEW

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces) # NEW

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces) # NEW

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()


