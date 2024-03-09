cmake_minimum_required(VERSION 3.8)
project(gazebo_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ADDED---------------------------------------------------
# 1. Find Packages 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

# 2. Create Executable for Joint Publisher
add_executable(joint_publisher src/joint_publisher.cpp)
ament_target_dependencies(joint_publisher rclcpp trajectory_msgs)

# 3. Copy Files 
install(TARGETS
  joint_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    launch 
    urdf 
    config
    meshes
  DESTINATION share/${PROJECT_NAME}
)
# END ADDED---------------------------------------------------


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()