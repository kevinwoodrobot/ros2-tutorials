cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# NEW ----------------------------------------------
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/AddressBook.msg"
)
# rosidl_generate_interfaces generates address_book.hpp
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
add_executable(publish_address_book src/publish_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp)
install(TARGETS
  publish_address_book
  DESTINATION lib/${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(publish_address_book "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)
# END NEW ----------------------------------------------

ament_package()
