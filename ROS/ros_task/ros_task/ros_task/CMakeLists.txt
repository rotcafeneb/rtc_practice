cmake_minimum_required(VERSION 3.8)
project(ros_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ros_interfaces REQUIRED)

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp builtin_interfaces)

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp builtin_interfaces)

add_executable(service src/service.cpp)
ament_target_dependencies(service rclcpp ros_interfaces)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp ros_interfaces)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp ros_interfaces rclcpp_action rclcpp_components)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp ros_interfaces rclcpp_action rclcpp_components)

install(TARGETS
  talker
  listener
  service
  client
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
