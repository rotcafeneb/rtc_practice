cmake_minimum_required(VERSION 3.8)
project(ros_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros_interfaces REQUIRED)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp ros_interfaces rclcpp_action rclcpp_components)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp ros_interfaces rclcpp_action rclcpp_components)

target_include_directories(
    action_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(
    action_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
