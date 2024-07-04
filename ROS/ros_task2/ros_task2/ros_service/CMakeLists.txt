cmake_minimum_required(VERSION 3.8)
project(ros_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_interfaces REQUIRED)

add_executable(service src/service.cpp)
ament_target_dependencies(service rclcpp ros_interfaces)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp ros_interfaces)

target_include_directories(
    service
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    
target_include_directories(
    client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  service
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
