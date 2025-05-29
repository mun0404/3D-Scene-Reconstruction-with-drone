cmake_minimum_required(VERSION 3.8)
project(trajectory)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(circular_path src/circular.cpp)
ament_target_dependencies(circular_path rclcpp px4_msgs)

install(TARGETS circular_path
  DESTINATION lib/${PROJECT_NAME})

ament_package()
