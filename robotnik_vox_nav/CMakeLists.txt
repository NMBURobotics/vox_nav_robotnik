cmake_minimum_required(VERSION 3.5)
project(robotnik_vox_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
 
install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME})

ament_package()