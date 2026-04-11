cmake_minimum_required(VERSION 3.8)
project(orchard_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Placeholder package for future custom interfaces.
# Current implementation uses standard ROS2 messages/services for Jazzy stability.
ament_package()
