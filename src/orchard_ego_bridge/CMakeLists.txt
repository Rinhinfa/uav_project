cmake_minimum_required(VERSION 3.8)
project(orchard_ego_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ego_local_planner_stub_node src/ego_local_planner_stub_node.cpp)
ament_target_dependencies(ego_local_planner_stub_node geometry_msgs nav_msgs rclcpp std_msgs)

add_executable(planning_arbitrator_node src/planning_arbitrator_node.cpp)
ament_target_dependencies(planning_arbitrator_node geometry_msgs nav_msgs rclcpp std_msgs)

install(
  TARGETS ego_local_planner_stub_node planning_arbitrator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
