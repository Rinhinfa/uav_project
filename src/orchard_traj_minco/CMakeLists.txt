cmake_minimum_required(VERSION 3.8)
project(orchard_traj_minco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(traj_optimizer_node src/traj_optimizer_node.cpp)
ament_target_dependencies(
  traj_optimizer_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_srvs
)

install(TARGETS traj_optimizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
