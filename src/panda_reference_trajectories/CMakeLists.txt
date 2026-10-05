cmake_minimum_required(VERSION 3.8)
project(panda_reference_trajectories)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# MoveIt dependencies
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# TF dependencies
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# ============================================================
# Reference trajectory generator
# ============================================================

add_executable(reference_generator
  src/reference_generator_node.cpp
  src/trajectory_builder.cpp
  src/generators/point_to_point.cpp
  src/generators/linear_path.cpp
  src/generators/arc_path.cpp
  src/generators/raster_path.cpp
  src/generators/spline_path.cpp
  src/generators/approach_retract.cpp
)

target_compile_features(reference_generator PUBLIC cxx_std_17)

target_include_directories(reference_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(reference_generator
  rclcpp
  geometry_msgs
  nav_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)


# ============================================================
# Cartesian path executor
# ============================================================

add_executable(cartesian_path_executor
  src/cartesian_path_executor.cpp
)

target_compile_features(cartesian_path_executor PUBLIC cxx_std_17)

target_include_directories(cartesian_path_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cartesian_path_executor
  rclcpp
  geometry_msgs
  nav_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)


# ============================================================
# Installation
# ============================================================

install(TARGETS
  reference_generator
  cartesian_path_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()