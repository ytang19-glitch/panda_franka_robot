cmake_minimum_required(VERSION 3.8)
project(panda_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_executable(reference_tracker
  src/reference_tracker.cpp
  src/cost_functions/joint_cost.cpp
  src/cost_functions/singularity_cost.cpp
  src/cost_functions/collision_cost.cpp
  src/cost_functions/smoothness_cost.cpp
  src/optimization/weighted_ik.cpp
  src/optimization/trajectory_optimizer.cpp
  src/trajectory/trajectory_validator.cpp
  src/trajectory/time_parameterization.cpp)
target_compile_features(reference_tracker PUBLIC cxx_std_17)
target_include_directories(reference_tracker PRIVATE include)
target_link_libraries(reference_tracker Eigen3::Eigen Threads::Threads)
ament_target_dependencies(reference_tracker rclcpp geometry_msgs nav_msgs trajectory_msgs moveit_msgs moveit_core moveit_ros_planning_interface)
install(TARGETS reference_tracker DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
