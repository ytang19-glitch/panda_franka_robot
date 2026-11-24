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

find_package(shape_msgs REQUIRED)
add_executable(collision_cost_test
  test/integration/test_moveit_collision_cost.cpp
  src/planning_scene/collision_distance.cpp
  src/cost_functions/collision_cost.cpp)
target_compile_features(collision_cost_test PUBLIC cxx_std_17)
target_include_directories(collision_cost_test PRIVATE include)
target_link_libraries(collision_cost_test Eigen3::Eigen Threads::Threads)
ament_target_dependencies(collision_cost_test rclcpp moveit_core moveit_msgs geometry_msgs shape_msgs moveit_ros_planning_interface)
install(TARGETS collision_cost_test DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_collision_cost test/unit/test_collision_cost.cpp src/cost_functions/collision_cost.cpp)
  target_compile_features(test_collision_cost PUBLIC cxx_std_17)
  target_include_directories(test_collision_cost PRIVATE include)
  target_link_libraries(test_collision_cost Eigen3::Eigen)
  add_test(NAME collision_cost_unit COMMAND test_collision_cost)
  install(TARGETS test_collision_cost DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
