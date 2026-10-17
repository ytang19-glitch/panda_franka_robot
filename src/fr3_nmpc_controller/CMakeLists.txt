cmake_minimum_required(VERSION 3.16)
project(fr3_nmpc_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pinocchio REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/robot_model.cpp src/optimizer.cpp src/solver_worker.cpp src/controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  pinocchio
)

target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)

pluginlib_export_plugin_description_file(controller_interface fr3_nmpc_controller_plugins.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_unimplemented_backends test/test_unimplemented_backends.cpp)
  target_link_libraries(test_unimplemented_backends ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
