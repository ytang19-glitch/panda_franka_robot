cmake_minimum_required(VERSION 3.16)
project(fr3_nmpc_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# Numerical model backend: loading a URDF does not establish hardware validity.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
find_package(Threads REQUIRED)

# Keep model-only builds possible while CasADi is being installed.
option(FR3_NMPC_WITH_CASADI "Build offline CasADi/IPOPT optimizer" OFF)
if(FR3_NMPC_WITH_CASADI)
  find_package(casadi CONFIG REQUIRED)
endif()

add_library(${PROJECT_NAME} SHARED
  src/robot_model.cpp
  src/optimizer.cpp
  src/solver_worker.cpp
  src/controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Match the plain linking signature used by ament_target_dependencies.
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  pinocchio::pinocchio
  Threads::Threads
)

if(FR3_NMPC_WITH_CASADI)
  target_compile_definitions(${PROJECT_NAME} PRIVATE FR3_NMPC_WITH_CASADI)
  target_link_libraries(${PROJECT_NAME} casadi::casadi)
  ament_export_dependencies(casadi)
endif()

add_executable(check_model examples/check_model.cpp)
target_link_libraries(check_model ${PROJECT_NAME})
install(TARGETS check_model DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(
  controller_interface
  fr3_nmpc_controller_plugins.xml
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_unimplemented_backends
    test/test_unimplemented_backends.cpp
  )

  if(TARGET test_unimplemented_backends)
    target_link_libraries(test_unimplemented_backends
      ${PROJECT_NAME}
    )
  endif()
  # Scheduling tests inject a fake solver and do not need CasADi or hardware.
  add_executable(test_solver_worker test/test_solver_worker.cpp)
  target_link_libraries(test_solver_worker ${PROJECT_NAME} Threads::Threads)
  add_test(NAME test_solver_worker COMMAND test_solver_worker)
  ament_add_gtest(test_robot_model test/test_robot_model.cpp)
  if(TARGET test_robot_model)
    target_link_libraries(test_robot_model ${PROJECT_NAME})
    target_compile_definitions(test_robot_model PRIVATE
      TEST_MODEL_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/seven_joint_test.urdf")
  endif()
  if(FR3_NMPC_WITH_CASADI)
    ament_add_gtest(test_optimizer test/test_optimizer.cpp)
    if(TARGET test_optimizer)
      target_link_libraries(test_optimizer ${PROJECT_NAME})
      target_compile_definitions(test_optimizer PRIVATE
        TEST_MODEL_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/seven_joint_test.urdf")
    endif()
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3
  pinocchio
  Threads
)

ament_package()
