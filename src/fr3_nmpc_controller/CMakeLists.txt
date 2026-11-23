cmake_minimum_required(VERSION 3.16)
project(fr3_nmpc_controller)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
find_package(Threads REQUIRED)
find_package(sensor_msgs REQUIRED)

option(FR3_NMPC_WITH_CASADI
  "Build offline CasADi/IPOPT optimizer"
  OFF
)

if(FR3_NMPC_WITH_CASADI)
  find_package(casadi CONFIG REQUIRED)
endif()

# Controller library
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
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE FR3_NMPC_WITH_CASADI
  )

  target_link_libraries(${PROJECT_NAME}
    casadi::casadi
  )

  ament_export_dependencies(casadi)
endif()

# Offline model checker
add_executable(check_model
  examples/check_model.cpp
)

target_compile_features(check_model PRIVATE cxx_std_17)
target_link_libraries(check_model PRIVATE ${PROJECT_NAME})

install(TARGETS check_model
  DESTINATION lib/${PROJECT_NAME}
)

# Offline NMPC tracking
if(FR3_NMPC_WITH_CASADI)
  add_executable(offline_tracking
    src/offline_tracking.cpp
  )

  target_compile_features(offline_tracking PRIVATE cxx_std_17)
  target_link_libraries(offline_tracking PRIVATE ${PROJECT_NAME})

  install(TARGETS offline_tracking
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ============================================================
# Gazebo NMPC Read-only Diagnostic
# ============================================================

if(FR3_NMPC_WITH_CASADI)

  add_executable(gazebo_nmpc_diagnostic
    src/gazebo_nmpc_diagnostic.cpp
  )

  target_compile_features(
    gazebo_nmpc_diagnostic
    PRIVATE cxx_std_17
  )

  target_link_libraries(
    gazebo_nmpc_diagnostic
    ${PROJECT_NAME}
  )

  ament_target_dependencies(
    gazebo_nmpc_diagnostic
    rclcpp
    sensor_msgs
  )

  install(
    TARGETS gazebo_nmpc_diagnostic
    DESTINATION lib/${PROJECT_NAME}
  )

endif()


# Controller plugin
pluginlib_export_plugin_description_file(
  controller_interface
  fr3_nmpc_controller_plugins.xml
)

# Installation
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

# Tests
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

  add_executable(test_solver_worker
    test/test_solver_worker.cpp
  )

  target_link_libraries(test_solver_worker
    ${PROJECT_NAME}
    Threads::Threads
  )

  add_test(
    NAME test_solver_worker
    COMMAND test_solver_worker
  )

  ament_add_gtest(test_robot_model
    test/test_robot_model.cpp
  )

  if(TARGET test_robot_model)
    target_link_libraries(test_robot_model
      ${PROJECT_NAME}
    )

    target_compile_definitions(test_robot_model PRIVATE
      TEST_MODEL_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/seven_joint_test.urdf"
    )
  endif()

  if(FR3_NMPC_WITH_CASADI)
    ament_add_gtest(test_optimizer
      test/test_optimizer.cpp
    )

    if(TARGET test_optimizer)
      target_link_libraries(test_optimizer
        ${PROJECT_NAME}
      )

      target_compile_definitions(test_optimizer PRIVATE
        TEST_MODEL_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/seven_joint_test.urdf"
      )
    endif()
  endif()
endif()

# Exports
ament_export_targets(export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

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


if(FR3_NMPC_WITH_CASADI)
  add_executable(check_callback_jacobian
    examples/check_callback_jacobian.cpp
  )

  target_compile_features(check_callback_jacobian PRIVATE cxx_std_17)

  target_link_libraries(check_callback_jacobian PRIVATE
    ${PROJECT_NAME}
    casadi::casadi
  )

  install(TARGETS check_callback_jacobian
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  add_executable(check_dynamics_derivatives
    test/check_dynamics_derivatives.cpp
  )

  target_link_libraries(check_dynamics_derivatives
    ${PROJECT_NAME}
  )

  install(TARGETS check_dynamics_derivatives
    DESTINATION lib/${PROJECT_NAME}
  )

  add_test(NAME dynamics_derivatives
    COMMAND check_dynamics_derivatives
      "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/seven_joint_test.urdf"
  )
endif()


ament_package()       
