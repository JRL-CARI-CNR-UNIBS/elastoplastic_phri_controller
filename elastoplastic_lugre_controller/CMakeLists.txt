cmake_minimum_required(VERSION 3.16)
project(elastoplastic_lugre_controller)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(BUILD_TESTING ON)

option(USE_LATEST_ROS2_CONTROL "Use Jazzy and later versions of ros2_control" ON)

find_package(ament_cmake                REQUIRED)
find_package(controller_interface       REQUIRED)
find_package(hardware_interface         REQUIRED)
find_package(control_toolbox            REQUIRED)
find_package(realtime_tools             REQUIRED)
find_package(pluginlib                  REQUIRED)
find_package(rclcpp                     REQUIRED)
find_package(rclcpp_lifecycle           REQUIRED)
find_package(sensor_msgs                REQUIRED)
find_package(nav_msgs                   REQUIRED)
find_package(geometry_msgs              REQUIRED)
find_package(moveit_msgs            REQUIRED)
find_package(tf2                        REQUIRED)
find_package(tf2_eigen                  REQUIRED)
find_package(urdfdom                    REQUIRED)
find_package(generate_parameter_library REQUIRED)

find_package(fmt                        REQUIRED)
find_package(rdyn_core                  REQUIRED)
find_package(eiquadprog                 REQUIRED)
find_package(Eigen3 3.4                 REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

generate_parameter_library(
  elastoplastic_parameters # cmake target name for the parameter library
  src/${PROJECT_NAME}/elastoplastic_controller_parameters.yaml # path to input yaml file
)

add_library(elastoplastic_controller SHARED
  src/${PROJECT_NAME}/elastoplastic_lugre_controller.cpp
  include/${PROJECT_NAME}/elastoplastic_lugre_controller.hpp
  include/elastoplastic_lugre_controller/utils.hpp
  # Cartesian interpolator
  src/${PROJECT_NAME}/interpolation/interpolator.cpp
  include/${PROJECT_NAME}/interpolation/interpolator.hpp
  include/${PROJECT_NAME}/interpolation/interpolation_data_types.hpp
  include/${PROJECT_NAME}/interpolation/interpolation.hpp
  include/${PROJECT_NAME}/interpolation/operators.hpp

  include/elastoplastic_lugre_controller/elastoplastic_variable_model.hpp
  src/elastoplastic_lugre_controller/elastoplastic_variable_model.cpp

  include/elastoplastic_lugre_controller/sot.hpp
)
add_library(elastoplastic_controller::elastoplastic_controller ALIAS elastoplastic_controller)

pluginlib_export_plugin_description_file(controller_interface elastoplastic.plugin.xml)

if(USE_LATEST_ROS2_CONTROL)
  target_compile_definitions(elastoplastic_controller PUBLIC USE_LATEST_ROS2_CONTROL)
endif()

ament_target_dependencies(elastoplastic_controller PUBLIC
  controller_interface
  hardware_interface
  control_toolbox
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  nav_msgs
  geometry_msgs
  moveit_msgs
  tf2
  tf2_eigen
  urdfdom
  realtime_tools
  )

target_link_libraries(elastoplastic_controller
  PUBLIC elastoplastic_parameters
  PUBLIC rdyn_core::rdyn_core_compiled
  PUBLIC Eigen3::Eigen
  PUBLIC fmt::fmt
  PUBLIC eiquadprog::eiquadprog
)

target_include_directories(elastoplastic_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

 # -ffast-math option is not added due to the std::isinf() function present in the code
target_compile_options(elastoplastic_controller PUBLIC -Wno-unknown-pragmas -funroll-loops -Wall -O3)
target_compile_definitions(elastoplastic_controller PUBLIC EIGEN_NO_AUTOMATIC_RESIZING)

install(TARGETS elastoplastic_controller elastoplastic_parameters
        EXPORT export_elastoplastic_controller
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_elastoplastic_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_eigen
  urdfdom
  realtime_tools
  control_toolbox
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)

  ament_add_gtest(test_elastoplastic_model
      test/test_elastoplastic_model.cpp)
  target_include_directories(test_elastoplastic_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_elastoplastic_model
    elastoplastic_controller::elastoplastic_controller
  )
  ament_target_dependencies(test_elastoplastic_model
    rclcpp
    rclcpp_lifecycle
  )
  target_compile_definitions(elastoplastic_controller PUBLIC "BUILD_TESTING")

  # ament_add_gtest(test_elastoplastic_controller
  #     test/test_elastoplastic_controller.cpp
  #   )
  # target_include_directories(test_elastoplastic_controller PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  # )
  # target_link_libraries(test_elastoplastic_controller
  #   elastoplastic_controller::elastoplastic_controller
  # )
  # ament_target_dependencies(test_elastoplastic_controller
  #   rclcpp
  #   rclcpp_lifecycle
  # )


  # add_rostest_with_parameters_gtest(test_with_cm
  #   test/test_with_cm.cpp
  #   ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml
  # )
  # target_link_libraries(test_with_cm elastoplastic_controller::elastoplastic_controller)
  # ament_target_dependencies(test_with_cm
  #   control_msgs
  #   controller_interface
  #   hardware_interface
  #   controller_manager
  # )

  install(DIRECTORY test
          DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
