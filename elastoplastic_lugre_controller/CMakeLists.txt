cmake_minimum_required(VERSION 3.16)
project(elastoplastic_lugre_controller)
add_compile_options(-funroll-loops -Wall -Ofast)
# set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 20)

set(BUILD_TESTING ON)

find_package(ament_cmake                REQUIRED)
find_package(controller_interface       REQUIRED)
find_package(hardware_interface         REQUIRED)
find_package(realtime_tools             REQUIRED)
find_package(pluginlib                  REQUIRED)
find_package(rclcpp                     REQUIRED)
find_package(rclcpp_lifecycle           REQUIRED)
find_package(sensor_msgs                REQUIRED)
find_package(nav_msgs                   REQUIRED)
find_package(geometry_msgs              REQUIRED)
find_package(tf2                        REQUIRED)
find_package(tf2_eigen                  REQUIRED)
find_package(urdfdom                    REQUIRED)
find_package(generate_parameter_library REQUIRED)

find_package(fmt                        REQUIRED)
find_package(rdyn_core                  REQUIRED)
find_package(Eigen3 3.4                 REQUIRED COMPONENTS Core)

generate_parameter_library(
  elastoplastic_parameters # cmake target name for the parameter library
  src/${PROJECT_NAME}/elastoplastic_controller_parameters.yaml # path to input yaml file
)


add_library(elastoplastic_controller SHARED
  src/${PROJECT_NAME}/elastoplastic_lugre_controller.cpp
  include/${PROJECT_NAME}/elastoplastic_lugre_controller.hpp
  test/test_params.yaml
)
add_library(elastoplastic_controller::elastoplastic_controller ALIAS elastoplastic_controller)

pluginlib_export_plugin_description_file(controller_interface elastoplastic.plugin.xml)

ament_target_dependencies(elastoplastic_controller PUBLIC
  controller_interface
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
  )

target_link_libraries(elastoplastic_controller
  PUBLIC elastoplastic_parameters
  PUBLIC rdyn_core::rdyn_core
  PUBLIC Eigen3::Eigen
  PUBLIC fmt::fmt)

target_include_directories(elastoplastic_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

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
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)

  ament_add_gtest(test_elastoplastic_controller
      test/test_elastoplastic_controller.cpp
    )
  target_include_directories(test_elastoplastic_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_elastoplastic_controller
    elastoplastic_controller::elastoplastic_controller
  )
  ament_target_dependencies(test_elastoplastic_controller
    rclcpp
    rclcpp_lifecycle
  )


  add_rostest_with_parameters_gtest(test_with_cm
    test/test_with_cm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml
  )
  target_link_libraries(test_with_cm elastoplastic_controller::elastoplastic_controller)
  ament_target_dependencies(test_with_cm
    control_msgs
    controller_interface
    hardware_interface
    controller_manager
  )
endif()

ament_package()
