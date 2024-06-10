cmake_minimum_required(VERSION 3.16)
project(elastoplastic_lugre_controller)
add_compile_options(-std=c++20 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake              REQUIRED)
find_package(controller_interface     REQUIRED)
find_package(hardware_interface       REQUIRED)
find_package(pluginlib                REQUIRED)
find_package(rclcpp                   REQUIRED)
find_package(rclcpp_lifecycle         REQUIRED)
find_package(sensor_msgs              REQUIRED)
find_package(geometry_msgs            REQUIRED)
find_package(tf2                      REQUIRED)
# find_package(rdyn_core                REQUIRED)

add_library(elastoplastic_controller SHARED
  src/${PROJECT_NAME}/elastoplastic_lugre_controller.cpp
  include/${PROJECT_NAME}/elastoplastic_lugre_controller.hpp
  src/elastoplastic_lugre_controller/elastoplastic_controller_parameters.yaml
)

ament_target_dependencies(elastoplastic_controller PUBLIC
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  tf2
  # rdyn_core
  )

target_include_directories(elastoplastic_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS elastoplastic_controller
        EXPORT export_elastoplastic_controller
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_elastoplastic_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  tf2
  # rdyn_core
)

ament_package()
