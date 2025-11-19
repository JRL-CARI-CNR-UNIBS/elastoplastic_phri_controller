cmake_minimum_required(VERSION 3.20)
project(elastoplastic_lugre_controller)
if(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE Debug)
endif()
# set(CMAKE_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(BUILD_TESTING OFF)

option(BUILD_ADAPTIVE_HQP "Build adaptive pseudo-admittance controller" ON)
option(USE_LATEST_ROS2_CONTROL "Use Jazzy and later versions of ros2_control" ON)


find_package(ament_cmake                 REQUIRED)
find_package(backward_ros                REQUIRED)

find_package(controller_interface        REQUIRED)
find_package(hardware_interface          REQUIRED)
find_package(filters 2.2.2               REQUIRED)
find_package(control_toolbox             REQUIRED)
find_package(control_msgs                REQUIRED)
find_package(realtime_tools              REQUIRED)
find_package(pluginlib                   REQUIRED)
find_package(rclcpp                      REQUIRED)
find_package(rclcpp_lifecycle            REQUIRED)
find_package(elastoplastic_msgs          REQUIRED)
find_package(nav_msgs                    REQUIRED)
find_package(sensor_msgs                 REQUIRED)
find_package(geometry_msgs               REQUIRED)
find_package(moveit_msgs                 REQUIRED)
find_package(tf2                         REQUIRED)
find_package(tf2_eigen                   REQUIRED)
find_package(urdfdom                     REQUIRED)
find_package(generate_parameter_library  REQUIRED)

find_package(pal_statistics              REQUIRED)
find_package(pal_statistics_msgs         REQUIRED)

find_package(fmt                         REQUIRED)
find_package(rdyn_core                   REQUIRED)
find_package(eiquadprog                  REQUIRED)
find_package(Eigen3 3.4                  REQUIRED)

find_package(state_observers             REQUIRED)
find_package(cnr_control_toolbox         REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

generate_parameter_library(
  elastoplastic_parameters # cmake target name for the parameter library
  src/${PROJECT_NAME}/elastoplastic_controller_parameters.yaml # path to input yaml file
)
generate_parameter_library(
  elastoplastic_dual_parameters # cmake target name for the parameter library
  src/${PROJECT_NAME}/elastoplastic_controller_dual_parameters.yaml # path to input yaml file
)

set(COMMON_FILES
  include/${PROJECT_NAME}/utils.hpp
  # Cartesian interpolator
  src/${PROJECT_NAME}/interpolation/interpolator.cpp
  include/${PROJECT_NAME}/interpolation/interpolator.hpp
  include/${PROJECT_NAME}/interpolation/interpolation_data_types.hpp
  include/${PROJECT_NAME}/interpolation/interpolation.hpp
  include/${PROJECT_NAME}/interpolation/operators.hpp
  include/${PROJECT_NAME}/elastoplastic_variable_model.hpp
  src/${PROJECT_NAME}/elastoplastic_variable_model.cpp
  include/${PROJECT_NAME}/sot.hpp)

add_library(elastoplastic_controller SHARED
  src/${PROJECT_NAME}/elastoplastic_lugre_controller.cpp
  src/${PROJECT_NAME}/clik.cpp
  include/${PROJECT_NAME}/elastoplastic_lugre_controller.hpp
  ${COMMON_FILES}
)

add_library(elastoplastic_controller_dual SHARED
  src/${PROJECT_NAME}/elastoplastic_lugre_controller_dual.cpp
  src/${PROJECT_NAME}/clik_dual.cpp
  include/${PROJECT_NAME}/elastoplastic_lugre_controller_dual.hpp
  ${COMMON_FILES}
)

add_library(elastoplastic_controller::elastoplastic_controller ALIAS elastoplastic_controller)
add_library(elastoplastic_controller::elastoplastic_controller_dual ALIAS elastoplastic_controller_dual)

pluginlib_export_plugin_description_file(controller_interface elastoplastic.plugin.xml)

if(USE_LATEST_ROS2_CONTROL)
  target_compile_definitions(elastoplastic_controller PUBLIC USE_LATEST_ROS2_CONTROL)
  target_compile_definitions(elastoplastic_controller_dual PUBLIC USE_LATEST_ROS2_CONTROL)
endif()

set(AMENT_LIBS
  controller_interface
  hardware_interface
  filters
  control_toolbox
  realtime_tools
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  nav_msgs
  elastoplastic_msgs
  geometry_msgs
  moveit_msgs
  tf2
  tf2_eigen
  urdfdom
  state_observers
  pal_statistics_msgs
  pal_statistics
  )

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set (RDYN_CONDITIONAL_TARGET rdyn_core::rdyn_core_compiled)
else()
  set (RDYN_CONDITIONAL_TARGET rdyn_core::rdyn_core)
endif()

target_link_libraries(elastoplastic_controller
    PUBLIC elastoplastic_parameters
    PUBLIC ${RDYN_CONDITIONAL_TARGET}
    PUBLIC Eigen3::Eigen
    PUBLIC fmt::fmt
    PUBLIC eiquadprog::eiquadprog
    # libcontroller_interface.so
    # libhardware_interface.so
    #libpal_statistics.so
    # librealtime_tools.so
  )

# target_link_directories(elastoplastic_controller
  # PUBLIC "/opt/pal/alum/lib"
# )

ament_target_dependencies(elastoplastic_controller PUBLIC
${AMENT_LIBS}
)


ament_target_dependencies(elastoplastic_controller_dual PUBLIC
  ${AMENT_LIBS}
)

target_link_libraries(elastoplastic_controller_dual
  PUBLIC elastoplastic_dual_parameters
  PUBLIC ${RDYN_CONDITIONAL_TARGET}
  PUBLIC Eigen3::Eigen
  PUBLIC fmt::fmt
  PUBLIC eiquadprog::eiquadprog
)

target_include_directories(elastoplastic_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  # "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../ros2_control/controller_interface/include>"
  # "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../ros2_control/hardware_interface/include>"
  # "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../ros2_control/joint_limits/include>"
  # "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE\_DIR}/../../pal_statistics/pal_statistics/include>"
  )

target_include_directories(elastoplastic_controller_dual PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

 # -ffast-math option is not added due to the std::isinf() function present in the code
# target_compile_options(elastoplastic_controller PUBLIC -Wno-unknown-pragmas -funroll-loops -Wall -O3)
target_compile_definitions(elastoplastic_controller PUBLIC EIGEN_NO_AUTOMATIC_RESIZING)
target_compile_definitions(elastoplastic_controller_dual PUBLIC EIGEN_NO_AUTOMATIC_RESIZING)

if(BUILD_ADAPTIVE_HQP)
  generate_parameter_library(
    adaptive_hqp_parameters # cmake target name for the parameter library
    src/adaptive_hqp/adaptive_hqp_parameters.yaml # path to input yaml file
  )

  find_package(pinocchio REQUIRED)
  add_library(adaptive_hqp SHARED
    include/adaptive_hqp/adaptive_hqp.hpp
    src/adaptive_hqp/adaptive_hqp.cpp
    src/adaptive_hqp/hqp.cpp
    ${COMMON_FILES})
  add_library(adaptive_hqp::adaptive_hqp ALIAS adaptive_hqp)

  ament_target_dependencies(adaptive_hqp PUBLIC
  ${AMENT_LIBS}
  )
  
  target_link_libraries(adaptive_hqp PUBLIC
    adaptive_hqp_parameters
    pinocchio::pinocchio
    PUBLIC ${RDYN_CONDITIONAL_TARGET}
    PUBLIC Eigen3::Eigen
    PUBLIC fmt::fmt
    PUBLIC eiquadprog::eiquadprog
    PUBLIC state_space_filters::state_space_filters
  )

  target_include_directories(adaptive_hqp PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

  set(ADAPTIVE_HQP_TARGETS adaptive_hqp adaptive_hqp_parameters)
  pluginlib_export_plugin_description_file(controller_interface others.plugin.xml)
endif()

install(TARGETS 
  elastoplastic_controller 
  elastoplastic_parameters 
  elastoplastic_controller_dual 
  elastoplastic_dual_parameters
  ${ADAPTIVE_HQP_TARGETS}
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
ament_export_dependencies(#controller_interface
  # hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_eigen
  urdfdom
  #realtime_tools
  control_toolbox
  state_observers
  elastoplastic_msgs
)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # ament_add_gtest(test_elastoplastic_model
  #     test/test_elastoplastic_model.cpp)
  # target_include_directories(test_elastoplastic_model PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  # )
  # target_link_libraries(test_elastoplastic_model
  #   elastoplastic_controller::elastoplastic_controller
  # )
  # ament_target_dependencies(test_elastoplastic_model
  #   rclcpp
  #   rclcpp_lifecycle
  # )
  # target_compile_definitions(elastoplastic_controller PUBLIC "BUILD_TESTING")

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_cm_elastoplastic_controller
    test/test_cm_elastoplastic_controller.cpp
  )
  target_link_libraries(test_cm_elastoplastic_controller
    elastoplastic_controller
    controller_manager::controller_manager
    controller_manager_msgs::controller_manager_msgs__rosidl_typesupport_cpp
    ros2_control_test_assets::ros2_control_test_assets
  )

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
