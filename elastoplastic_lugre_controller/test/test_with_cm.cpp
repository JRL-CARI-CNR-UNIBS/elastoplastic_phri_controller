#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "test_urdf.hpp"

#include <algorithm>
#include <fmt/printf.h>

std::vector<rclcpp::Parameter> getOverrideParameters(){
  return {
      {"target_joint_trajectory_topic", "/joints_target"},
      {"ft_sensor_name", "tcp_ft_sensor"},
      {"frames.base", "az_base_footprint"},
      {"frames.tool", "az_robotiq_ft_frame_id"},
      {"frames.sensor", "az_ft300s_sensor"},
      {"joints", std::vector<std::string>{"az_shoulder_pan_joint",
                                          "az_shoulder_lift_joint",
                                          "az_elbow_joint",
                                          "az_wrist_1_joint",
                                          "az_wrist_2_joint",
                                          "az_wrist_3_joint"}},
      {"command_interfaces", std::vector<std::string>{hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY}},
      {"state_interfaces", std::vector<std::string>{hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY}},
      {"floating_base.enabled", false},
      {"floating_base.input_target_topic", "/reference_cmd_vel"},
      {"floating_base.odom","/odom"},
      {"wrench_deadband",std::vector<double>{1.0, 0.0}},
      {"impedance.inertia", std::vector<double>{10.0, 10.0, 10.0}},
      {"impedance.lugre.sigma_0", 1000.0},
      {"impedance.lugre.sigma_1", 10.0},
      {"impedance.lugre.sigma_2", 10.0},
      {"impedance.lugre.z_ss", 0.5},
      {"impedance.lugre.z_ba", 0.1},
      {"impedance.tau_w", 0.02},
      {"impedance.reset_condition.reset_window_size", 1000},
      {"impedance.reset_condition.reset_threshold", 0.1}
  };
}

TEST(ElastoplasticLoadOnCM, load_controller)
{
  rclcpp::Node dummy = rclcpp::Node("dummy");

  std::shared_ptr<rclcpp::Executor> exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions opt;
  opt.parameter_overrides(getOverrideParameters());

  controller_manager::ControllerManager cm(
      std::make_unique<hardware_interface::ResourceManager>(
          test_urdf),
      exec,
      "test_controller_manager");

  cm.set_parameter({"elastoplastic_controller.type", "elastoplastic_controller/ElastoplasticController"});

  // rclcpp::ParameterMap map = rclcpp::parameter_map_from_yaml_file("test_params.yaml");

  std::vector<std::string> nodes = dummy.get_node_names();
  ASSERT_TRUE(std::ranges::find(nodes, "/test_controller_manager") != nodes.end()); // Just to be sure

  EXPECT_EQ(cm.load_controller("elastoplastic_controller"), nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
