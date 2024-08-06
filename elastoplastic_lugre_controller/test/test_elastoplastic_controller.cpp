#include "test_urdf.hpp"
#include "test_elastoplastic_controller.hpp"

#include "controller_interface/controller_interface_base.hpp"

#include "rclcpp_lifecycle/state.hpp"

// #include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "fmt/format.h"

#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>


TEST_F(ElastoplasticControllerTest, init_controller)
{
  ASSERT_EQ(SetUpController(), controller_interface::return_type::OK);
}


TEST_F(ElastoplasticControllerTest, configure_controller)
{
  SetUpController();

  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
}


TEST_F(ElastoplasticControllerTest, check_interfaces)
{
  SetUpController();

  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);

  auto command_interfaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_interfaces.names.size(), command_interfaces_names_.size() * joints_.size());

  auto state_interfaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_interfaces.names.size(), state_interfaces_names_.size() * joints_.size() + 6);
}


TEST_F(ElastoplasticControllerTest, active_controller)
{
  SetUpController();

  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);
}


TEST_F(ElastoplasticControllerTest, export_reference_interfaces)
{
  SetUpController();

  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * command_interfaces_names_.size());
}

// TEST_F(ElastoplasticControllerTest, unchained_control)
// {
  // SetUpController();

  // controller_->get_node()->declare_parameter("robot_description", test_urdf);
  // ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  // ASSERT_EQ(controller_->set_chained_mode(false), true);
  // ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  // auto dummy_node = std::make_shared<rclcpp::Node>("dummy_node");

  // auto pub_ft_data = dummy_node->create_publisher<geometry_msgs::msg::Twist>("/input_cmd_vel",1);
  // auto pub_odom_data = dummy_node->create_publisher<nav_msgs::msg::Odometry>("/odom",1);
// }


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
