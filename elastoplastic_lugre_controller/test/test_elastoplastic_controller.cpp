#include "test_urdf.hpp"
#include "test_elastoplastic_controller.hpp"

#include "controller_interface/controller_interface_base.hpp"

// #include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "fmt/format.h"

#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


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
  EXPECT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);
}


TEST_F(ElastoplasticControllerTest, export_reference_interfaces)
{
  SetUpController();

  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(export_reference_interfaces().size(), joints_.size() * command_interfaces_names_.size());
}

TEST_F(ElastoplasticControllerTest, unchained_control_no_base_no_force)
{
  SetUpController();

  set_initial_position();
  std::ranges::fill(fts_state_values_, 0.0);

  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * command_interfaces_names_.size());
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::for_each(joint_command_values_.begin(),
      std::next(joint_command_values_.begin(), joints_.size()),
      [](const double val){EXPECT_EQ(val, INITIAL_POS);});

}


TEST_F(ElastoplasticControllerTest, unchained_control_no_base_use_force)
{
  SetUpController();

  set_initial_position();
  fts_state_values_.at(0) = 100.0;
  fts_state_values_.at(1) = 100.0;
  fts_state_values_.at(2) = 100.0;
  fts_state_values_.at(3) = 0.0;
  fts_state_values_.at(4) = 0.0;
  fts_state_values_.at(5) = 0.0;

  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * command_interfaces_names_.size());
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::for_each(joint_command_values_.begin(),
      std::next(joint_command_values_.begin(), joints_.size()),
      [](const double val){EXPECT_NE(val, INITIAL_POS);});
}

TEST_F(ElastoplasticControllerTest, unchained_control_base_no_force)
{
  SetUpController();

  set_initial_position();
  std::ranges::fill(fts_state_values_, 0.0);

  controller_->get_node()->set_parameter({"floating_base.enabled", true});
  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * command_interfaces_names_.size());
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::for_each(joint_command_values_.begin(),
      std::next(joint_command_values_.begin(), joints_.size()),
      [](const double val){EXPECT_EQ(val, INITIAL_POS);});
}

TEST_F(ElastoplasticControllerTest, unchained_control_base_force)
{
  SetUpController();

  set_initial_position();
  fts_state_values_.at(0) = 100.0;
  fts_state_values_.at(1) = 100.0;
  fts_state_values_.at(2) = 100.0;
  fts_state_values_.at(3) = 0.0;
  fts_state_values_.at(4) = 0.0;
  fts_state_values_.at(5) = 0.0;

  controller_->get_node()->set_parameter({"floating_base.enabled", true});
  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * command_interfaces_names_.size());
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::for_each(joint_command_values_.begin(),
      std::next(joint_command_values_.begin(), joints_.size()),
      [](const double val){EXPECT_NE(val, INITIAL_POS);});
}

TEST_F(ElastoplasticControllerTest, unchained_control_no_base_torque)
{
  SetUpController();

  set_initial_position();
  fts_state_values_.at(0) = 0.0;
  fts_state_values_.at(1) = 0.0;
  fts_state_values_.at(2) = 0.0;
  fts_state_values_.at(3) = 100.0;
  fts_state_values_.at(4) = 100.0;
  fts_state_values_.at(5) = 100.0;

  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * command_interfaces_names_.size());
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::for_each(joint_command_values_.begin(),
      std::next(joint_command_values_.begin(), joints_.size()),
      [](const double val){EXPECT_EQ(val, INITIAL_POS);});
}

TEST_F(ElastoplasticControllerTest, unchained_control_no_base_use_force_debug_publisher)
{
  SetUpController();

  set_initial_position();
  fts_state_values_.at(0) = 100.0;
  fts_state_values_.at(1) = 100.0;
  fts_state_values_.at(2) = 100.0;
  fts_state_values_.at(3) = 0.0;
  fts_state_values_.at(4) = 0.0;
  fts_state_values_.at(5) = 0.0;

  controller_->get_node()->set_parameter({"debug", true});
  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * command_interfaces_names_.size());
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto dummy_node = std::make_shared<rclcpp::Node>("dummy_node");
  auto sub_z = dummy_node->create_subscription<std_msgs::msg::Float64MultiArray>("/z", 1, [](const std_msgs::msg::Float64MultiArray::SharedPtr msg){RCLCPP_INFO(rclcpp::get_logger("test"), "Got message!");});

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::for_each(joint_command_values_.begin(),
      std::next(joint_command_values_.begin(), joints_.size()),
      [](const double val){EXPECT_NE(val, INITIAL_POS);});

  std_msgs::msg::Float64MultiArray msg_out; rclcpp::MessageInfo info;
  bool message_received {false};
  // rclcpp::spin_some(dummy_node);
  auto start = std::chrono::steady_clock::now();
  do
  {
    message_received = sub_z->take(msg_out, info);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while(!message_received && std::chrono::steady_clock::now() - start < std::chrono::seconds(10));
  EXPECT_TRUE(message_received);
  EXPECT_NE(msg_out.data.at(0), INITIAL_POS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
