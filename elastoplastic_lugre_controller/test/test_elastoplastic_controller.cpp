#include "test_urdf.hpp"
#include "test_elastoplastic_controller.hpp"

#include "controller_interface/controller_interface_base.hpp"

// #include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "fmt/format.h"

#include <geometry_msgs/msg/wrench_stamped.hpp>
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
  EXPECT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(ElastoplasticControllerTest, robot_description_from_topic)
{
  SetUpController();

  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);

  auto dummy_node = rclcpp::Node("dummy_node");
  auto pub_rb = dummy_node.create_publisher<std_msgs::msg::String>("/robot_description", rclcpp::QoS(2).transient_local());
  std_msgs::msg::String rb;
  rb.data = test_urdf;
  pub_rb->publish(rb);

  rclcpp::spin_some(controller_->get_node()->get_node_base_interface());

  EXPECT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(controller_->get_node()->has_parameter("robot_description"));
}

// TEST_F(ElastoplasticControllerTest, multiple_robot_description_from_topic)
// {
//   SetUpController();

//   auto dummy_node = rclcpp::Node("dummy_node");
//   auto pub_rb = dummy_node.create_publisher<std_msgs::msg::String>("/elastoplastic_controller/robot_description", 1);
//   std_msgs::msg::String rb;
//   rb.data = test_urdf;
//   pub_rb->publish(rb);

//   ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
//   EXPECT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);
// }


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
  EXPECT_EQ(export_reference_interfaces().size(), joints_.size() * 2);
}

TEST_F(ElastoplasticControllerTest, unchained_control_no_base_no_force)
{
  SetUpController();

  set_initial_position();
  std::ranges::fill(fts_state_values_, 0.0);



  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * 2);
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  // print_joint();

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  for(size_t idx = 0; idx < joints_.size(); ++idx)
  {
    EXPECT_EQ(joint_command_values_.at(idx), initial_pos_.at(idx));
  }

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
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * 2);
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  for(size_t idx = 0; idx < joints_.size(); ++idx)
  {
    EXPECT_NE(joint_command_values_.at(idx), initial_pos_.at(idx));
  }
}

TEST_F(ElastoplasticControllerTest, unchained_control_base_no_force)
{
  SetUpController();

  set_initial_position();
  std::ranges::fill(fts_state_values_, 0.0);

  controller_->get_node()->set_parameter({"floating_base.enabled", true});
  controller_->get_node()->declare_parameter("robot_description", test_urdf);
  ASSERT_EQ(configureController(), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * 2);
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  for(size_t idx = 0; idx < joints_.size(); ++idx)
  {
    EXPECT_EQ(joint_command_values_.at(idx), initial_pos_.at(idx));
  }
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
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * 2);
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  for(size_t idx = 0; idx < joints_.size(); ++idx)
  {
    EXPECT_NE(joint_command_values_.at(idx), initial_pos_.at(idx));
  }
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
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * 2);
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  for(size_t idx = 0; idx < joints_.size(); ++idx)
  {
    EXPECT_EQ(joint_command_values_.at(idx), initial_pos_.at(idx));
  }
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
  ASSERT_EQ(export_reference_interfaces().size(), joints_.size() * 2);
  ASSERT_EQ(controller_->set_chained_mode(false), true);
  ASSERT_EQ(activateController(), controller_interface::CallbackReturn::SUCCESS);

  auto dummy_node = std::make_shared<rclcpp::Node>("dummy_node");
  auto sub_friction_in_world = dummy_node->create_subscription<geometry_msgs::msg::WrenchStamped>("/test_elastoplastic_controller/friction_in_world", 1, [](const geometry_msgs::msg::WrenchStamped::SharedPtr msg){RCLCPP_INFO(rclcpp::get_logger("test"), "Got message!");});

  auto result = controller_->update(rclcpp::Clock().now(), std::chrono::milliseconds(10));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  for(size_t idx = 0; idx < joints_.size(); ++idx)
  {
    EXPECT_NE(joint_command_values_.at(idx), initial_pos_.at(idx));
  }

  geometry_msgs::msg::WrenchStamped msg_out; rclcpp::MessageInfo info;
  bool message_received {false};
  auto start = std::chrono::steady_clock::now();
  do
  {
    message_received = sub_friction_in_world->take(msg_out, info);
    rclcpp::spin_some(dummy_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while(!message_received && std::chrono::steady_clock::now() - start < std::chrono::seconds(10));
  ASSERT_TRUE(message_received);
  EXPECT_NE(msg_out.wrench.force.x, 0.0);
  EXPECT_NE(msg_out.wrench.force.y, 0.0);
  EXPECT_NE(msg_out.wrench.force.z, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
