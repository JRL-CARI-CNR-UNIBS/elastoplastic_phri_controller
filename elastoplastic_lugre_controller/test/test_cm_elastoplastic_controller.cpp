#include <gmock/gmock.h>

#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestCMElastoplasticController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    executor, ros2_control_test_assets::minimal_robot_urdf, true, "test_controller_manager");

  const std::string test_file_path =
    std::string(TEST_FILES_DIRECTORY) + "/test_params.yaml";

  cm.set_parameter({"test_elastoplastic_controller.params_file", test_file_path});
  cm.set_parameter(
    {"test_elastoplastic_controller.type",
     "elastoplastic/ElastoplasticController"});

  ASSERT_NE(
    cm.load_controller(
      "test_elastoplastic_controller", "elastoplastic/ElastoplasticController"),
    nullptr);

  rclcpp::shutdown();
}

TEST(TestCMElastoplasticController, configure_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    executor, ros2_control_test_assets::minimal_robot_urdf, true, "test_controller_manager");

  const std::string controller_name = "test_elastoplastic_controller";

  const std::string test_file_path =
    std::string(TEST_FILES_DIRECTORY) + "/test_params.yaml";

  cm.set_parameter({controller_name + ".params_file", test_file_path});
  cm.set_parameter(
    {controller_name + ".type",
     "elastoplastic/ElastoplasticController"});

  ASSERT_NE(
    cm.load_controller(
      controller_name, "elastoplastic/ElastoplasticController"),
    nullptr);

  EXPECT_EQ(cm.configure_controller(controller_name), controller_interface::return_type::OK);

  rclcpp::shutdown();
}