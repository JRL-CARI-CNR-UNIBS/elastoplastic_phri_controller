#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include "test_urdf.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <future>
#include <fmt/printf.h>

using namespace std::chrono_literals;

struct ParamData {
  std::vector<std::string> joints_ {"az_shoulder_pan_joint",
                                     "az_shoulder_lift_joint",
                                     "az_elbow_joint",
                                     "az_wrist_1_joint",
                                     "az_wrist_2_joint",
      "az_wrist_3_joint"};
  std::vector<std::string> command_interface_names_ {hardware_interface::HW_IF_POSITION};
} param_data;

std::vector<rclcpp::Parameter> getOverrideParameters(){
  return {
      {"target_joint_trajectory_topic", "/joints_target"},
      {"ft_sensor_name", "tcp_fts_sensor"},
      {"frames.base", "az_base_footprint"},
      {"frames.tool", "az_robotiq_ft_frame_id"},
      {"frames.sensor", "az_ft300s_sensor"},
      {"joints", param_data.joints_},
      {"command_interfaces",param_data.command_interface_names_},
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

  std::shared_ptr<controller_manager::ControllerManager> cm = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(
          test_urdf),
      exec,
      "test_controller_manager");

  cm->declare_parameter("elastoplastic_controller.type", "elastoplastic_controller/ElastoplasticController");

  std::string install_test_path = ament_index_cpp::get_package_share_directory("elastoplastic_lugre_controller");
  cm->declare_parameter("elastoplastic_controller.params_file", install_test_path + "/test/test_params.yaml");

  std::vector<std::string> nodes = dummy.get_node_names();
  ASSERT_TRUE(std::ranges::find(nodes, "/test_controller_manager") != nodes.end()); // Just to be sure

  EXPECT_NE(cm->load_controller("elastoplastic_controller"), nullptr);
}

TEST(ElastoplasticLoadOnCM, configure_without_rd_check)
{
  rclcpp::Node dummy = rclcpp::Node("dummy");

  auto rb_pub = dummy.create_publisher<std_msgs::msg::String>("/test_controller_manager/robot_description", rclcpp::QoS(2).transient_local());
  auto rb_pub2 = dummy.create_publisher<std_msgs::msg::String>("/robot_description", rclcpp::QoS(2).transient_local());
  std_msgs::msg::String rd;
  rd.data = test_urdf;
  rb_pub->publish(rd);
  rb_pub2->publish(rd);


  auto test_client = dummy.create_client<controller_manager_msgs::srv::SetHardwareComponentState>("/test_controller_manager/set_hardware_component_state");

  std::shared_ptr<rclcpp::Executor> exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  std::shared_ptr<controller_manager::ControllerManager> cm = std::make_shared<controller_manager::ControllerManager>(
      exec,
      "test_controller_manager");

  // Wait until controller_manager is ready
  rclcpp::spin_some(cm);
  auto tin = std::chrono::system_clock::now();
  while(not test_client->service_is_ready() && std::chrono::system_clock::now() - tin < 10s)
  {
    std::this_thread::sleep_for(500ms);
  }
  ASSERT_TRUE(test_client->service_is_ready());

  cm->declare_parameter("elastoplastic_controller.type", "elastoplastic_controller/ElastoplasticController");
  std::string install_test_path = ament_index_cpp::get_package_share_directory("elastoplastic_lugre_controller");
  cm->declare_parameter("elastoplastic_controller.params_file", install_test_path + "/test/test_params.yaml");
  controller_interface::ControllerInterfaceBaseSharedPtr cib = cm->load_controller("elastoplastic_controller");
  EXPECT_NE(cib, nullptr);

  EXPECT_EQ(cm->configure_controller("elastoplastic_controller"), controller_interface::return_type::OK);

}

TEST(ElastoplasticLoadOnCM, get_interfaces)
{
  rclcpp::Node dummy = rclcpp::Node("dummy");

  auto rb_pub = dummy.create_publisher<std_msgs::msg::String>("/test_controller_manager/robot_description", rclcpp::QoS(2).transient_local());
  std_msgs::msg::String rd;
  rd.data = test_urdf;
  rb_pub->publish(rd);

  std::shared_ptr<rclcpp::Executor> exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  std::shared_ptr<controller_manager::ControllerManager> cm = std::make_shared<controller_manager::ControllerManager>(
      exec,
      "test_controller_manager");

  rclcpp::spin_some(cm);

  auto li_client = dummy.create_client<controller_manager_msgs::srv::ListHardwareInterfaces>("/test_controller_manager/list_hardware_interfaces");

  RCLCPP_INFO(rclcpp::get_logger("get_interfaces_TEST_logger"), "Call service");
  auto ros_future = li_client->async_send_request(std::make_shared<controller_manager_msgs::srv::ListHardwareInterfaces::Request>());
  rclcpp::spin_some(cm);
  ASSERT_EQ(rclcpp::spin_until_future_complete(dummy.get_node_base_interface(), ros_future), rclcpp::FutureReturnCode::SUCCESS);

  controller_manager_msgs::srv::ListHardwareInterfaces::Response::SharedPtr res = ros_future.get();

  for (std::string iface : param_data.joints_)
  {
    EXPECT_NE(std::ranges::find(res->command_interfaces, iface + "/" + hardware_interface::HW_IF_POSITION, &controller_manager_msgs::msg::HardwareInterface::name), res->command_interfaces.end());
  }

  for (std::string iface : param_data.joints_)
  {
    EXPECT_NE(std::ranges::find(res->state_interfaces, iface + "/" + hardware_interface::HW_IF_POSITION, &controller_manager_msgs::msg::HardwareInterface::name), res->state_interfaces.end());
  }

}

TEST(ElastoplasticLoadOnCM, activate_with_rd_from_topic)
{
  rclcpp::Node dummy = rclcpp::Node("dummy");

  auto rb_pub = dummy.create_publisher<std_msgs::msg::String>("/test_controller_manager/robot_description", rclcpp::QoS(2).transient_local());
  auto rb_pub2 = dummy.create_publisher<std_msgs::msg::String>("/robot_description", rclcpp::QoS(2).transient_local());
  std_msgs::msg::String rd;
  rd.data = test_urdf;
  rb_pub->publish(rd);
  rb_pub2->publish(rd);

  auto test_client = dummy.create_client<controller_manager_msgs::srv::SetHardwareComponentState>("/test_controller_manager/set_hardware_component_state");

  std::shared_ptr<rclcpp::Executor> exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  std::shared_ptr<controller_manager::ControllerManager> cm = std::make_shared<controller_manager::ControllerManager>(
      exec,
      "test_controller_manager");

  rclcpp::executors::SingleThreadedExecutor ex_cm;

  std::unique_ptr<std::thread> cm_thread = std::make_unique<std::thread>([&cm, &ex_cm](){
    ex_cm.add_node(cm);
    ex_cm.spin();
    ex_cm.remove_node(cm);
  });

  // Wait until controller_manager is ready
  auto tin = std::chrono::system_clock::now();
  while(not test_client->service_is_ready() && std::chrono::system_clock::now() - tin < 10s)
  {
    std::this_thread::sleep_for(500ms);
  }
  ASSERT_TRUE(test_client->service_is_ready());

  cm->declare_parameter("elastoplastic_controller.type", "elastoplastic_controller/ElastoplasticController");
  std::string install_test_path = ament_index_cpp::get_package_share_directory("elastoplastic_lugre_controller");
  cm->declare_parameter("elastoplastic_controller.params_file", install_test_path + "/test/test_params.yaml");
  controller_interface::ControllerInterfaceBaseSharedPtr cib = cm->load_controller("elastoplastic_controller");
  EXPECT_NE(cib, nullptr);

  EXPECT_EQ(cm->configure_controller("elastoplastic_controller"), controller_interface::return_type::OK);

  exec->spin_some();

  std::vector<std::string> starting {"elastoplastic_controller"};
  std::vector<std::string> stopping {};
  auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm,
      starting, stopping, controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  bool c_update {true};
  std::unique_ptr<std::thread> update_cm_thread = std::make_unique<std::thread>([&cm, &c_update](){
    while(c_update)
      cm->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  });

  EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());

  ex_cm.cancel();
  cm_thread->join();
  c_update = false;
  update_cm_thread->join();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
