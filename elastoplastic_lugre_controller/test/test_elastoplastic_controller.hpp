#ifndef TEST_ELASTOPLASTIC_CONTROLLER_HPP
#define TEST_ELASTOPLASTIC_CONTROLLER_HPP

#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"
#include "controller_interface/chainable_controller_interface.hpp"

// #include "gmock/gmock.h"
#include <gtest/gtest.h>

#include <memory>
#include <rclcpp/logger.hpp>

class ElastoplasticControllerTest : public ::testing::Test
{
protected:
  ElastoplasticControllerTest()
  {
    controller_ = std::make_unique<elastoplastic::ElastoplasticController>();
  }

  std::vector<rclcpp::Parameter> getOverrideParameters(){
      return {
        {"target_joint_trajectory_topic", "/joints_target"},
        {"ft_sensor_name", ft_sensor_name_},
        {"frames.base", "az_base_footprint"},
        {"frames.tool", "az_robotiq_ft_frame_id"},
        {"frames.sensor", "az_ft300s_sensor"},
        {"joints", joints_},
        {"command_interfaces", command_interfaces_names_},
        {"state_interfaces", state_interfaces_names_},
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

  controller_interface::return_type SetUpController()
  {
    return SetUpController("test_elastoplastic_controller", getOverrideParameters());
  }

  controller_interface::return_type SetUpController(
      const std::string & name, const std::vector<rclcpp::Parameter>& overrides)
  {
    auto options = rclcpp::NodeOptions()
                       .parameter_overrides(overrides)
                       .allow_undeclared_parameters(false)
                       .automatically_declare_parameters_from_overrides(true);

    auto res = controller_->init(name, "", options);

    // EXPECT_NE(controller_->export_reference_interfaces().size(), 0);
    assign_interfaces();

    controller_->get_node()->get_logger().set_level(rclcpp::Logger::Level::Warn);

    return res;
  }

  controller_interface::CallbackReturn configureController()
  {
    return controller_->on_configure(rclcpp_lifecycle::State());
  }

  controller_interface::CallbackReturn activateController()
  {
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
    return controller_->on_activate(rclcpp_lifecycle::State());
  }

  void assign_interfaces()
  {
    joint_command_values_.resize(command_interfaces_names_.size() * joints_.size());
    joint_state_values_.resize(command_interfaces_names_.size() * joints_.size());
    fts_state_values_.resize(ft_sensor_interfaces_.size());

    std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
    loaned_command_interfaces.reserve(command_interfaces_names_.size() * joints_.size());
    command_interfaces_.reserve(command_interfaces_names_.size() * joints_.size());

    for(size_t idx = 0; idx < joints_.size(); ++idx)
    {
      command_interfaces_.emplace_back(
          hardware_interface::CommandInterface(
              joints_[idx], command_interfaces_names_[0], &joint_command_values_[idx]
          )
      );
      loaned_command_interfaces.emplace_back(command_interfaces_.back());
    }
    for(size_t idx = 0; idx < joints_.size(); ++idx)
    {
      command_interfaces_.emplace_back(
          hardware_interface::CommandInterface(
              joints_[idx], command_interfaces_names_[1], &joint_command_values_[joints_.size() + idx]
              )
          );
      loaned_command_interfaces.emplace_back(command_interfaces_.back());
    }


    std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;
    loaned_state_interfaces.reserve(state_interfaces_names_.size() * joints_.size() + ft_sensor_interfaces_.size());
    state_interfaces_.reserve(state_interfaces_names_.size() * joints_.size() + ft_sensor_interfaces_.size());

    for(const auto& interface : state_interfaces_names_)
    {
      auto it = std::ranges::find(state_interfaces_names_, interface);
      auto index_interface = std::distance(state_interfaces_names_.begin(), it);
      for (size_t idx = 0; idx < joints_.size(); ++idx)
      {
        state_interfaces_.emplace_back(
            hardware_interface::StateInterface(
                joints_[idx], interface, &joint_state_values_[index_interface * joints_.size() + idx]
            )
        );
        loaned_state_interfaces.emplace_back(state_interfaces_.back());
      }
    }

    for (size_t idx = 0; idx < ft_sensor_interfaces_.size(); ++idx)
    {
      state_interfaces_.emplace_back(
          hardware_interface::StateInterface(
              ft_sensor_name_, ft_sensor_interfaces_[idx], &fts_state_values_[idx]
          )
      );
      loaned_state_interfaces.emplace_back(state_interfaces_.back());
    }

    controller_->assign_interfaces(std::move(loaned_command_interfaces),
                                   std::move(loaned_state_interfaces));

  }

  std::vector<hardware_interface::CommandInterface> export_reference_interfaces()
  {
    return controller_->export_reference_interfaces();
  }

  void set_initial_position()
  {
    std::fill(joint_state_values_.begin(),
        std::next(joint_state_values_.begin(), joints_.size()),
        INITIAL_POS);
  }



protected:
  std::vector<std::string> joints_ = {"az_shoulder_pan_joint",
                                      "az_shoulder_lift_joint",
                                      "az_elbow_joint",
                                      "az_wrist_1_joint",
                                      "az_wrist_2_joint",
                                      "az_wrist_3_joint"};
  std::vector<std::string> command_interfaces_names_ {hardware_interface::HW_IF_POSITION,
                                                        hardware_interface::HW_IF_VELOCITY};
  std::vector<std::string> state_interfaces_names_ {hardware_interface::HW_IF_POSITION,
                                                      hardware_interface::HW_IF_VELOCITY};
  std::vector<std::string> ft_sensor_interfaces_ {"force.x",  "force.y",  "force.z",
                                                    "torque.x", "torque.y", "torque.z"};
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  std::vector<double> joint_command_values_;
  std::vector<double> joint_state_values_;
  std::vector<double> fts_state_values_;
  std::unique_ptr<elastoplastic::ElastoplasticController> controller_;

  const std::string ft_sensor_name_ {"tcp_fts_sensor"};

  constexpr static double INITIAL_POS = M_PI_2;
};

#endif // TEST_ELASTOPLASTIC_CONTROLLER_HPP
