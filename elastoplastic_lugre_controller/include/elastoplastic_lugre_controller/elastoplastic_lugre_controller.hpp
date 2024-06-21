#ifndef ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
#define ELASTOPLASTIC_LUGRE_CONTROLLER_HPP

#include "rdyn_core/primitives.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <semantic_components/force_torque_sensor.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "elastoplastic_parameters.hpp"

#include <deque>
#include <stdexcept>
#include <ranges>

namespace elastoplastic {

class ElastoplasticController : public controller_interface::ChainableControllerInterface
{
public:
    ElastoplasticController();


    controller_interface::InterfaceConfiguration command_interface_configuration() const override;


    controller_interface::InterfaceConfiguration state_interface_configuration() const override;


    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;


    controller_interface::CallbackReturn on_init() override;


    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;


    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;


    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;


    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;


    controller_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state) override;


    controller_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & previous_state) override;
protected:
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    controller_interface::return_type update_reference_from_subscribers() override;

    void get_target_callback(const sensor_msgs::msg::JointState& msg);
    void get_aux_target_callback(const geometry_msgs::msg::Twist& msg);

protected:
    std::shared_ptr<elastoplastic_controller::ParamListener> m_param_listener;
    elastoplastic_controller::Params m_parameters;

    template <typename T>
    using InterfaceReference = std::vector<std::vector<std::reference_wrapper<T>>>;

    InterfaceReference<hardware_interface::LoanedStateInterface  > m_joint_state_interfaces;
    InterfaceReference<hardware_interface::LoanedCommandInterface> m_joint_command_interfaces;
    InterfaceReference<hardware_interface::LoanedStateInterface>   m_aux_state_interfaces;
    InterfaceReference<hardware_interface::LoanedCommandInterface> m_aux_command_interfaces;

    std::unique_ptr<semantic_components::ForceTorqueSensor> m_ft_sensor;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub_robot_description;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_sub_target_joint_trajectory;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub_aux_target;

    realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState> m_rt_buffer_joint_trajectory;
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist>    m_rt_buffer_aux_target;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray> ::SharedPtr m_pub_z;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray> ::SharedPtr m_pub_w;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_pub_friction_in_base;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_pub_wrench_in_base;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>  ::SharedPtr m_pub_pose_in_base;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>  ::SharedPtr m_pub_target_in_base;

    rdyn::ChainPtr m_chain_base_tool;
    rdyn::ChainPtr m_chain_base_sensor;

    double m_nax;

    Eigen::VectorXd m_q;
    Eigen::VectorXd m_qp;
    Eigen::VectorXd m_qpp;

    const std::vector<std::string> m_allowed_interface_types {
          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY,
          hardware_interface::HW_IF_ACCELERATION};

    struct AuxAxis : std::tuple<std::array<std::string,6>, std::vector<int>, Eigen::Vector6d>
    {
      std::string& name(size_t idx) {return std::get<0>(*this).at(idx);}
      const std::string& name(size_t idx) const {return std::get<0>(*this).at(idx);}
      const std::array<std::string, 6>& names() const {return std::get<0>(*this);}

      int& enabled(size_t idx) {return std::get<1>(*this).at(idx);}
      const int& enabled(size_t idx) const {return std::get<1>(*this).at(idx);}
      std::vector<int>& enabled() {return std::get<1>(*this);}
      const std::vector<int>& enabled() const {return std::get<1>(*this);}

      Eigen::Vector6d& values() {return std::get<2>(*this);}
      const Eigen::Vector6d& values() const {return std::get<2>(*this);}

      bool init(std::vector<std::string> v, std::vector<bool> b){
        enabled().clear();
        if (v.size() != 6 || b.size() != 6)
        {
          return false;
        }
        for(size_t idx = 0; idx < 6; idx++)
        {
          name(idx) = v.at(idx);
          // enabled(idx) = b.at(idx)? 1:0;
          if(b.at(idx)) enabled().push_back(idx);
        }
        values().setZero();
        return true;
      }

      Eigen::MatrixXd jacobian() {
        auto id = Eigen::MatrixXd::Identity(6,6)(enabled(), Eigen::all);
      }
    } m_aux_axis;

    struct Interfaces{
      struct InterfaceType : std::array<bool, 3> {
        bool& position() {return (*this).at(0);}
        bool& velocity() {return (*this).at(1);}
        bool& effort  () {return (*this).at(2);}
        const bool& position() const {return (*this).at(0);}
        const bool& velocity() const {return (*this).at(1);}
        const bool& effort  () const {return (*this).at(2);}
        bool check() {return position() || velocity() || effort();}
      } state, command;
      bool check() {return state.check() && command.check();}
    } m_has_interfaces;

    struct Limits {
      Eigen::VectorXd pos_upper;
      Eigen::VectorXd pos_lower;
      Eigen::VectorXd vel;
      Eigen::VectorXd acc;
    } m_limits;

    Eigen::Vector3d m_inertia_inv;

    struct ModelState{
      Eigen::Vector3d z;
      Eigen::Vector3d w;
      double r;

      void clear() {
        z.setZero();
        w.setZero();
        r = 0;
      }
    } m_state;

    struct IntegralState{
      Eigen::Vector6d position;
      Eigen::Vector6d velocity;
      void clear(){position.setZero(); velocity.setZero();}
    } m_cart_tool_in_base;

    std::deque<double> m_reset_window;
};
}

#endif // ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
