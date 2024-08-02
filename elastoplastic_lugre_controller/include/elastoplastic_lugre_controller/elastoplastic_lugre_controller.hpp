#ifndef ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
#define ELASTOPLASTIC_LUGRE_CONTROLLER_HPP

#include "rdyn_core/primitives.h"

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include <semantic_components/force_torque_sensor.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "elastoplastic_parameters.hpp"

#include <deque>
#include <stdexcept>
#include <ranges>

namespace elastoplastic {

class ElastoplasticController : public controller_interface::ChainableControllerInterface
{
public:
    ElastoplasticController() {};


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


    // controller_interface::CallbackReturn on_cleanup(
    //     const rclcpp_lifecycle::State & previous_state) override;


    // controller_interface::CallbackReturn on_error(
    //     const rclcpp_lifecycle::State & previous_state) override;


    // controller_interface::CallbackReturn on_shutdown(
    //     const rclcpp_lifecycle::State & previous_state) override;
protected:
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    controller_interface::return_type update_reference_from_subscribers() override;

    void get_target_callback(const geometry_msgs::msg::Twist& msg);
    void get_fb_target_callback(const geometry_msgs::msg::Twist& msg);
    void get_odometry_callback(const nav_msgs::msg::Odometry& msg);

protected:
    std::shared_ptr<elastoplastic_controller::ParamListener> m_param_listener;
    elastoplastic_controller::Params m_parameters;

    template <typename T>
    using InterfaceReference = std::vector<std::vector<std::reference_wrapper<T>>>;

    InterfaceReference<hardware_interface::LoanedStateInterface  > m_joint_state_interfaces;
    InterfaceReference<hardware_interface::LoanedCommandInterface> m_joint_command_interfaces;
    InterfaceReference<hardware_interface::LoanedStateInterface>   m_fb_state_interfaces;
    InterfaceReference<hardware_interface::LoanedCommandInterface> m_fb_command_interfaces;

    size_t m_joint_reference_interfaces;

    std::unique_ptr<semantic_components::ForceTorqueSensor> m_ft_sensor;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub_fb_target;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_base_odometry;

    realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> m_rt_buffer_fb_target;
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseWithCovariance>   m_rt_buffer_base_pose_in_world;
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistWithCovariance>   m_rt_buffer_base_twist_in_base;

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cmd_vel;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray> ::SharedPtr m_pub_z;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray> ::SharedPtr m_pub_w;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_pub_friction_in_base;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_pub_wrench_in_base;

    rdyn::ChainPtr m_chain_base_tool;
    rdyn::ChainPtr m_chain_base_sensor;

    size_t m_nax;

    Eigen::VectorXd m_q;
    Eigen::VectorXd m_qp;
    Eigen::VectorXd m_qpp;

    const std::vector<std::string> m_allowed_interface_types {
          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY};

    struct FloatBaseData
    {
      Eigen::Vector6d twist;
      bool enabled {true};

      std::string ns;
      const size_t& nax() const {return nax_;}
      const std::array<size_t, 3>& idxs() const {return idxs_;}

      Eigen::MatrixXd jacobian()
      {
        Eigen::Matrix<double,6,6> id = Eigen::MatrixXd::Identity(6,6);
        return id(idxs(), Eigen::all);
      };

    private:
      constexpr static size_t nax_ {3};
      constexpr static std::array<size_t, 3> idxs_ {1,2,6};

    } m_float_base;

    struct Interfaces{
      struct InterfaceType : std::array<bool, 2> {
        bool& position() {return (*this).at(0);}
        bool& velocity() {return (*this).at(1);}
        const bool& position() const {return (*this).at(0);}
        const bool& velocity() const {return (*this).at(1);}
      } state, command;
      std::array<std::string, 2> hwi = {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
      bool check() {return state.position() && state.velocity() && (command.position() || command.velocity());}
      const std::array<std::string, 2>& names(){return hwi;}
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
    } m_elastoplastic_target_tool_in_world;

    std::deque<double> m_reset_window;
};
}

#endif // ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
