#ifndef ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
#define ELASTOPLASTIC_LUGRE_CONTROLLER_HPP

#include "elastoplastic_model.hpp"

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

    bool ready_for_activation()
    {
      return m_robot_description_configuration == RDStatus::OK;
    }

protected:
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    controller_interface::return_type update_reference_from_subscribers() override;

    void configure_after_robot_description_callback(const std_msgs::msg::String::SharedPtr msg);

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

    size_t m_joint_reference_interfaces_size;

    std::unique_ptr<semantic_components::ForceTorqueSensor> m_ft_sensor;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub_fb_target;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_base_odometry;

    realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> m_rt_buffer_fb_target;
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseWithCovariance>   m_rt_buffer_base_pose_in_world;
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistWithCovariance>   m_rt_buffer_base_twist_in_base;

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cmd_vel;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray> ::SharedPtr m_pub_z;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray> ::SharedPtr m_pub_w;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_pub_friction_in_world;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_pub_wrench_in_world;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cart_vel_error;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pub_pos_correction;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pub_vel_correction;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_clik_pub;

    enum class RDStatus {
      OK,
      ERROR,
      EMPTY
    } m_robot_description_configuration {ElastoplasticController::RDStatus::EMPTY};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub_robot_description;

    rdyn::ChainPtr m_chain_base_tool;
    rdyn::ChainPtr m_chain_base_sensor;

    size_t m_nax;

    Eigen::VectorXd m_q;
    Eigen::VectorXd m_qp;
    Eigen::VectorXd m_qpp;

    // Required both for states and at least one for command
    const std::vector<std::string> m_allowed_interface_types {
          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY};
    std::array<bool, 2> m_used_command_interfaces;


    struct FloatBaseData
    {
      Eigen::Vector6d twist_in_base;
      bool enabled {true};

      std::string ns;
      const size_t nax() const {return enabled? nax_: 0;}
      const std::array<size_t, 3>& idxs() const {return idxs_;}

      Eigen::Matrix6Xd jacobian()
      {
        if(!enabled)
        {
          return Eigen::Matrix<double, 6, -1>(6, 0);
        }
        Eigen::Matrix<double,6,-1> id = Eigen::MatrixXd::Identity(6,6);
        return id(Eigen::all, idxs());
      };

    private:
      constexpr static size_t nax_ {3};
      constexpr static std::array<size_t, 3> idxs_ {0,1,5};

    } m_float_base;

    std::vector<std::string> m_state_interfaces_names;
    std::vector<std::string> m_command_interfaces_names;


    struct Limits {
      Eigen::VectorXd pos_upper;
      Eigen::VectorXd pos_lower;
      Eigen::VectorXd vel;
      Eigen::VectorXd acc;
    } m_limits;

    std::unique_ptr<ElastoplasticModel> m_elastoplastic_model;

    struct IntegralState{
      Eigen::Vector6d position;
      Eigen::Vector6d velocity;
      void clear(){position.setZero(); velocity.setZero();}
    } m_delta_elastoplastic_in_world;


    ElastoplasticModelData get_model_data()
    {
      ElastoplasticModelData data;
      data.inertia_inv = Eigen::Map<Eigen::Vector3d>(m_parameters.impedance.inertia.data(), 3).cwiseInverse();
      data.lugre.sigma_0 = m_parameters.impedance.lugre.sigma_0;
      data.lugre.sigma_1 = m_parameters.impedance.lugre.sigma_1;
      data.lugre.sigma_2 = m_parameters.impedance.lugre.sigma_2;
      data.lugre.z_ss = m_parameters.impedance.lugre.z_ss;
      data.lugre.z_ba = m_parameters.impedance.lugre.z_ba;
      data.lugre.tau_w = m_parameters.impedance.tau_w;
      data.reset_condition.reset_window_size = m_parameters.impedance.reset_condition.reset_window_size;
      data.reset_condition.reset_threshold = m_parameters.impedance.reset_condition.reset_threshold;
      return data;
    }
};
}

#endif // ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
