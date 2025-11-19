#ifndef ADAPTIVE_HQP_HPP
#define ADAPTIVE_HQP_HPP

// local libs
#include "adaptive_hqp_parameters.hpp"
#include "elastoplastic_lugre_controller/interpolation/interpolator.hpp"
#include "elastoplastic_lugre_controller/notch_filter.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"
// #include "elastoplastic_variable_model.hpp"
#include <Eigen/src/Core/util/Constants.h>
#include <elastoplastic_msgs/msg/detail/adaptive_hqp_controller_state__struct.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <state_space_filters/filtered_values.h>

// fundamental libs
#include "Eigen/Dense"

// other libs
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "rdyn_core/primitives.h" // IWYU pragma: export
#include "state_observers/kalman_filter.hpp"

// ros lib
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp" // IWYU pragma: export
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "semantic_components/force_torque_sensor.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// ros msgs
// IWYU pragma: begin_keep
#include "elastoplastic_msgs/msg/adaptive_hqp_controller_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "moveit_msgs/msg/cartesian_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
// IWYU pragma: end_keep

namespace Eigen {
using Matrix6Xd = Matrix<double, 6, Dynamic>;
}

namespace elastoplastic {

namespace pin = pinocchio;

inline Eigen::VectorXd to_pinocchio_config(const Eigen::VectorXd &q) {
  Eigen::VectorXd out(q.size() + 1);
  out.head<2>() = q.head<2>();
  out(2) = std::cos(q(2));
  out(3) = std::sin(q(2));
  out.tail(out.size() - 4) = q.tail(q.size() - 3);
  return out;
}

class AdaptiveHQP : public controller_interface::ChainableControllerInterface {
private:
  std::shared_ptr<adaptive_hqp::ParamListener> m_param_listener;
  adaptive_hqp::Params m_parameters;

  template <typename T>
  using InterfaceReference = std::vector<std::reference_wrapper<T>>;

  std::vector<InterfaceReference<hardware_interface::LoanedStateInterface>>
      m_joint_state_interfaces;
  std::vector<InterfaceReference<hardware_interface::LoanedCommandInterface>>
      m_joint_command_interfaces;
  // InterfaceReference<hardware_interface::LoanedStateInterface>
  // m_mobile_base_state_interfaces;
  InterfaceReference<hardware_interface::LoanedCommandInterface>
      m_mobile_base_command_interfaces;

  std::unique_ptr<semantic_components::ForceTorqueSensor> m_ft_sensor;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      m_sub_mobile_base_target;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      m_sub_mobile_base_odometry;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      m_sub_mobile_base_pose;

  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> m_rt_buffer_base_odom;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseWithCovarianceStamped>
      m_rt_buffer_base_local;
  std::atomic<bool> m_got_new_base_pose;

  rclcpp::Time m_last_odom_msg_time;

  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<std::thread> m_tf_base_pose_recovery_thread;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_support_node_exec;
  rclcpp::Node::SharedPtr m_node_support; // tf and log
  void update_base_pose_from_tf();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cmd_vel;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>
      m_rt_pub_cmd_vel;
  rclcpp::Publisher<elastoplastic_msgs::msg::AdaptiveHQPControllerState>::
      SharedPtr m_pub_full_state;
  std::unique_ptr<realtime_tools::RealtimePublisher<
      elastoplastic_msgs::msg::AdaptiveHQPControllerState>>
      m_rt_pub_full_state;

  constexpr static double M_MINIMUM_SAMPLING_TIME{1e-4};
  constexpr static double M_INITIAL_INTERPOLATOR_DELTA{5e-3};
  constexpr static unsigned int M_SE3{6};
  constexpr static unsigned int M_SE2{3};

  std::string m_planar_joint_name;

  enum class RDStatus {
    OK,
    ERROR,
    EMPTY
  } m_robot_description_configuration{AdaptiveHQP::RDStatus::EMPTY};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      m_sub_robot_description;

  rdyn::ChainPtr m_chain_base_tool;
  rdyn::ChainPtr m_chain_base_sensor;
  rdyn::ChainPtr m_chain_world_tool;
  rdyn::ChainPtr m_chain_world_base;

  pin::Model m_chain_base_tool_model;
  pin::Data m_chain_base_tool_data;
  pin::Model m_chain_world_base_model;
  pin::Data m_chain_world_base_data;
  pin::Model m_chain_world_tool_model;
  pin::Data m_chain_world_tool_data;

  std::vector<pin::JointIndex> m_jnt_id;
  pin::FrameIndex m_tool_id;
  pin::FrameIndex m_sensor_id;
  pin::FrameIndex m_base_id;

  std::vector<std::string> m_joint_names;

  size_t m_nax;
  size_t m_full_nax;

  Eigen::VectorXd m_q;
  Eigen::VectorXd m_qp;
  Eigen::VectorXd m_qpp;

  double m_dt;

  double m_kp_joint_task, m_kv_joint_task;

  Eigen::MatrixXd m_W; // Weight matrix for CLIK

  Eigen::VectorXd m_initial_q;
  Eigen::Affine3d m_initial_T_world_tool;
  Eigen::Affine3d m_initial_T_world_base;
  Eigen::VectorXd m_q_prec;
  Eigen::VectorXd m_qp_prec;
  Eigen::VectorXd m_qpp_prec;
  Eigen::Vector6d m_wrench_in_sensor_prec;
  Eigen::Affine3d m_T_tool_sensor;

  Eigen::Vector6d m_admittance_value;

  Eigen::Affine3d m_T_world_base;

  Eigen::Vector6d m_offset_wrench_tool_in_world;
  std::future<bool> m_offset_future;

  state_observer::KalmanFilter m_base_position_filter;
  state_observer::KalmanFilter m_joint_filter;

  // Required both for states and at least one for command
  const std::vector<std::string> m_required_interface_types{
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT};
  const std::vector<std::string> m_allowed_interface_types{
      hardware_interface::HW_IF_TORQUE};
  std::array<bool, 3> m_used_command_interfaces;

  struct FloatBaseData {
    FloatBaseData() = delete;
    FloatBaseData(const bool en)
        : enabled(en), base_joint_names(en ? std::vector<std::string>(
                                                 {"move_x", "move_y", "rot_z"})
                                           : std::vector<std::string>()) {}

    const bool enabled;
    const std::vector<std::string> base_joint_names;

    size_t nax() const { return enabled ? nax_ : 0; }
    // std::vector<std::string> base_joint_names() { return enabled ?
    // base_joint_names_ : std::vector<std::string>{}; }
    Eigen::Vector3d vel_limits;
    Eigen::Vector3d acc_limits;

  private:
    constexpr static size_t nax_{3};
  };
  std::unique_ptr<FloatBaseData> m_mobile_base;

  Eigen::Vector3d m_velocity_base_in_base;
  bool m_mobile_base_pose_updated;

  std::vector<std::string> m_state_interfaces_names;
  std::vector<std::string> m_command_interfaces_names;

  struct Limits {
    Eigen::VectorXd pos_upper;
    Eigen::VectorXd pos_lower;
    Eigen::VectorXd vel;
    Eigen::VectorXd acc;
  } m_limits;

  std::unique_ptr<ElastoplasticModel> m_elastoplastic_model;

  struct ClikData {
    const Eigen::VectorXd &position_references, velocity_references;
    const Eigen::Vector6d &twist_tool_world_in_world;
    //, next_twist_tool_world_in_world;
    const Eigen::Affine3d &T_world_tool;
    //, next_T_world_tool;
    const Eigen::Vector6d &target_acc_tool_target_in_world;
    const Eigen::Matrix6Xd &J_world_tool_in_world;
    const Eigen::Affine3d &target_T_world_tool;
    const Eigen::Vector6d &target_twist_tool_world_in_world;
    const Eigen::Vector6d &wrench_tool_in_world;
    const bool got_new_odom;
  };

  std::vector<NotchFilter> m_wrench_filters;
  std::vector<eigen_control_toolbox::FilteredScalar> m_low_pass_filters;

  Eigen::Vector6d m_zp;

  utils::Logistic m_logistic;
  double m_logis_prec;

  utils::interpolation::Interpolator m_interpolator;
  rclcpp::Subscription<moveit_msgs::msg::CartesianTrajectory>::SharedPtr
      m_carteisan_trj_sub;

  utils::Slider m_pos_task_slider;

  Eigen::Vector6d m_computed_target_acc_tool_world_in_world;
  Eigen::Vector6d m_computed_target_twist_tool_world_in_world;
  Eigen::Affine3d m_computed_target_T_world_tool;

  enum class FTSource { FT_SENSOR, TOPIC, TORQUE } m_ft_source;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_wrench_sub;
  constexpr static char FT_TOPIC[] = "~/wrench_input";
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::WrenchStamped>
      m_wrench_topic_buffer;

  Eigen::Vector6d get_wrench_from_topic() {
    auto w = m_wrench_topic_buffer.readFromRT()->wrench;
    return Eigen::Vector6d(
        {w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z});
  }

  Eigen::Vector6d get_wrench_from_sensor() {
    geometry_msgs::msg::Wrench w;
    m_ft_sensor->get_values_as_message(w);
    double s = m_parameters.ft_invert_sign ? -1.0 : 1.0;
    return Eigen::Vector6d({s * w.force.x, s * w.force.y, s * w.force.z,
                            s * w.torque.x, s * w.torque.y, s * w.torque.z});
  }

  double m_invert_torque;
  Eigen::Vector6d
  get_wrench_from_torque(const Eigen::JacobiSVD<Eigen::Matrix6Xd> &svd,
                         const Eigen::VectorXd &tau) {
    return m_invert_torque * svd.solve(tau);
  }

  bool m_base_use_cmd_ifaces;

public:
  AdaptiveHQP() {}

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time,
                            const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  // controller_interface::CallbackReturn on_shutdown(
  //     const rclcpp_lifecycle::State & previous_state) override;

  bool ready_for_activation() {
    return m_robot_description_configuration == RDStatus::OK;
  }

protected:
  std::vector<hardware_interface::CommandInterface>
  on_export_reference_interfaces() override;

  controller_interface::return_type
  update_reference_from_subscribers(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) override;

  void configure_after_robot_description_callback(
      const std_msgs::msg::String::SharedPtr msg);

  std::optional<Eigen::VectorXd> clik(const ClikData &data);
  Eigen::VectorXd compute_clik_as_qp(const ClikData &data,
                                     const Eigen::Vector6d &a_position_error,
                                     const Eigen::Vector6d &a_twist_error,
                                     const Eigen::Vector6d &a_acc_non_linear);
  Eigen::VectorXd compute_clik_as_inv(const ClikData &data,
                                      const Eigen::Vector6d &a_position_error,
                                      const Eigen::Vector6d &a_twist_error,
                                      const Eigen::Vector6d &a_acc_non_linear);

  bool write_cmd_vel_zero();
  bool write_cmd_vel(const Eigen::Ref<Eigen::Vector3d> &v);
  void get_target_callback(const geometry_msgs::msg::Twist &msg);
  void get_mobile_base_target_callback(const geometry_msgs::msg::Twist &msg);
  void get_odometry_callback(const nav_msgs::msg::Odometry &msg);
  void get_localization_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
};
} // namespace elastoplastic

#endif // ADAPTIVE_HQP_HPP
