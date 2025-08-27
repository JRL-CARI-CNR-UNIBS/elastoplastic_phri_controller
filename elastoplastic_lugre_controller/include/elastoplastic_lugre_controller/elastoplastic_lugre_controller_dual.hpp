#ifndef ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
#define ELASTOPLASTIC_LUGRE_CONTROLLER_HPP

// local libs
#include "elastoplastic_dual_parameters.hpp"
#include "elastoplastic_lugre_controller/interpolation/interpolator.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"
#include "elastoplastic_variable_model.hpp"

// fundamental libs
#include "Eigen/Dense"

// other libs
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
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// ros msgs
// IWYU pragma: begin_keep
#include "elastoplastic_msgs/msg/elastoplastic_dual_controller_state.hpp"
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
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
// IWYU pragma: end_keep

namespace elastoplastic
{

template <typename T> using Couple = std::array<T, 2>;

class ElastoplasticControllerDual : public controller_interface::ChainableControllerInterface {
private:
  std::shared_ptr<elastoplastic_controller_dual::ParamListener> m_param_listener;
  elastoplastic_controller_dual::Params m_parameters;

  template <typename T> using InterfaceReference = std::vector<std::reference_wrapper<T>>;

  std::vector<InterfaceReference<hardware_interface::LoanedStateInterface>> m_joint_state_interfaces;
  std::vector<InterfaceReference<hardware_interface::LoanedCommandInterface>> m_joint_command_interfaces;
  // InterfaceReference<hardware_interface::LoanedStateInterface> m_mobile_base_state_interfaces;
  InterfaceReference<hardware_interface::LoanedCommandInterface> m_mobile_base_command_interfaces;

  size_t m_joint_reference_interfaces_size;

  Couple<std::unique_ptr<semantic_components::ForceTorqueSensor>> m_ft_sensors;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub_mobile_base_target;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_mobile_base_odometry;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_sub_mobile_base_pose;

  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> m_rt_buffer_base_odom;

  rclcpp::Time m_last_odom_msg_time;

  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_bcast;
  bool m_enable_shared_frame_bcast;
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<std::thread> m_tf_base_pose_recovery_thread;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_support_node_exec;
  rclcpp::Node::SharedPtr m_node_support; // tf and log
  void update_base_pose_from_tf();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cmd_vel;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> m_rt_pub_cmd_vel;
  rclcpp::Publisher<elastoplastic_msgs::msg::ElastoplasticDualControllerState>::SharedPtr m_pub_full_state;
  std::unique_ptr<realtime_tools::RealtimePublisher<elastoplastic_msgs::msg::ElastoplasticDualControllerState>>
    m_rt_pub_full_state;

  constexpr static double M_MINIMUM_SAMPLING_TIME{1e-4};
  constexpr static unsigned int M_SE3{6};
  constexpr static unsigned int M_SE2{3};
  constexpr static char SHARED_FRAME_NAME[]{"shared"};

  enum class RDStatus { OK, ERROR, EMPTY } m_robot_description_configuration{ElastoplasticControllerDual::RDStatus::EMPTY};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub_robot_description;

  Couple<rdyn::ChainPtr> m_chain_base_tools;
  Couple<rdyn::ChainPtr> m_chain_base_sensors;
  Couple<rdyn::ChainPtr> m_chain_world_tools;

  struct Side {
    enum { LEFT = 0, RIGHT = 1, COMMON = 2, BASE = 3 };
    constexpr static std::array<int, 2> arms() { return std::array<int, 2>({{LEFT, RIGHT}}); }
  };

  std::vector<std::string> m_joint_names;

  Couple<size_t> m_nax_s;            // number of axis for each single full chain
  std::array<size_t, 4> m_split_nax; // number of axis for each part. Order based on `Side`
  std::array<size_t, 3> m_idx_st;
  size_t m_nax;      // number of axis, without mobile base
  size_t m_full_nax; // number of axis + 3 for mobile base

  Eigen::VectorXd m_q;
  Eigen::VectorXd m_qp;
  Eigen::VectorXd m_qpp;
  Couple<std::vector<unsigned int>> m_sel;

  double m_dt;

  double m_kp_joint_task, m_kv_joint_task;

  Eigen::MatrixXd m_W; // Weight matrix for CLIK
  
  Eigen::VectorXd m_initial_q;
  Eigen::Vector12d m_wrench_in_sensor_prec;

  Eigen::Vector6d m_admittance_value;

  Eigen::Affine3d m_T_world_base;

  Eigen::Vector12d m_offset_wrench_sensor_in_sensor;
  Eigen::Vector12d m_offset_wrench_tool_in_world;
  std::future<bool> m_offset_future;

  state_observer::KalmanFilter m_base_position_filter;
  state_observer::KalmanFilter m_joint_filter;

  // Required both for states and at least one for command
  const std::vector<std::string> m_allowed_interface_types {
                                                           hardware_interface::HW_IF_POSITION,
                                                           hardware_interface::HW_IF_VELOCITY};
  Couple<bool> m_used_command_interfaces;

  struct FloatBaseData {
    bool enabled {true};
    size_t nax() const { return enabled ? nax_ : 0; }
    std::vector<std::string> base_joint_names() { return enabled ? base_joint_names_ : std::vector<std::string>{}; }
    Eigen::Vector3d vel_limits;
    Eigen::Vector3d acc_limits;

  private:
    constexpr static size_t nax_ {3};
    const std::vector<std::string> base_joint_names_ {"move_x", "move_y", "rot_z"};

  } m_mobile_base;

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
  Eigen::Vector6d m_cartesian_pos_limits;

  std::unique_ptr<ElastoplasticModel> m_elastoplastic_model;

  struct ClikData {
    const Eigen::VectorXd &position_references, velocity_references;
    const Eigen::Vector12d& twist_tool_world_in_world;
    const Eigen::Vector6d& twist_shared_world_in_world;
    //, next_twist_tool_world_in_world;
    const std::array<Eigen::Affine3d, 2>& T_world_tool;
    const Eigen::Affine3d& T_world_shared;
    //, next_T_world_tool;
    const Eigen::Vector6d& target_acc_tool_target_in_world;
    const Eigen::Matrix12Xd& J_world_tools_in_world;
    // const Eigen::Matrix12Xd& J_base_tools_in_world;
    const Eigen::Affine3d& target_T_world_tool;
    const Eigen::Vector6d& target_twist_shared_world_in_world;
    const Eigen::Vector12d& wrench_tool_in_world;
    const Eigen::Vector6d& wrench_shared_in_world;
    bool got_new_odom;
  };

  Eigen::Vector6d m_zp;

  std::array<double, 12> m_deadbands;

  utils::Logistic m_logistic;
  double m_logis_prec;

  utils::SigmoidSys m_pos_task_slider;

  utils::interpolation::Interpolator m_interpolator;
  rclcpp::Subscription<moveit_msgs::msg::CartesianTrajectory>::SharedPtr m_carteisan_trj_sub;

  Eigen::Vector6d m_computed_target_acc_shared_world_in_world;
  Eigen::Vector6d m_computed_target_twist_shared_world_in_world;
  Eigen::Affine3d m_computed_target_T_world_shared;

  Eigen::Affine3d m_T_left_shared;
  Eigen::Affine3d m_T_right_shared_ideal;
  // Da rivedere
  Eigen::Affine3d get_shared_frame(const Eigen::Affine3d& T_world_left, const Eigen::Affine3d& T_world_right) {
    // return T_world_left * m_T_left_shared;
    Eigen::Affine3d T_world_shared;
    T_world_shared.translation() = 0.5 * (T_world_left.translation() + T_world_right.translation());
    Eigen::AngleAxisd AA_left_right(T_world_left.linear().transpose() * T_world_right.linear());
    AA_left_right.angle() *= 0.5;
    T_world_shared.linear() = T_world_left.linear() * AA_left_right.matrix();
    return T_world_shared;
  }

  Eigen::Vector6d get_wrench(const int side) {
    geometry_msgs::msg::Wrench w;
    m_ft_sensors[side]->get_values_as_message(w);
    double s = m_parameters.ft_invert_sign ? -1.0 : 1.0;
    return Eigen::Vector6d({s * w.force.x, s * w.force.y, s * w.force.z, s * w.torque.x, s * w.torque.y, s * w.torque.z});
  }

  Eigen::Vector12d get_wrenches() { return (Eigen::Vector12d() << get_wrench(Side::LEFT), get_wrench(Side::RIGHT)).finished(); }

  void update_grasp_matrices(const Eigen::Matrix3d& R_world_shared);

  Eigen::Matrix612d m_grasp_matrix_wrench;
  Eigen::Matrix612d m_grasp_matrix_twist;
  Eigen::Vector3d m_p_left_shared_in_shared;  // Virtual stick
  Eigen::Vector3d m_p_right_shared_in_shared; // Virtual stick

  bool m_base_use_cmd_ifaces;

  std::mutex m_mutex;
  Eigen::Affine3d m_T_world_shared;

public:
  ElastoplasticControllerDual() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  // controller_interface::CallbackReturn on_shutdown(
  //     const rclcpp_lifecycle::State & previous_state) override;

  bool ready_for_activation()
  {
    return m_robot_description_configuration == RDStatus::OK;
  }

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
  override;

  void configure_after_robot_description_callback(const std_msgs::msg::String::SharedPtr msg);

  std::optional<Eigen::VectorXd> clik(const ClikData& data);
  Eigen::VectorXd compute_clik_as_qp(const ClikData &data, const Eigen::Vector6d &a_position_error,
                                     const Eigen::Vector6d &a_twist_error, const Eigen::Vector6d &a_acc_non_linear);
  Eigen::VectorXd compute_clik_as_inv(const ClikData &data, const Eigen::Vector6d &a_position_error,
                                      const Eigen::Vector6d &a_twist_error, const Eigen::Vector6d &a_acc_non_linear);

  bool write_cmd_vel(const Eigen::Vector3d& v);
  void get_target_callback(const geometry_msgs::msg::Twist& msg);
  void get_mobile_base_target_callback(const geometry_msgs::msg::Twist & msg);
  void get_odometry_callback(const nav_msgs::msg::Odometry & msg);
  void get_localization_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
};
}

#endif // ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
