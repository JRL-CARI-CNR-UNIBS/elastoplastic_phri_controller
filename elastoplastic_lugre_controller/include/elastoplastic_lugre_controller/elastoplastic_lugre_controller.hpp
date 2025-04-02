#ifndef ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
#define ELASTOPLASTIC_LUGRE_CONTROLLER_HPP

#include "elastoplastic_model_6d.hpp"
#include "elastoplastic_parameters.hpp"
#include "rdyn_core/primitives.h"

#include "Eigen/Core"

#include "controller_interface/chainable_controller_interface.hpp"
#include "eiquadprog/eiquadprog-fast.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "semantic_components/force_torque_sensor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"


// #include "derivatives.hpp"

namespace elastoplastic
{

class ElastoplasticController : public controller_interface::ChainableControllerInterface
{
private:
  std::shared_ptr<elastoplastic_controller::ParamListener> m_param_listener;
  elastoplastic_controller::Params m_parameters;

  template<typename T>
  using InterfaceReference = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReference<hardware_interface::LoanedStateInterface> m_joint_state_interfaces;
  InterfaceReference<hardware_interface::LoanedCommandInterface> m_joint_command_interfaces;
  InterfaceReference<hardware_interface::LoanedStateInterface> m_mobile_base_state_interfaces;
  InterfaceReference<hardware_interface::LoanedCommandInterface> m_mobile_base_command_interfaces;

  size_t m_joint_reference_interfaces_size;

  std::unique_ptr<semantic_components::ForceTorqueSensor> m_ft_sensor;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub_mobile_base_target;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_mobile_base_odometry;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_sub_mobile_base_pose;

  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> m_rt_buffer_mobile_base_target;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseWithCovarianceStamped>
      m_rt_buffer_base_pose_in_world;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistWithCovariance>
      m_rt_buffer_base_twist_in_base;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> m_rt_buffer_base_odom;

  rclcpp::Time m_last_odom_msg_time;
  rclcpp::Time m_last_localization_msg_time;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cmd_vel;

  // Debug publishers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_pub_friction_in_world;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_pub_wrench_in_world;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_pub_wrench_in_tool;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cart_vel_error;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_delta_acceleration;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_delta_velocity;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_delta_pose;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_twist_in_world;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr m_xp_pub;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr m_pub_joint_reference;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pub_fk_world_tool;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pub_fk_base_tool;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pub_next_pose;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pub_z;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pub_w;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_clik_components_pub;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_clik_correction_pub;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pub_weights;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr m_pub_alfa;

  constexpr static double M_POSITION_TOLLERANCE = 1e-4;
  constexpr static double M_VELOCITY_TOLLERANCE = 1e-5;
  constexpr static double M_MINIMUM_SAMPLING_TIME = 1e-4;
  constexpr static unsigned int M_CARTESIAN_DIM = 6;
  constexpr static double M_SLACK_GAIN = 1e2;

  enum class RDStatus
  {
    OK,
    ERROR,
    EMPTY
  } m_robot_description_configuration {ElastoplasticController::RDStatus::EMPTY};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub_robot_description;

  rdyn::ChainPtr m_chain_base_tool;
  rdyn::ChainPtr m_chain_base_sensor;
  rdyn::ChainPtr m_chain_world_base;
  rdyn::ChainPtr m_chain_world_tool;

  std::vector<std::string> m_joint_names;

  size_t m_nax;
  size_t m_full_nax;

  Eigen::VectorXd m_q;
  Eigen::VectorXd m_qp;
  Eigen::VectorXd m_qpp;

  double m_dt;

  double m_kp_last_task, m_kv_last_task;

  Eigen::MatrixXd m_W; // Weight matrix for CLIK

  Eigen::VectorXd m_q_prec;
  Eigen::VectorXd m_qp_prec;
  Eigen::VectorXd m_qpp_prec;
  Eigen::Vector6d m_wrench_in_sensor_prec;

  Eigen::Affine3d m_T_world_base;
  Eigen::Affine3d m_T_world_tool_initial;

  // Required both for states and at least one for command
  const std::vector<std::string> m_allowed_interface_types {
                                                           hardware_interface::HW_IF_POSITION,
                                                           hardware_interface::HW_IF_VELOCITY};
  std::array<bool, 2> m_used_command_interfaces;


  struct FloatBaseData
  {
    bool enabled {true};

    std::string ns;
    Eigen::Vector3d velocity_in_base;
    size_t nax() const { return enabled ? nax_ : 0; }
    std::vector<std::string> base_joint_names() { return enabled ? base_joint_names_ : std::vector<std::string>{}; }
    Eigen::Vector3d vel_limits;
    Eigen::Vector3d acc_limits;

  private:
    constexpr static size_t nax_ {3};
    const std::vector<std::string> base_joint_names_ {"move_x", "move_y", "rot_z"};

  } m_mobile_base;

  bool m_mobile_base_pose_updated;

  std::vector<std::string> m_state_interfaces_names;
  std::vector<std::string> m_command_interfaces_names;


  struct Limits
  {
    Eigen::VectorXd pos_upper;
    Eigen::VectorXd pos_lower;
    Eigen::VectorXd vel;
    Eigen::VectorXd acc;
  } m_limits;

  std::unique_ptr<ElastoplasticModel6D> m_elastoplastic_model;

  struct IntegralState
  {
    Eigen::Vector6d position;
    Eigen::Vector6d velocity;
    void clear() {position.setZero(); velocity.setZero();}
  } m_delta_elastoplastic_in_world;

  struct ClikData {
    Eigen::VectorXd&  position_references,
                      velocity_references;
    Eigen::Vector6d&  twist_tool_world_in_world,
                      next_twist_tool_world_in_world;
    Eigen::Affine3d&  T_world_tool,
                      next_T_world_tool;
    Eigen::Vector6d&  acc_tool_target_in_world;
    Eigen::Matrix6Xd& J_world_tool_in_world;
  };

  struct SatRelWeights {
    double max;
    double slope;
    Eigen::Array3d inflection;
  } m_saturation_relax_weight;

  eiquadprog::solvers::EiquadprogFast m_eiquadprog;

  //  std::array<Eigen::MatrixXd, 6> update_hessian(
  //    const Eigen::Matrix6Xd & jacobian,
  //    const Eigen::VectorXd & q);

  // tf2_ros::Buffer::SharedPtr m_tf_buffer;
  // std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

  // cppoptlib::Problem prb;

  // std::array<Eigen::MatrixXd, 6> m_hessian;
  // struct BFGSData
  // {
  //   Eigen::Matrix6Xd jacobian_p;
  //   std::array<Eigen::MatrixXd, 6> hessian_p;
  // } bfgs_prev;

public:
  ElastoplasticController() {}

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

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
  override;

  void configure_after_robot_description_callback(const std_msgs::msg::String::SharedPtr msg);

  Eigen::VectorXd compute_clik(const ClikData& data, const bool use_qp = true);
  Eigen::VectorXd compute_clik_as_qp(const ClikData &data, const Eigen::Vector6d &a_position_error,
                                     const Eigen::Vector6d &a_twist_error, const Eigen::Vector6d &a_acc_non_linear);
  Eigen::VectorXd compute_clik_as_inv(const ClikData &data, const Eigen::Vector6d &a_position_error,
                                      const Eigen::Vector6d &a_twist_error, const Eigen::Vector6d &a_acc_non_linear);

  void get_target_callback(const geometry_msgs::msg::Twist & msg);
  void get_mobile_base_target_callback(const geometry_msgs::msg::Twist & msg);
  void get_odometry_callback(const nav_msgs::msg::Odometry & msg);
  void get_localization_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  double relax_weights(const Eigen::Array3d& twist);
};

}

#endif // ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
