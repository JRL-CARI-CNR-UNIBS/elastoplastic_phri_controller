#ifndef ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
#define ELASTOPLASTIC_LUGRE_CONTROLLER_HPP

// local libs
#include "elastoplastic_lugre_controller/interpolation/interpolator.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"
#include <Eigen/src/Core/util/Constants.h>
#include <elastoplastic_lugre_controller/elastoplastic_parameters.hpp>
#include <elastoplastic_msgs/msg/elastoplastic_controller_state.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <state_space_filters/filtered_values.h>

// fundamental libs
#include "Eigen/Dense"

// other libs

// ros lib
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp" // IWYU pragma: export
#include "realtime_tools/realtime_publisher.hpp"
#include "semantic_components/force_torque_sensor.hpp"
#include <control_toolbox/pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf_world/types.h>

// ros msgs
// IWYU pragma: begin_keep
#include "elastoplastic_msgs/msg/admittance_hqp_controller_state.hpp"
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
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
// IWYU pragma: end_keep

namespace Eigen {
using Matrix6Xd = Matrix<double, 6, Dynamic>;
}

namespace elastoplastic {

namespace pin = pinocchio;

class ElastoplasticController
    : public controller_interface::ChainableControllerInterface {
private:
  std::shared_ptr<elastoplastic_controller::ParamListener> m_param_listener;
  elastoplastic_controller::Params m_parameters;

  template <typename T>
  using InterfaceReference = std::vector<std::reference_wrapper<T>>;

  std::vector<InterfaceReference<hardware_interface::LoanedStateInterface>>
      m_joint_state_interfaces;
  std::vector<InterfaceReference<hardware_interface::LoanedCommandInterface>>
      m_joint_command_interfaces;
  InterfaceReference<hardware_interface::LoanedCommandInterface>
      m_mobile_base_command_interfaces;

  std::unique_ptr<semantic_components::ForceTorqueSensor> m_ft_sensor;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      m_sub_mobile_base_target;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      m_sub_mobile_base_odometry;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      m_sub_mobile_base_pose;

  std::unique_ptr<std::thread> m_tf_base_pose_recovery_thread;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_cmd_vel;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>
      m_rt_pub_cmd_vel;
  rclcpp::Publisher<elastoplastic_msgs::msg::ElastoplasticControllerState>::
      SharedPtr m_pub_full_state;
  std::unique_ptr<realtime_tools::RealtimePublisher<
      elastoplastic_msgs::msg::ElastoplasticControllerState>>
      m_rt_pub_full_state;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr
      m_state_controller_publisher;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Int16>>
      m_rt_state_controller_publisher;
  int m_hqp_state;

  constexpr static double M_MINIMUM_SAMPLING_TIME{1e-4};
  constexpr static double M_INITIAL_INTERPOLATOR_DELTA{5e-3};
  constexpr static unsigned int M_SE3{6};
  constexpr static unsigned int M_SE2{3};

  std::string m_planar_joint_name;

  enum class RDStatus {
    OK,
    ERROR,
    EMPTY
  } m_robot_description_configuration{ElastoplasticController::RDStatus::EMPTY};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      m_sub_robot_description;

  pin::Model m_model;
  pin::Data m_model_data;

  std::vector<pin::JointIndex> m_jnt_id;
  std::vector<pin::JointIndex> m_jnt_vs_id;
  pin::FrameIndex m_tool_id;
  pin::FrameIndex m_sensor_id;
  pin::FrameIndex m_base_id;

  std::vector<std::string> m_joint_names;

  size_t m_arm_nax;

  std::unique_ptr<ElastoplasticModel> m_elastoplastic_model;

  Eigen::Vector6d m_zp;

  // Impedance state
  Eigen::VectorXd m_q;
  Eigen::VectorXd m_qp;
  Eigen::VectorXd m_qpp;

  // Real states
  Eigen::VectorXd m_q_in;
  Eigen::VectorXd m_qp_in;
  Eigen::VectorXd m_tau_in;

  Eigen::Affine3d m_computed_target_T_world_tool;
  Eigen::Vector6d m_computed_target_twist_tool_world_in_world;
  Eigen::Vector6d m_computed_target_acc_tool_world_in_world;


  Eigen::Vector6d m_reference_target_acc_tool_world_in_world;
  Eigen::Vector6d m_reference_target_twist_tool_world_in_world;
  Eigen::Affine3d m_reference_target_T_world_tool;

  Eigen::VectorXd m_full_position_references;
  Eigen::VectorXd m_full_velocity_references;

  double m_dt;

  Eigen::MatrixXd m_W; // Weight matrix for CLIK

  Eigen::VectorXd m_initial_q;
  Eigen::Vector6d m_wrench_in_sensor_prec;
  Eigen::Affine3d m_T_tool_sensor;
  Eigen::Affine3d m_ref_T_world_base;
  Eigen::Affine3d m_init_T_world_tool;

  Eigen::Vector6d m_admittance_value;

  pin::SE3 m_world_M_base;

  Eigen::Vector6d m_offset_wrench_tool_in_world;
  std::future<bool> m_offset_future;

  // Required both for states and at least one for command
  const std::vector<std::string> m_required_interface_types{
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT};
  const std::vector<std::string> m_allowed_interface_types{
      hardware_interface::HW_IF_EFFORT};
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

  std::vector<std::string> m_state_interfaces_names;
  std::vector<std::string> m_command_interfaces_names;

  struct CartesianImpedanceParams {
    Eigen::Matrix6d K;
    Eigen::Matrix6d D;
    Eigen::Matrix6d invM;
    Eigen::Vector6d enabled_axis;

    CartesianImpedanceParams()
        : K(Eigen::Matrix6d::Zero()), D(Eigen::Matrix6d::Zero()),
          invM(Eigen::Matrix6d::Zero()), enabled_axis(Eigen::Vector6d::Zero()) {
    }
  } m_impedance;

  struct Limits {
    Eigen::VectorXd pos_upper;
    Eigen::VectorXd pos_lower;
    Eigen::VectorXd vel;
    Eigen::VectorXd acc;
  } m_limits;


  std::vector<eigen_control_toolbox::FilteredScalar> m_low_pass_filters;

  elastoplastic::utils::interpolation::Interpolator m_interpolator;
  rclcpp::Subscription<moveit_msgs::msg::CartesianTrajectory>::SharedPtr
      m_carteisan_trj_sub;

  Eigen::Vector6d get_wrench_from_sensor() {
    geometry_msgs::msg::Wrench w;
    m_ft_sensor->get_values_as_message(w);
    double s = m_parameters.ft_invert_sign ? -1.0 : 1.0;
    return Eigen::Vector6d({s * w.force.x, s * w.force.y, s * w.force.z,
                            s * w.torque.x, s * w.torque.y, s * w.torque.z});
  }

  enum class SoTEnabled {
    TRACKING,
    ADMITTANCE,
    FROM_TRACKING_TO_ADMITTANCE,
    FROM_ADMITTANCE_TO_TRACKING
  } m_what_sot_is_active;

  bool m_base_use_cmd_ifaces;

public:
  ElastoplasticController() {}

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
  update_reference_from_subscribers() override;

  void configure_after_robot_description_callback(
      const std_msgs::msg::String::SharedPtr msg);

  std::optional<Eigen::VectorXd> optimize(Eigen::Vector6d &wrench);

  bool write_cmd_vel_zero();
  bool write_cmd_vel(const Eigen::Ref<Eigen::Vector3d> &v);
  void get_target_callback(const geometry_msgs::msg::Twist &msg);
  void get_mobile_base_target_callback(const geometry_msgs::msg::Twist &msg);
  //   void get_odometry_callback(const nav_msgs::msg::Odometry &msg);
  void get_localization_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
  void build_pinocchio_model(urdf::ModelInterfaceSharedPtr &urdf_model,
                             bool attach_mobile_base);
  Eigen::Vector6d get_wrench_in_world(pin::Data &data);
};
} // namespace elastoplastic

#endif // ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
