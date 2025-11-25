#include "adaptive_hqp/adaptive_hqp.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"

#include "control_toolbox/filters.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "urdfdom_headers/urdf_model/model.h" // IWYU pragma: export

#include "rclcpp/qos.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <control_toolbox/filters.hpp>
#include <elastoplastic_msgs/msg/detail/adaptive_hqp_controller_state__struct.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/multibody/joint/joint-planar.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <rclcpp/logging.hpp>
#include <urdf_parser/urdf_parser.h>
#include <urdf_world/types.h>

#ifdef USE_LATEST_ROS2_CONTROL
// #define GET_VALUE_FROM_INTERFACE(interface) interface.get_optional().value()
#define GET_VALUE_FROM_INTERFACE(interface) interface.get_value()
#else
#define GET_VALUE_FROM_INTERFACE(interface) interface.get_value()
#endif

// #define ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MOBILE_BASE
#define ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MANIPULATOR
// #define ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MANIPULATOR__USE_KALMAN
#define USE_CARTESIAN_REFERENCE

namespace elastoplastic {

namespace utils {

void toWrenchMsg(const Eigen::Vector6d &v, geometry_msgs::msg::Wrench &msg) {
  // msg.force = tf2::toMsg2(v.head<3>());
  // msg.torque = tf2::toMsg2(v.tail<3>());
  msg.force.x = v(0);
  msg.force.y = v(1);
  msg.force.z = v(2);
  msg.torque.x = v(3);
  msg.torque.y = v(4);
  msg.torque.z = v(5);
}

geometry_msgs::msg::Wrench toWrenchMsg(const Eigen::Vector6d &v) {
  geometry_msgs::msg::Wrench msg;
  toWrenchMsg(v, msg);
  return msg;
}

geometry_msgs::msg::WrenchStamped toWrenchStampedMsg(const Eigen::Vector6d &v) {
  geometry_msgs::msg::WrenchStamped msg;
  toWrenchMsg(v, msg.wrench);
  return msg;
}

void deadband(const double low, const double high, Eigen::Vector6d &wr) {
  // if (wr.head<3>().norm() < low) {
  // wr.head<3>().setZero();
  // } else {
  // wr.head<3>().normalized() * (wr.head<3>().norm() - low);
  // }
  // if (wr.tail<3>().norm() < high) {
  // wr.tail<3>().setZero();
  // } else {
  // wr.tail<3>().normalized() * (wr.tail<3>().norm() - high);
  // }
  for (int idx = 0; idx < 3; idx++) {
    if (std::abs(wr(idx)) < low) {
      wr(idx) = 0;
    } else {
      wr(idx) = sgn(wr(idx)) * (std::abs(wr(idx)) - low);
    }
  }
  for (int idx = 3; idx < 6; idx++) {
    if (std::abs(wr(idx)) < high) {
      wr(idx) = 0;
    } else {
      wr(idx) = sgn(wr(idx)) * (std::abs(wr(idx)) - high);
    }
  }
}

inline ElastoplasticModelData get_model_data(const adaptive_hqp::Params &params,
                                             const double update_rate) {
  ElastoplasticModelData data;
  std::copy(params.impedance.inertia.begin(), params.impedance.inertia.end(),
            data.inertia_inv.diagonal().begin());
  std::copy(params.impedance.k.begin(), params.impedance.k.end(),
            data.k.diagonal().begin());
  std::copy(params.impedance.d.begin(), params.impedance.d.end(),
            data.d.diagonal().begin());
  data.enable_axis = params.impedance.enable_axis;
  return data;
}
} // namespace utils

using namespace std::chrono_literals;

bool AdaptiveHQP::write_cmd_vel(const Eigen::Ref<Eigen::Vector3d> &v) {
  bool b = true;
  if (m_base_use_cmd_ifaces) {
    for (int idx = 0; idx < 3; ++idx) {
      b = b && m_mobile_base_command_interfaces.at(idx).get().set_value(v(idx));
    }
  } else {
    geometry_msgs::msg::Twist msg(
        rosidl_runtime_cpp::MessageInitialization::ZERO);
    msg.linear.x = v(0);
    msg.linear.y = v(1);
    msg.angular.z = v(2);
    if (!m_rt_pub_cmd_vel->tryPublish(msg)) {
      LOG_WARN_THROTTLE_COUNT(
          get_node()->get_logger(), get_node()->get_clock(), 1,
          "Missed publish on " << m_parameters.cmd_vel_topic);
      b = false;
    }
  }
  return b;
}

bool AdaptiveHQP::write_cmd_vel_zero() {
  bool b = true;
  if (m_base_use_cmd_ifaces) {
    for (int idx = 0; idx < 3; ++idx) {
      b = b && m_mobile_base_command_interfaces.at(idx).get().set_value(0.0);
    }
  } else {
    geometry_msgs::msg::Twist msg(
        rosidl_runtime_cpp::MessageInitialization::ZERO);
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.angular.z = 0.0;
    if (!m_rt_pub_cmd_vel->tryPublish(msg)) {
      LOG_WARN_THROTTLE_COUNT(
          get_node()->get_logger(), get_node()->get_clock(), 1,
          "Missed publish on " << m_parameters.cmd_vel_topic);
      b = false;
    }
  }
  return b;
}

controller_interface::CallbackReturn AdaptiveHQP::on_init() {
  m_param_listener =
      std::make_shared<adaptive_hqp::ParamListener>(this->get_node());
  RCLCPP_DEBUG(get_node()->get_logger(),
               "Elastoplastic controller correctly loaded");
  return controller_interface::CallbackReturn::SUCCESS;
}

void AdaptiveHQP::configure_after_robot_description_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  if (m_robot_description_configuration == RDStatus::OK) {
    RCLCPP_DEBUG(get_node()->get_logger(), "New robot_description ignored");
    return;
  }

  std::string robot_description = msg->data;
  if (robot_description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Missing robot_description by controller_manager");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  } else {
    RCLCPP_INFO(get_node()->get_logger(),
                "Robot description obtained correctly");
  }

  if (not get_node()->has_parameter("robot_description")) {
    get_node()->declare_parameter("robot_description", robot_description);
  }

  RCLCPP_INFO(get_node()->get_logger(), "[[DEBUG]] creation URDF");

  // RCLCPP_INFO(get_node()->get_logger(), "robot_description:\n%s",
  // robot_description.c_str());

  urdf::ModelInterfaceSharedPtr urdf_model;
  try {
    urdf_model = urdf::parseURDF(robot_description);
  } catch (std::exception &ex) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "ex: " << ex.what());
  }

  RCLCPP_INFO(get_node()->get_logger(), "URDF model created 1");
  if (not urdf_model) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot create URDF model from robot_description provided by "
                 "controller_manager");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  RCLCPP_INFO(get_node()->get_logger(), "URDF model created");

  Eigen::Vector3d gravity({m_parameters.gravity.at(0),
                           m_parameters.gravity.at(1),
                           m_parameters.gravity.at(2)});
  m_chain_base_tool = rdyn::createChain(*urdf_model, m_parameters.frames.base,
                                        m_parameters.frames.tool, gravity);
  m_chain_base_sensor = rdyn::createChain(*urdf_model, m_parameters.frames.base,
                                          m_parameters.frames.sensor, gravity);
  if (not m_chain_base_tool) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot create rdyn chain from base (%s) to tool (%s)",
                 m_parameters.frames.base.c_str(),
                 m_parameters.frames.tool.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  if (not m_chain_base_sensor) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot create rdyn chain from base (%s) to sensor (%s)",
                 m_parameters.frames.base.c_str(),
                 m_parameters.frames.sensor.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  if (m_mobile_base->enabled) {
    urdf::ModelInterfaceSharedPtr mobile_base_model =
        urdf::parseURDF(utils::MOBILE_BASE_URDF);
    m_chain_world_base =
        rdyn::createChain(*mobile_base_model, "x_base", "mount_link", gravity);

    m_chain_world_tool =
        rdyn::joinChains(m_chain_world_base, m_chain_base_tool);
  } else {
    m_chain_world_tool = rdyn::createChain(*urdf_model, m_parameters.frames.map,
                                           m_parameters.frames.tool, gravity);
  }

  if (!m_chain_world_tool) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot create rdyn chain from world to tool (%s)",
                 m_parameters.frames.tool.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  RCLCPP_INFO(get_node()->get_logger(), "RDyn chains created");

  // Check motor and transmission ratios array lengths
  if (m_parameters.motor_torque_constants.size() !=
      m_parameters.joints.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Motor torque constants vector has not the same size of the "
                 "joint vector: %ld != %ld",
                 m_parameters.motor_torque_constants.size(),
                 m_parameters.joints.size());
    m_robot_description_configuration = RDStatus::ERROR;
  }
  if (m_parameters.reduction_ratios.size() != m_parameters.joints.size()) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Reduction ratio constants vector has not the same size of the "
        "joint vector: %ld != %ld",
        m_parameters.reduction_ratios.size(), m_parameters.joints.size());
    m_robot_description_configuration = RDStatus::ERROR;
  }
  if (m_parameters.joints_static_friction.size() !=
      m_parameters.joints.size()) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Joint static friction constants vector has not the same size of the "
        "joint vector: %ld != %ld",
        m_parameters.joints_static_friction.size(), m_parameters.joints.size());
    m_robot_description_configuration = RDStatus::ERROR;
  }
  if (m_parameters.joints_damping.size() != m_parameters.joints.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Joint damping constants vector has not the same size of the "
                 "joint vector: %ld != %ld",
                 m_parameters.joints_damping.size(),
                 m_parameters.joints.size());
    m_robot_description_configuration = RDStatus::ERROR;
  }
  if (std::count_if(m_parameters.mobile_base.virtual_inertia.begin(),
                    m_parameters.mobile_base.virtual_inertia.end(),
                    [](const double &v) { return v > 0; }) != 3) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Virtual inertia constants vector have non-positive values");
    m_robot_description_configuration = RDStatus::ERROR;
  }
  if (std::count_if(m_parameters.mobile_base.virtual_damping.begin(),
                    m_parameters.mobile_base.virtual_damping.end(),
                    [](const double &v) { return v > 0; }) != 3) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Virtual damping constants vector have non-positive values");
    m_robot_description_configuration = RDStatus::ERROR;
  }

  // Pinocchio models
  // Full model
  pin::Model chain_world_tool_full;
  pin::urdf::buildModel(urdf_model, pin::JointModelPlanar(),
                        chain_world_tool_full);
  m_planar_joint_name = chain_world_tool_full.names[1];
  // Get unused joint names
  std::vector<std::string> joint_names_to_lock = chain_world_tool_full.names;
  joint_names_to_lock.erase(joint_names_to_lock.begin(),
                            joint_names_to_lock.begin() +
                                2); // Rimuovi planar joint dalla lista
  joint_names_to_lock.erase(
      std::remove_if(joint_names_to_lock.begin(), joint_names_to_lock.end(),
                     [this](const std::string &s) {
                       return std::find(m_parameters.joints.begin(),
                                        m_parameters.joints.end(),
                                        s) != m_parameters.joints.end();
                     }),
      joint_names_to_lock.end());
  // Get unused joint ids
  std::vector<pin::JointIndex> joint_id_to_lock(joint_names_to_lock.size());
  std::transform(joint_names_to_lock.begin(), joint_names_to_lock.end(),
                 joint_id_to_lock.begin(),
                 [&chain_world_tool_full](const auto &s) {
                   return chain_world_tool_full.getJointId(s);
                 });

  m_chain_world_tool_model =
      pin::buildReducedModel(chain_world_tool_full, joint_id_to_lock,
                             pin::neutral(chain_world_tool_full));
  m_chain_world_tool_model.gravity.linear() << gravity;
  m_chain_world_tool_data = pin::Data(m_chain_world_tool_model);

  RCLCPP_INFO(get_node()->get_logger(), "Pinocchio joints:");
  std::for_each(m_chain_world_tool_model.names.begin(),
                m_chain_world_tool_model.names.end(),
                [this](const std::string &s) {
                  RCLCPP_INFO(get_node()->get_logger(), "joint: %s", s.c_str());
                });

  m_tool_id = m_chain_world_tool_model.getFrameId(m_parameters.frames.tool);
  if (m_chain_world_tool_model.nframes == m_tool_id) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Frame %s does not exist in the model",
                 m_parameters.frames.tool.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  m_sensor_id = m_chain_world_tool_model.getFrameId(m_parameters.frames.sensor);
  if (m_chain_world_tool_model.nframes == m_sensor_id) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Frame %s does not exist in the model",
                 m_parameters.frames.sensor.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  m_base_id = m_chain_world_tool_model.getFrameId(m_parameters.frames.base);
  if (m_chain_world_tool_model.nframes == m_base_id) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Frame %s does not exist in the model",
                 m_parameters.frames.base.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  m_jnt_id.reserve(m_parameters.joints.size());
  for (const auto &jnt : m_parameters.joints) {
    m_jnt_id.push_back(m_chain_world_tool_model
                           .idx_qs[m_chain_world_tool_model.getJointId(jnt)]);
  }
  m_jnt_vs_id.reserve(m_parameters.joints.size());
  for (const auto &jnt : m_parameters.joints) {
    m_jnt_vs_id.push_back(
        m_chain_world_tool_model
            .idx_vs[m_chain_world_tool_model.getJointId(jnt)]);
  }
  // END - pinocchio models

  m_limits.pos_upper.resize(m_nax);
  m_limits.pos_lower.resize(m_nax);
  m_limits.vel.resize(m_nax);
  m_limits.acc.resize(m_nax);

  if (m_parameters.soft_limits.size() != m_nax) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "soft_limits parameters size != " << m_nax);
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  for (size_t ax = 0; ax < m_nax; ++ax) {
    m_limits.pos_upper(ax) =
        urdf_model->getJoint(m_parameters.joints.at(ax))->limits->upper -
        m_parameters.soft_limits.at(ax);
    m_limits.pos_lower(ax) =
        urdf_model->getJoint(m_parameters.joints.at(ax))->limits->lower +
        m_parameters.soft_limits.at(ax);

    if (utils::almost_zero(m_limits.pos_upper(ax)) &&
        utils::almost_zero(m_limits.pos_lower(ax))) {
      m_limits.pos_upper(ax) = std::numeric_limits<double>::infinity();
      m_limits.pos_lower(ax) = -std::numeric_limits<double>::infinity();
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Upper and Lower limits are both equal to 0, set +/- infinity");
    }

    m_limits.vel(ax) =
        urdf_model->getJoint(m_parameters.joints.at(ax))->limits->velocity;
    m_limits.acc(ax) =
        m_parameters.acceleration_limits_coeff * m_limits.vel(ax);
    RCLCPP_DEBUG(get_node()->get_logger(),
                 "Limits joint %ld: upper = %5.2f, lower = %5.2f, vel = %5.2f, "
                 "acc = %5.2f",
                 ax, m_limits.pos_upper(ax), m_limits.pos_lower(ax),
                 m_limits.vel(ax), m_limits.acc(ax));
  }
  RCLCPP_INFO(get_node()->get_logger(), "Kinematics limits: OK");

  std::string what;
  m_joint_names.resize(m_parameters.joints.size() + m_mobile_base->nax());
  std::ranges::copy(m_mobile_base->base_joint_names, m_joint_names.begin());
  std::ranges::copy(m_parameters.joints,
                    std::next(m_joint_names.begin(), m_mobile_base->nax()));
  m_chain_base_tool->setInputJointsName(m_parameters.joints, what);
  m_chain_base_sensor->setInputJointsName(m_parameters.joints, what);
  m_chain_world_tool->setInputJointsName(m_joint_names, what);

  RCLCPP_INFO(get_node()->get_logger(), "Kinematics: COMPLETED");

  m_robot_description_configuration = RDStatus::OK;
}

controller_interface::CallbackReturn
AdaptiveHQP::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  m_parameters = m_param_listener->get_params();

  // The parameter update_rate, if not defined, is provided by the
  // controller_manager
  auto update_rate = this->get_node()->get_parameter("update_rate").as_int();
  m_dt = 1.0 / double(update_rate);
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), "dt: " << m_dt);
  if (m_dt < M_MINIMUM_SAMPLING_TIME) {
    RCLCPP_FATAL(this->get_node()->get_logger(),
                 "dt: %.6f, too low. Minimum sampling time: %.6f", m_dt,
                 M_MINIMUM_SAMPLING_TIME);
    return controller_interface::CallbackReturn::ERROR;
  }
  m_elastoplastic_model = std::make_unique<ElastoplasticModel>(
      utils::get_model_data(m_parameters, update_rate));

  m_mobile_base =
      std::make_unique<FloatBaseData>(m_parameters.mobile_base.enabled);

  m_wrench_filters.reserve(6);
  m_low_pass_filters.reserve(6);
  for (int idx = 0; idx < 6; idx++) {
    // m_wrench_filters.emplace_back(m_parameters.wrench.notch_filter.fc,
    // m_parameters.wrench.notch_filter.Q,
    // get_update_rate());
    m_low_pass_filters.emplace_back();
  }

  m_full_nax = m_mobile_base->enabled
                   ? m_parameters.joints.size() + m_mobile_base->nax()
                   : m_parameters.joints.size();
  m_nax = m_parameters.joints.size();
  RCLCPP_DEBUG(this->get_node()->get_logger(),
               "Full NAx: %ld, Manipulator NAx: %ld", m_full_nax, m_nax);
  m_q.resize(m_full_nax);
  m_qp.resize(m_full_nax);
  m_qpp.resize(m_full_nax);

  m_q_in.resize(m_full_nax);
  m_qp_in.resize(m_full_nax);
  m_tau_in.resize(m_full_nax);

  m_q_prec.resize(m_full_nax);
  m_qp_prec.resize(m_full_nax);
  m_qpp_prec.resize(m_full_nax);

  if (std::ranges::min(m_parameters.impedance.inertia) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Inertia has negative values!");
    return controller_interface::CallbackReturn::FAILURE;
  }

  using namespace std::placeholders;
  m_mobile_base_pose_updated = false;
  m_sub_mobile_base_odometry =
      this->get_node()->create_subscription<nav_msgs::msg::Odometry>(
          m_parameters.mobile_base.odom, 10,
          std::bind(&AdaptiveHQP::get_odometry_callback, this, _1));

  m_got_new_base_pose.store(false);
  m_rt_buffer_base_local.initRT(geometry_msgs::msg::PoseWithCovarianceStamped(
      rosidl_runtime_cpp::MessageInitialization::ALL));
  m_sub_mobile_base_pose =
      this->get_node()
          ->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              m_parameters.mobile_base.localization_topic, 10,
              [this](const geometry_msgs::msg::PoseWithCovarianceStamped &msg) {
                m_rt_buffer_base_local.writeFromNonRT(msg);
                m_got_new_base_pose.store(true);
              });

  if (m_mobile_base->enabled) {
    m_mobile_base_pose_updated = true;
    m_pub_cmd_vel =
        this->get_node()->create_publisher<geometry_msgs::msg::Twist>(
            m_parameters.cmd_vel_topic, rclcpp::SystemDefaultsQoS());
    m_rt_pub_cmd_vel = std::make_unique<
        realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
        m_pub_cmd_vel);
  }

  if (m_parameters.wrench.source == "ft_sensor") {
    m_ft_source = FTSource::FT_SENSOR;
  } else if (m_parameters.wrench.source == "torque") {
    m_ft_source = FTSource::TORQUE;
    m_invert_torque = m_parameters.wrench.invert_torque ? -1 : 1;
  } else if (m_parameters.wrench.source == "topic") {
    m_ft_source = FTSource::TOPIC;
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "'%s' is not a valid source",
                 m_parameters.wrench.source.c_str());
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (m_ft_source == FTSource::FT_SENSOR) {
    m_ft_sensor = std::make_unique<semantic_components::ForceTorqueSensor>(
        m_parameters.ft_sensor_name);
  }

  m_pub_full_state =
      this->get_node()
          ->create_publisher<
              elastoplastic_msgs::msg::AdaptiveHQPControllerState>(
              "~/full_state", rclcpp::SensorDataQoS());

  m_state_interfaces_names.reserve(m_required_interface_types.size());
  m_command_interfaces_names.reserve(m_required_interface_types.size());

  for (const auto &interface : m_required_interface_types) {
    auto it = std::ranges::find(m_parameters.state_interfaces, interface);
    if (it == m_parameters.state_interfaces.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Missing State interfaces from parameters");
      return controller_interface::CallbackReturn::FAILURE;
    } else {
      m_state_interfaces_names.push_back(*it);
      RCLCPP_INFO(get_node()->get_logger(), "State interface name: %s",
                  (*it).c_str());
    }
  }
  if (m_ft_source == FTSource::TORQUE) {
    m_state_interfaces_names.push_back(hardware_interface::HW_IF_TORQUE);
    RCLCPP_INFO(get_node()->get_logger(), "State interface name: %s",
                hardware_interface::HW_IF_TORQUE);
  } else if (m_ft_source == FTSource::TOPIC) {
    m_wrench_topic_buffer.initRT(geometry_msgs::msg::Wrench(
        rosidl_runtime_cpp::MessageInitialization::ZERO));
    m_wrench_sub = get_node()->create_subscription<geometry_msgs::msg::Wrench>(
        FT_TOPIC, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::Wrench msg) {
          m_wrench_topic_buffer.writeFromNonRT(msg);
        });
  }

  for (const auto &interface : m_required_interface_types) {
    auto it = std::ranges::find(m_parameters.command_interfaces, interface);
    if (it != m_parameters.command_interfaces.end()) {
      m_command_interfaces_names.push_back(*it);
      RCLCPP_INFO(get_node()->get_logger(), "Command interface name: %s",
                  (*it).c_str());
    }
  }
  if (m_command_interfaces_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Missing Command interfaces from parameters");
    return controller_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Pre Robot description");

  // Robot description-related operations
  if (get_node()->has_parameter("robot_description")) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Robot description from parameter");
    std_msgs::msg::String::SharedPtr rd =
        std::make_shared<std_msgs::msg::String>();
    rd->data = get_node()->get_parameter("robot_description").as_string();
    configure_after_robot_description_callback(rd);
  } else {
#ifdef USE_LATEST_ROS2_CONTROL
    RCLCPP_DEBUG(get_node()->get_logger(),
                 "Robot description from controller manager");
    std_msgs::msg::String::SharedPtr rd =
        std::make_shared<std_msgs::msg::String>();
    rd->data = this->get_robot_description();
    configure_after_robot_description_callback(rd);
#else
    RCLCPP_DEBUG(get_node()->get_logger(), "Robot description from topic");
    rclcpp::QoS qos(1);
    qos.transient_local();
    m_robot_description_configuration = RDStatus::EMPTY;
    m_sub_robot_description =
        get_node()->create_subscription<std_msgs::msg::String>(
            m_parameters.robot_description_topic, qos,
            std::bind(&AdaptiveHQP::configure_after_robot_description_callback,
                      this, std::placeholders::_1));
#endif
  }

  RCLCPP_INFO(get_node()->get_logger(), "Pre Init vari");

  std::ranges::fill(m_used_command_interfaces, false);
  if (std::ranges::find(m_command_interfaces_names,
                        m_required_interface_types[0]) !=
      m_command_interfaces_names.end()) {
    m_used_command_interfaces.at(0) = true;
  }
  if (std::ranges::find(m_command_interfaces_names,
                        m_required_interface_types[1]) !=
      m_command_interfaces_names.end()) {
    m_used_command_interfaces.at(1) = true;
  }
  if (std::ranges::find(m_command_interfaces_names,
                        m_required_interface_types[2]) !=
      m_command_interfaces_names.end()) {
    m_used_command_interfaces.at(2) = true;
  }

  m_W.setIdentity(m_full_nax, m_full_nax);

  m_kp_joint_task = m_parameters.clik.joint_task.kp;
  m_kv_joint_task = m_parameters.clik.joint_task.kv;

  m_mobile_base->vel_limits = {m_parameters.mobile_base.max_vel.linear[0],
                               m_parameters.mobile_base.max_vel.linear[1],
                               m_parameters.mobile_base.max_vel.angular};
  m_mobile_base->acc_limits = {m_parameters.mobile_base.max_acc_x,
                               m_parameters.mobile_base.max_acc_y,
                               m_parameters.mobile_base.max_acc_yaw};

  m_carteisan_trj_sub =
      get_node()->create_subscription<moveit_msgs::msg::CartesianTrajectory>(
          m_parameters.cartesian_trajectory_topic, 1,
          [this](const moveit_msgs::msg::CartesianTrajectory &msg) {
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "got trajectory");
            if (msg.header.frame_id != m_parameters.frames.map) {
              RCLCPP_WARN_STREAM(get_node()->get_logger(),
                                 "Trajectory received but in wrong reference "
                                 "frame. Should be in {"
                                     << m_parameters.frames.map
                                     << "} but instead is in {"
                                     << msg.header.frame_id << "}. Skipping");
              return;
            }
            // if([this](const moveit_msgs::msg::CartesianTrajectoryPoint& p) ->
            // bool {
            //     return
            //     (p.point.pose.position.x -
            //     this->m_computed_target_T_world_tool.translation().x() >
            //     M_INITIAL_INTERPOLATOR_DELTA) || (p.point.pose.position.y -
            //     this->m_computed_target_T_world_tool.translation().y() >
            //     M_INITIAL_INTERPOLATOR_DELTA) || (p.point.pose.position.z -
            //     this->m_computed_target_T_world_tool.translation().z() >
            //     M_INITIAL_INTERPOLATOR_DELTA);
            //   } (msg.points.front())
            // ) {
            //   RCLCPP_WARN_STREAM(get_node()->get_logger(), "Trajectory first
            //   point is not the actual point!"); return;
            // }
            m_interpolator = utils::interpolation::Interpolator::from_msg(msg);
            RCLCPP_INFO_STREAM(get_node()->get_logger(),
                               "-> " << m_interpolator.is_empty());
          });

  m_rt_buffer_base_odom.initRT(
      nav_msgs::msg::Odometry(rosidl_runtime_cpp::MessageInitialization::ALL));

  RCLCPP_INFO(get_node()->get_logger(), "Pre Kalman");

  // Joint Kalman filter
  Eigen::MatrixXd kfjA(3 * m_nax, 3 * m_nax), kfjB(3 * m_nax, m_nax),
      kfjC(2 * m_nax, 3 * m_nax), kfjQ(3 * m_nax, 3 * m_nax),
      kfjR(2 * m_nax, 2 * m_nax);
  kfjA.setZero();
  kfjA.topLeftCorner(2 * m_nax, 2 * m_nax)
      << Eigen::MatrixXd::Identity(m_nax, m_nax),
      Eigen::MatrixXd::Identity(m_nax, m_nax) * m_dt,
      Eigen::MatrixXd::Zero(m_nax, m_nax),
      Eigen::MatrixXd::Identity(m_nax, m_nax);
  kfjA.bottomRightCorner(m_nax, m_nax).setIdentity();
  kfjB << Eigen::MatrixXd::Identity(m_nax, m_nax) * 0.5 * std::pow(m_dt, 2),
      Eigen::MatrixXd::Identity(m_nax, m_nax) * m_dt,
      Eigen::MatrixXd::Zero(m_nax, m_nax);
  kfjC.setZero();
  kfjC.leftCols(2 * m_nax).setIdentity();
  kfjC.bottomRightCorner(m_nax, m_nax).setIdentity();
  kfjQ.setIdentity();
  kfjQ.diagonal() << Eigen::Map<Eigen::VectorXd>(
      m_parameters.kalman_filter.manipulator.position.data(), m_nax),
      Eigen::Map<Eigen::VectorXd>(
          m_parameters.kalman_filter.manipulator.velocity.data(), m_nax),
      Eigen::VectorXd::Constant(m_nax, 1e-3);
  kfjR.setIdentity();
  m_joint_filter = state_observer::KalmanFilter(kfjA, kfjB, kfjC, kfjQ, kfjR);

  RCLCPP_INFO(get_node()->get_logger(), "Pre TF");

  // Map->base transform handling
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
  // m_tf_buffer->setUsingDedicatedThread(true);
  // m_tf_buffer->setCreateTimerInterface(
  // std::make_shared<tf2_ros::CreateTimerROS>(get_node()->get_node_base_interface(),
  // get_node()->get_node_timers_interface()));
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(
      *m_tf_buffer, this->get_node(), true);
  // m_tf_base_pose_recovery_thread =
  // std::make_unique<std::thread>(&AdaptiveHQP::update_base_pose_from_tf,
  // this);
  if (m_mobile_base->enabled) {
    bool can_transform{false};
    do {
      can_transform = m_tf_buffer->canTransform(m_parameters.frames.map,
                                                m_parameters.frames.base,
                                                tf2::TimePointZero);
    } while (!can_transform);
    RCLCPP_INFO(get_node()->get_logger(), "Found transform from map to base");
    m_T_world_base = tf2::transformToEigen(m_tf_buffer->lookupTransform(
        m_parameters.frames.map, m_parameters.frames.base, tf2::TimePointZero));
  } else {
    m_T_world_base.setIdentity();
  }

  m_base_use_cmd_ifaces = m_parameters.mobile_base.use_command_interfaces;

  RCLCPP_INFO(get_node()->get_logger(), "on_configure() completed");

  return controller_interface::CallbackReturn::SUCCESS;
}

void AdaptiveHQP::update_base_pose_from_tf() {
  // m_node_support =
  // rclcpp::Node::make_shared("elastoplastic_controller_support_node",
  // fmt::format("{}{}", this->get_node()->get_namespace(),
  // this->get_node()->get_name())); RCLCPP_INFO(m_node_support->get_logger(),
  // "Support node created..."); m_tf_listener =
  // std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer, m_node_support,
  // false); RCLCPP_INFO(m_node_support->get_logger(), "listener created...");
  // m_support_node_exec =
  // std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  // RCLCPP_INFO(m_node_support->get_logger(), "Executor created...");
  // m_support_node_exec->add_node(m_node_support);
  // RCLCPP_INFO(m_node_support->get_logger(), "Node added. Start spinning...");
  // m_support_node_exec->spin();
}

controller_interface::InterfaceConfiguration
AdaptiveHQP::state_interface_configuration() const {
  RCLCPP_INFO(get_node()->get_logger(), "Starting state interface export");
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interface_configuration.names.reserve(
      m_parameters.joints.size() * m_required_interface_types.size() + 6);

  for (const auto &jnt : m_parameters.joints) {
    state_interface_configuration.names.emplace_back(
        fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION));
    RCLCPP_INFO(get_node()->get_logger(), "State Interface (position): %s",
                state_interface_configuration.names.back().c_str());
  }
  for (const auto &jnt : m_parameters.joints) {
    state_interface_configuration.names.emplace_back(
        fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
    RCLCPP_INFO(get_node()->get_logger(), "State Interface (velocity): %s",
                state_interface_configuration.names.back().c_str());
  }
  for (const auto &jnt : m_parameters.joints) {
    state_interface_configuration.names.emplace_back(
        fmt::format("{}/{}", jnt, hardware_interface::HW_IF_EFFORT));
    RCLCPP_INFO(get_node()->get_logger(), "State Interface (effort): %s",
                state_interface_configuration.names.back().c_str());
  }

  // for (const auto& jnt : m_parameters.mobile_base.joints) {
  //   state_interface_configuration.names.emplace_back(fmt::format("{}/{}",
  //   jnt, hardware_interface::HW_IF_VELOCITY));
  // }

  if (m_ft_source == FTSource::TORQUE) {
    for (const auto &jnt : m_parameters.joints) {
      state_interface_configuration.names.emplace_back(
          fmt::format("{}/{}", jnt, hardware_interface::HW_IF_TORQUE));
      RCLCPP_INFO(get_node()->get_logger(), "State Interface: %s",
                  state_interface_configuration.names.back().c_str());
    }
  } else if (m_ft_source == FTSource::FT_SENSOR) {
    std::vector<std::string> ft_interfaces =
        m_ft_sensor->get_state_interface_names();
    state_interface_configuration.names.insert(
        state_interface_configuration.names.end(), ft_interfaces.begin(),
        ft_interfaces.end());
    RCLCPP_INFO(get_node()->get_logger(), "State Interface: %s",
                state_interface_configuration.names.back().c_str());
  }

  return state_interface_configuration;
}

controller_interface::InterfaceConfiguration
AdaptiveHQP::command_interface_configuration() const {
  RCLCPP_INFO(get_node()->get_logger(), "Starting command interface export");
  controller_interface::InterfaceConfiguration command_interface_configuration;
  command_interface_configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interface_configuration.names.reserve(
      m_parameters.joints.size() * m_command_interfaces_names.size());
  if (std::ranges::find(m_command_interfaces_names,
                        m_required_interface_types[0]) !=
      m_command_interfaces_names.end()) {
    for (const auto &jnt : m_parameters.joints) {
      command_interface_configuration.names.emplace_back(
          fmt::format("{}/{}", jnt, m_required_interface_types[0]));
      RCLCPP_INFO(get_node()->get_logger(), "Command Interface: %s",
                  command_interface_configuration.names.back().c_str());
    }
  }
  if (std::ranges::find(m_command_interfaces_names,
                        m_required_interface_types[1]) !=
      m_command_interfaces_names.end()) {
    for (const auto &jnt : m_parameters.joints) {
      command_interface_configuration.names.emplace_back(
          fmt::format("{}/{}", jnt, m_required_interface_types[1]));
      RCLCPP_INFO(get_node()->get_logger(), "Command Interface: %s",
                  command_interface_configuration.names.back().c_str());
    }
  }
  if (std::ranges::find(m_command_interfaces_names,
                        m_required_interface_types[2]) !=
      m_command_interfaces_names.end()) {
    for (const auto &jnt : m_parameters.joints) {
      command_interface_configuration.names.emplace_back(
          fmt::format("{}/{}", jnt, m_required_interface_types[2]));
      RCLCPP_INFO(get_node()->get_logger(), "Command Interface: %s",
                  command_interface_configuration.names.back().c_str());
    }
  }

  if (m_mobile_base->enabled && m_base_use_cmd_ifaces) {
    for (const auto &iface : m_parameters.mobile_base.command_interfaces) {
      // command_interface_configuration.names.emplace_back(fmt::format("{}/{}",
      // iface, hardware_interface::HW_IF_VELOCITY));
      command_interface_configuration.names.emplace_back(iface);
      RCLCPP_INFO(get_node()->get_logger(), "Command Interface: %s",
                  command_interface_configuration.names.back().c_str());
    }
  }

  return command_interface_configuration;
}

controller_interface::CallbackReturn
AdaptiveHQP::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  auto t_start = get_node()->get_clock()->now();
  while (!ready_for_activation() && !m_mobile_base_pose_updated &&
         get_node()->get_clock()->now() - t_start < std::chrono::seconds(10)) {
    RCLCPP_INFO(this->get_node()->get_logger(),
                "Waiting robot description-related operations");
    get_node()->get_clock()->sleep_for(std::chrono::milliseconds(1000));
  }
  if (m_robot_description_configuration != RDStatus::OK) {
    RCLCPP_ERROR(get_node()->get_logger(), "No robot description found");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (m_param_listener->is_old(m_parameters)) {
    m_parameters = m_param_listener->get_params();
    m_elastoplastic_model = std::make_unique<ElastoplasticModel>(
        utils::get_model_data(m_parameters, get_update_rate()));
    std_msgs::msg::String::SharedPtr rd =
        std::make_shared<std_msgs::msg::String>();
    rd->data = this->get_robot_description();
    configure_after_robot_description_callback(rd);

    if (m_parameters.wrench.notch_filter.enable) {
      // std::ranges::for_each(m_wrench_filters, [this](NotchFilter &f) {
      // f.configure(m_parameters.wrench.notch_filter.fc,
      // m_parameters.wrench.notch_filter.Q, get_update_rate());
      // });
      for (int idx = 0; idx < m_low_pass_filters.size(); idx++) {
        m_low_pass_filters.at(idx).activateFilter(
            m_parameters.wrench.deadband.at(0), 50.0,
            m_parameters.wrench.notch_filter.fc, m_dt, 0.0);
      }
      RCLCPP_INFO(get_node()->get_logger(), "Reconfigured Notch Filter");
    }
  }

  m_rt_pub_full_state = std::make_unique<realtime_tools::RealtimePublisher<
      elastoplastic_msgs::msg::AdaptiveHQPControllerState>>(m_pub_full_state);

  m_elastoplastic_model->clear();

  m_joint_state_interfaces.resize(3);
  m_joint_command_interfaces.resize(3);

  for (const auto &interface : m_required_interface_types) {
    auto it = std::ranges::find(m_required_interface_types, interface);
    auto idx = std::distance(m_required_interface_types.begin(), it);
    if (not controller_interface::get_ordered_interfaces(
            state_interfaces_, m_parameters.joints, interface,
            m_joint_state_interfaces.at(idx))) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Missing joints state interfaces: %ld names vs %ld interfaces",
          m_parameters.joints.size(), m_joint_state_interfaces.at(idx).size());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  assert(command_interfaces_.size() == m_parameters.joints.size());
  auto at_least_one_command_interface{false};
  for (const auto &interface : m_required_interface_types) {
    auto it = std::ranges::find(m_required_interface_types, interface);
    auto idx = std::distance(m_required_interface_types.begin(), it);
    if (not controller_interface::get_ordered_interfaces(
            command_interfaces_, m_parameters.joints, interface,
            m_joint_command_interfaces.at(idx))) {
      continue;
    }
    at_least_one_command_interface = true;
  }
  if (!at_least_one_command_interface) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Missing at least one joints command interface");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (m_ft_source == FTSource::FT_SENSOR) {
    if (!m_ft_sensor->assign_loaned_state_interfaces(state_interfaces_)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Cannot assing state interface to ft_sensor");
      return controller_interface::CallbackReturn::ERROR;
    }
  } else if (m_ft_source == FTSource::TORQUE) {
    if (not controller_interface::get_ordered_interfaces(
            state_interfaces_, m_parameters.joints,
            hardware_interface::HW_IF_TORQUE, m_joint_state_interfaces.at(2))) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Missing joints state interfaces (torque): %ld names vs %ld "
                   "interfaces",
                   m_parameters.joints.size(),
                   m_joint_state_interfaces.at(2).size());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  if (m_mobile_base->enabled && m_base_use_cmd_ifaces) {
    if (not controller_interface::get_ordered_interfaces(
            command_interfaces_, m_parameters.mobile_base.command_interfaces,
            "", m_mobile_base_command_interfaces)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Missing base controller command interfaces");
      return controller_interface::CallbackReturn::FAILURE;
    }
    // else {
    // for (const auto& iface : m_mobile_base_command_interfaces) {
    // RCLCPP_INFO_STREAM(get_node()->get_logger(), "mobile base iface: " <<
    // iface.get().get_name());
    // }
    // }
    // if (!controller_interface::get_ordered_interfaces(state_interfaces_,
    // m_parameters.mobile_base.joints,
    //                                                   hardware_interface::HW_IF_VELOCITY,
    //                                                   m_mobile_base_state_interfaces))
    //                                                   {
    //   RCLCPP_ERROR(get_node()->get_logger(), "Missing mobile base joints");
    //   return controller_interface::CallbackReturn::FAILURE;
    // };
  }

  RCLCPP_INFO(get_node()->get_logger(), "Command interfaces:");
  std::for_each(command_interfaces_.begin(), command_interfaces_.end(),
                [&](const auto &rif) {
                  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                                     "Interface: " << rif.get_name());
                });

  // Joint initialization
  std::transform(m_joint_state_interfaces.at(0).begin(),
                 m_joint_state_interfaces.at(0).end(), m_q.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface &lsi) {
                   return GET_VALUE_FROM_INTERFACE(lsi);
                 });
  std::transform(m_joint_state_interfaces.at(1).begin(),
                 m_joint_state_interfaces.at(1).end(), m_qp.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface &lsi) {
                   return GET_VALUE_FROM_INTERFACE(lsi);
                 });
  m_qpp.setZero();

  m_last_odom_msg_time = this->get_node()->get_clock()->now();

  if (m_mobile_base->enabled) {
    m_q.head<2>() = m_T_world_base.translation().head<2>();
    m_q(2) = utils::vector_from_affine(m_T_world_base)(5);
    m_qp.head<3>().setZero();
  }

  m_velocity_base_in_base.setZero();

  m_initial_q = m_q;

  // For debug purposes
  m_q_prec.setZero();
  m_qp_prec.setZero();
  m_qpp_prec.setZero();

  m_wrench_in_sensor_prec.setZero();
  m_admittance_value.setZero();

  // Per AHQP
  m_computed_target_T_world_tool = m_chain_world_tool->getTransformation(m_q);
  m_computed_target_acc_tool_world_in_world.setZero();
  m_computed_target_twist_tool_world_in_world.setZero();

  m_logis_prec = 0;

  m_offset_wrench_tool_in_world.setZero();
  m_offset_future = std::async(std::launch::async, [this](void) -> bool {
    // Compensate force offset
    if (utils::almost_zero(m_parameters.offset_force_window)) {
      return true;
    }
    Eigen::Vector6d offset_wrench;
    offset_wrench.setZero();
    const double offset_force_window =
        std::round(m_parameters.offset_force_window * get_update_rate());
    Eigen::Vector6d exp_filter_prec_state = Eigen::Vector6d::Zero();

    if (m_ft_source == FTSource::TORQUE) {
      // Wrench is already in world
      Eigen::Matrix6Xd J = m_chain_base_tool->getJacobian(m_q);
      Eigen::JacobiSVD<Eigen::Matrix6Xd> svd(J, Eigen::ComputeThinU |
                                                    Eigen::ComputeThinV);
      Eigen::VectorXd tau_j(m_nax);
      for (int idx = 0; idx < offset_force_window; ++idx) {
        std::transform(m_joint_state_interfaces.at(2).begin(),
                       m_joint_state_interfaces.at(2).end(),
                       tau_j.head(m_nax).begin(),
                       [](const hardware_interface::LoanedStateInterface &lsi) {
                         return GET_VALUE_FROM_INTERFACE(lsi);
                       });
        Eigen::Vector6d wr = get_wrench_from_torque(svd, tau_j);
        utils::deadband(m_parameters.wrench.deadband[0],
                        m_parameters.wrench.deadband[1], wr);
        std::transform(wr.begin(), wr.end(), offset_wrench.begin(),
                       offset_wrench.begin(), std::plus<double>{});
        std::this_thread::sleep_for(rclcpp::Rate(get_update_rate()).period());
      }
      offset_wrench /= offset_force_window;
    } else if (m_ft_source == FTSource::FT_SENSOR) {
      // Wrench is in sensor frame
      Eigen::Vector6d offset_wrench_sensor_in_sensor = Eigen::Vector6d::Zero();
      for (int idx = 0; idx < offset_force_window; ++idx) {
        Eigen::Vector6d wr = get_wrench_from_sensor();

        if (m_parameters.wrench.notch_filter.enable) {
          // std::transform(
          // wr.begin(), wr.end(), m_wrench_filters.begin(), wr.begin(),
          // [](const double w, NotchFilter &f) { return f.update(w); });
          std::transform(
              wr.begin(), wr.end(), m_low_pass_filters.begin(), wr.begin(),
              [](const double w, eigen_control_toolbox::FilteredScalar &f) {
                f.update(w);
                return f.getUpdatedValue();
              });
        }

        // Exponential filter
        std::transform(wr.begin(), wr.end(), exp_filter_prec_state.begin(),
                       wr.begin(), [this](const double w, const double w_prec) {
                         return filters::exponentialSmoothing(
                             w, w_prec, m_parameters.wrench.filter_alfa);
                       });

        // accumulate
        std::transform(
            wr.begin(), wr.end(), offset_wrench_sensor_in_sensor.begin(),
            offset_wrench_sensor_in_sensor.begin(), std::plus<double>{});

        std::this_thread::sleep_for(rclcpp::Rate(get_update_rate()).period());
      }
      offset_wrench_sensor_in_sensor /= offset_force_window;
      // utils::deadband(m_parameters.wrench.deadband[0],
      // m_parameters.wrench.deadband[1], offset_wrench_sensor_in_sensor);

      // Transform wrench offset in world
      Eigen::Vector6d offset_wrench_tool_in_tool =
          rdyn::spatialDualTranformation(
              offset_wrench_sensor_in_sensor,
              m_chain_base_tool->getTransformation(m_q.tail(m_nax)).inverse() *
                  m_chain_base_sensor->getTransformation(m_q.tail(m_nax)));

      offset_wrench =
          rdyn::spatialRotation(offset_wrench_tool_in_tool,
                                m_chain_world_tool->getTransformation(m_q)
                                    .linear()); // offset_wrench_tool_in_world
    } else if (m_ft_source == FTSource::TOPIC) {
      // Expected wrench_on_tool_in_tool
      Eigen::Vector6d offset_wrench_tool_in_tool = Eigen::Vector6d::Zero();
      for (int idx = 0; idx < offset_force_window; ++idx) {
        Eigen::Vector6d wr = get_wrench_from_topic();
        utils::deadband(m_parameters.wrench.deadband[0],
                        m_parameters.wrench.deadband[1], wr);
        std::transform(wr.begin(), wr.end(), offset_wrench_tool_in_tool.begin(),
                       offset_wrench_tool_in_tool.begin(), std::plus<double>{});
        std::this_thread::sleep_for(rclcpp::Rate(get_update_rate()).period());
      }
      offset_wrench_tool_in_tool /= offset_force_window;
      offset_wrench =
          rdyn::spatialRotation(offset_wrench_tool_in_tool,
                                m_chain_world_tool->getTransformation(m_q)
                                    .linear()); // offset_wrench_tool_in_world
    }
    m_offset_wrench_tool_in_world = offset_wrench;
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Wrench Offset computed: "
                           << m_offset_wrench_tool_in_world.transpose());
    return true;
  });

  if (m_ft_source == FTSource::FT_SENSOR) {
    Eigen::Affine3d T_base_tool =
        m_chain_base_tool->getTransformation(m_q.tail(m_nax));
    Eigen::Affine3d T_base_sensor =
        m_chain_base_sensor->getTransformation(m_q.tail(m_nax));
    m_T_tool_sensor = T_base_tool.inverse() * T_base_sensor;
  }

  Eigen::Matrix6d kfA, kfC, kfQ, kfR, kfP0;
  Eigen::Matrix<double, 6, 3> kfB;
  kfA << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity() * m_dt,
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();
  kfB << Eigen::Matrix3d::Identity() * 0.5 * std::pow(m_dt, 2),
      Eigen::Matrix3d::Identity() * m_dt;
  kfC.setIdentity();
  kfQ.setIdentity();
  kfR.setIdentity() * 1e2;
  kfP0.setIdentity() * 1e6;
  m_base_position_filter = state_observer::KalmanFilter(
      kfA, kfB, kfC,
      (Eigen::VectorXd(6) << m_q.head<3>(), m_qp.head<3>()).finished(), kfQ,
      kfR, kfP0);
  m_joint_filter.initialize((Eigen::VectorXd(3 * m_nax) << m_q.tail(m_nax),
                             m_qp.tail(m_nax), Eigen::VectorXd::Zero(m_nax))
                                .finished());

  // m_pos_task_slider.init(1, 1e4, 1e-5,
  // utils::Slider::SliderFunction::SIGMOID);
  std::transform(m_joint_state_interfaces.at(0).begin(),
                 m_joint_state_interfaces.at(0).end(), m_q.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface &lsi) {
                   return GET_VALUE_FROM_INTERFACE(lsi);
                 });
  std::transform(m_joint_state_interfaces.at(1).begin(),
                 m_joint_state_interfaces.at(1).end(), m_qp.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface &lsi) {
                   return GET_VALUE_FROM_INTERFACE(lsi);
                 });
  m_qpp.setZero();
  RCLCPP_DEBUG(get_node()->get_logger(), "Activated...");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdaptiveHQP::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  m_interpolator.end_plan();
  for (int idx = 0; idx < m_low_pass_filters.size(); idx++) {
    m_low_pass_filters.at(idx).deactivateFilter();
  }

  std::transform(m_joint_state_interfaces.at(0).begin(),
                 m_joint_state_interfaces.at(0).end(), m_q.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface &lsi) {
                   return GET_VALUE_FROM_INTERFACE(lsi);
                 });
  std::transform(m_joint_state_interfaces.at(1).begin(),
                 m_joint_state_interfaces.at(1).end(), m_qp.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface &lsi) {
                   return GET_VALUE_FROM_INTERFACE(lsi);
                 });
  m_qpp.setZero();

  m_computed_target_T_world_tool = m_chain_world_tool->getTransformation(m_q);
  m_computed_target_acc_tool_world_in_world.setZero();
  m_computed_target_twist_tool_world_in_world.setZero();

  if (m_mobile_base->enabled) {
    Eigen::Vector6d empty = Eigen::Vector6d::Zero();
    geometry_msgs::msg::Twist cmd_vel = tf2::toMsg(empty);
    m_pub_cmd_vel->publish(cmd_vel);
    write_cmd_vel_zero();
  }

  m_rt_pub_full_state->stop();

  m_elastoplastic_model->clear();

  m_joint_state_interfaces.clear();
  m_joint_command_interfaces.clear();
  m_mobile_base_command_interfaces.clear();
  // m_mobile_base_state_interfaces.clear();

  if (m_ft_source == FTSource::FT_SENSOR) {
    m_ft_sensor->release_interfaces();
  }

  m_wrench_in_sensor_prec.setZero();

  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdaptiveHQP::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
  // if (m_support_node_exec->is_spinning()) {
  //   m_support_node_exec->cancel();
  // }
  // if (m_tf_base_pose_recovery_thread->joinable())
  //   m_tf_base_pose_recovery_thread->join();
  // m_support_node_exec->remove_node(m_node_support);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdaptiveHQP::on_error(const rclcpp_lifecycle::State &previous_state) {
  return AdaptiveHQP::on_deactivate(previous_state);
}

std::vector<hardware_interface::CommandInterface>
AdaptiveHQP::on_export_reference_interfaces() {
  reference_interfaces_.resize(m_parameters.joints.size() *
                               m_required_interface_types.size());
  std::vector<hardware_interface::CommandInterface> reference_interfaces{};
  for (const auto &hwi : m_required_interface_types) {
    for (const auto &jnt : m_parameters.joints) {

      reference_interfaces.push_back(hardware_interface::CommandInterface(
          get_node()->get_name(), fmt::format("{}/{}", jnt, hwi),
          &reference_interfaces_[reference_interfaces.size()]));
    }
  }

  return reference_interfaces;
}

controller_interface::return_type
AdaptiveHQP::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  /* "Joint trajectory available only in chainable mode with
   * joint_trajectory_controller" */

  std::copy(m_q.tail(m_nax).begin(), m_q.tail(m_nax).end(),
            reference_interfaces_.begin()); // position
  std::fill(std::next(reference_interfaces_.begin(), m_nax),
            reference_interfaces_.end(), 0.0); // velocity

  return controller_interface::return_type::OK;
}

void AdaptiveHQP::get_odometry_callback(const nav_msgs::msg::Odometry &msg) {
  m_rt_buffer_base_odom.writeFromNonRT(msg);
}

controller_interface::return_type
AdaptiveHQP::update_and_write_commands(const rclcpp::Time & /*time*/,
                                       const rclcpp::Duration & /*period*/) {
  rclcpp::Time t_start = get_node()->get_clock()->now();

  // **********
  // ** Read **
  // **********

#ifdef ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MOBILE_BASE
  // Base state
  bool got_new_odom = false;
  if (m_mobile_base->enabled) {
    Eigen::Vector6d twist_base_world_in_world, twist_base_world_in_base;

    // try
    // {
    // geometry_msgs::msg::TransformStamped T_world_base_msg =
    // m_tf_buffer->lookupTransform(m_parameters.frames.map,
    // m_parameters.frames.base, tf2::TimePointZero);
    // m_T_world_base = tf2::transformToEigen(T_world_base_msg);
    // }
    // catch (tf2::LookupException &ex)
    // {
    // RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Could not get
    // transformation between " << m_parameters.frames.map << " and "
    // << m_parameters.frames.base
    // << ". Fallback on computed data");
    // }
    // catch (std::exception &)
    // {
    // RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Error while getting " <<
    // m_parameters.frames.map << " and "
    //  << m_parameters.frames.base
    //  << ". Fallback on computed data");
    // }
    if (m_got_new_base_pose.load()) {
      Eigen::fromMsg(m_rt_buffer_base_local.readFromRT()->pose.pose,
                     m_T_world_base);
      m_got_new_base_pose.store(false);
    }

    // NOTE: Support only inter-process comms
    nav_msgs::msg::Odometry odom_msg;
    [[maybe_unused]] rclcpp::MessageInfo msg_info;
    if (m_sub_mobile_base_odometry->take(odom_msg, msg_info)) {
      Eigen::fromMsg(odom_msg.twist.twist, twist_base_world_in_base);
      got_new_odom = true;
    } else {
      twist_base_world_in_base =
          utils::twist_from_base_velocity(m_velocity_base_in_base);
    }

    // // Recover twist from odometry
    // nav_msgs::msg::Odometry odom_msg = *(m_rt_buffer_base_odom.readFromRT());
    // if (rclcpp::Time(odom_msg.header.stamp) - m_last_odom_msg_time >
    // std::chrono::duration<double>(m_dt) ||
    // rclcpp::Time(odom_msg.header.stamp) - m_last_odom_msg_time <
    // std::chrono::seconds(0)) { twist_base_world_in_base =
    // utils::twist_from_base_velocity(m_velocity_base_in_base); } else {
    // Eigen::fromMsg(odom_msg.twist.twist, twist_base_world_in_base);
    // }
    // m_last_odom_msg_time = odom_msg.header.stamp;

    // Eigen::Vector4d wheel_vel;
    // std::transform(m_mobile_base_state_interfaces.begin(),
    // m_mobile_base_state_interfaces.end(), wheel_vel.begin(),
    // [](const hardware_interface::LoanedStateInterface& lsi) -> double {
    // return GET_VALUE_FROM_INTERFACE(lsi); }); twist_base_world_in_base =
    // utils::mecanum_direct_kinematics(wheel_vel,
    // m_parameters.mobile_base.wheel_radius,
    // m_parameters.mobile_base.sum_of_lx_and_ly);

    twist_base_world_in_world = rdyn::spatialRotation(twist_base_world_in_base,
                                                      m_T_world_base.linear());

    // Build state vectors
    Eigen::Vector6d estim_base =
        m_base_position_filter.predict(m_qpp.head<M_SE2>());
    if (got_new_odom) {
      Eigen::Vector6d base_read;
      base_read.tail<M_SE2>() =
          utils::base_velocity_from_twist(twist_base_world_in_world);
      base_read.head<2>() = m_T_world_base.translation().head<2>();
      base_read(2) = Eigen::AngleAxisd(m_T_world_base.linear()).angle();
      m_base_position_filter.update(base_read);
      estim_base = m_base_position_filter.get_state();
    }
    // Eigen::Vector6d estim_base = m_base_position_filter.update(base_read,
    // Eigen::Vector3d::Zero());
    m_qp.head<M_SE2>() = estim_base.tail<M_SE2>();
    m_q.head<M_SE2>() = estim_base.head<M_SE2>();
    // m_qp.head<M_SE2>() =
    // utils::base_velocity_from_twist(twist_base_world_in_world); m_q.head<2>()
    // = m_T_world_base.translation().head<2>(); m_q(2) =
    // Eigen::AngleAxisd(m_T_world_base.linear()).angle();
  }
#else
  bool got_new_odom = true;
#endif

  m_q_in.head<3>() = m_q.head<3>();
  m_qp_in.head<3>() = m_qp.head<3>();
  m_tau_in.head<3>().setZero();
  std::transform(m_joint_state_interfaces.at(0).begin(),
                 m_joint_state_interfaces.at(0).end(),
                 m_q_in.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface &lsi) {
                   return GET_VALUE_FROM_INTERFACE(lsi);
                 });
  std::transform(m_joint_state_interfaces.at(1).begin(),
                 m_joint_state_interfaces.at(1).end(),
                 m_qp_in.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface &lsi) {
                   return GET_VALUE_FROM_INTERFACE(lsi);
                 });
  std::transform(m_joint_state_interfaces.at(2).begin(),
                 m_joint_state_interfaces.at(2).end(),
                 m_tau_in.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface &lsi) {
                   return GET_VALUE_FROM_INTERFACE(lsi);
                 });
#ifdef ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MANIPULATOR
  // Manipulator State
#ifdef ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MANIPULATOR__USE_KALMAN
  q_qp_out = m_joint_filter.update(q_m_qp_in, m_qpp.tail(m_nax));
  m_q.tail(m_nax) = q_qp_out.head(m_nax);
  m_qp.tail(m_nax) = q_qp_out.tail(m_nax);
#else
  m_q.tail(m_nax) = m_q_in.tail(m_nax);
  m_qp.tail(m_nax) = m_qp_in.tail(m_nax);
#endif

#endif

  Eigen::Affine3d T_world_tool = m_chain_world_tool->getTransformation(m_q);
  Eigen::Vector6d twist_tool_world_in_world =
      m_chain_world_tool->getTwistTool(m_q, m_qp);
  Eigen::VectorXd full_position_references(m_full_nax),
      full_velocity_references(m_full_nax);

#ifndef USE_CARTESIAN_REFERENCE
  /* Joint Reference */
  // Target base
  Eigen::Vector6d target_twist_base_world_in_world;
  Eigen::Vector3d mobile_base_pose_in_world;
  if (m_mobile_base->enabled) {
    Eigen::Vector6d target_twist_base_world_in_base;
    Eigen::fromMsg(*(m_rt_buffer_mobile_base_target.readFromRT()),
                   target_twist_base_world_in_base);
    target_twist_base_world_in_world =
        move_from_base_to_world(target_twist_base_world_in_base);

    Eigen::Affine3d T_target_world_base;
    T_target_world_base = rdyn::spatialIntegration(
        m_T_world_base, target_twist_base_world_in_world, m_dt);
    mobile_base_pose_in_world << T_target_world_base.translation().head<2>(),
        Eigen::AngleAxisd(T_target_world_base.linear()).angle();
  } else {
    mobile_base_pose_in_world.setZero();
    target_twist_base_world_in_world.setZero();
  }

  // Target manipulator
  Eigen::VectorXd joint_position_references(m_nax);
  joint_position_references = Eigen::Map<Eigen::VectorXd>(
      reference_interfaces_.data(), m_parameters.joints.size());
  Eigen::VectorXd joint_velocity_references(m_nax);
  joint_velocity_references = Eigen::Map<Eigen::VectorXd>(
      std::next(reference_interfaces_.data(), m_nax), m_nax);
  if (m_mobile_base->enabled) {
    full_position_references.head(m_mobile_base->nax())
        << mobile_base_pose_in_world;
    full_velocity_references.head(m_mobile_base->nax())
        << utils::base_velocity_from_twist(target_twist_base_world_in_world);
  }
  full_position_references.tail(m_nax) << joint_position_references;
  full_velocity_references.tail(m_nax) << joint_velocity_references;

  Eigen::Vector6d target_twist_tool_world_in_world =
      m_chain_world_tool->getJacobian(full_position_references) *
      full_velocity_references;

#else

  full_position_references.tail(m_nax) = m_initial_q.tail(m_nax);
  full_velocity_references.setZero();

  /* Cartesian reference */
  if (!m_interpolator.is_plan_started() && m_interpolator.is_ready()) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Start interpolation of cartesian plan");
    m_interpolator.start_plan(get_node()->get_clock()->now());
  }
  // TODO: end plan

  Eigen::Vector6d reference_target_acc_tool_world_in_world;
  Eigen::Vector6d reference_target_twist_tool_world_in_world;
  Eigen::Affine3d reference_target_T_world_tool;
  if (m_interpolator.is_plan_started()) {
    auto status =
        m_interpolator.interpolate(get_node()->get_clock()->now(),
                                   reference_target_acc_tool_world_in_world,
                                   reference_target_twist_tool_world_in_world,
                                   reference_target_T_world_tool);
    if (status != utils::interpolation::Interpolator::InterpolationResult::OK) {
      reference_target_acc_tool_world_in_world.setZero();
      reference_target_twist_tool_world_in_world.setZero();
      reference_target_T_world_tool =
          m_chain_world_tool->getTransformation(m_initial_q);
    }
  } else {
    reference_target_acc_tool_world_in_world.setZero();
    reference_target_twist_tool_world_in_world.setZero();
    reference_target_T_world_tool =
        m_chain_world_tool->getTransformation(m_initial_q);
  }
  if (m_mobile_base->enabled) {
    full_velocity_references.head<M_SE2>() = utils::base_velocity_from_twist(
        m_computed_target_twist_tool_world_in_world);
    Eigen::Affine3d T_world_base_ref =
        m_chain_world_tool->getTransformationLink(m_initial_q,
                                                  m_parameters.frames.base);
    Eigen::Affine3d T_world_tool_ref =
        m_chain_world_tool->getTransformation(m_initial_q);
    full_position_references.head<M_SE2>() =
        utils::base_velocity_from_twist(utils::vector_from_affine(
            m_computed_target_T_world_tool * T_world_tool_ref.inverse() *
            T_world_base_ref));
  }

#endif

  /* FT state */
  Eigen::Vector6d wrench_tool_in_world;
  Eigen::Matrix6Xd J_world_tool_in_world = m_chain_world_tool->getJacobian(m_q);

  if (m_ft_source == FTSource::TORQUE) {
    Eigen::VectorXd tau_j(m_nax);
    Eigen::JacobiSVD<Eigen::Matrix6Xd> svd_torque(
        J_world_tool_in_world.transpose(),
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::transform(m_joint_state_interfaces.at(2).begin(),
                   m_joint_state_interfaces.at(2).end(),
                   tau_j.head(m_nax).begin(),
                   [](const hardware_interface::LoanedStateInterface &lsi) {
                     return GET_VALUE_FROM_INTERFACE(lsi);
                   });
    wrench_tool_in_world = get_wrench_from_torque(svd_torque, tau_j);

    utils::deadband(m_parameters.wrench.deadband[0],
                    m_parameters.wrench.deadband[1], wrench_tool_in_world);

    std::transform(wrench_tool_in_world.begin(), wrench_tool_in_world.end(),
                   m_wrench_in_sensor_prec.begin(),
                   wrench_tool_in_world.begin(),
                   [this](const double w, const double w_prec) {
                     return filters::exponentialSmoothing(
                         w, w_prec, m_parameters.wrench.filter_alfa);
                   });

    m_wrench_in_sensor_prec = wrench_tool_in_world;
  } else if (m_ft_source == FTSource::FT_SENSOR) {
    Eigen::Vector6d wrench_sensor_in_sensor;
    wrench_sensor_in_sensor = get_wrench_from_sensor();

    if (wrench_sensor_in_sensor.hasNaN()) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                           *this->get_node()->get_clock(), 1000,
                           "Force sensor contains NaN values. Full measure "
                           "discarded and replaced with zero");
      wrench_sensor_in_sensor.setZero();
    } else if (wrench_sensor_in_sensor.cwiseAbs().maxCoeff() > 1e20) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                           *this->get_node()->get_clock(), 1000,
                           "Force sensor contains overflowed values. Full "
                           "measure discarded and replaced with zero");
      wrench_sensor_in_sensor.setZero();
    }

    if (m_parameters.wrench.notch_filter.enable) {
      std::transform(
          wrench_sensor_in_sensor.begin(), wrench_sensor_in_sensor.end(),
          m_low_pass_filters.begin(), wrench_sensor_in_sensor.begin(),
          [](const double w, eigen_control_toolbox::FilteredScalar &f) {
            f.update(w);
            return f.getUpdatedValue();
          });
    }

    // Exponential filter
    std::transform(
        wrench_sensor_in_sensor.begin(), wrench_sensor_in_sensor.end(),
        m_wrench_in_sensor_prec.begin(), wrench_sensor_in_sensor.begin(),
        [this](const double w, const double w_prec) {
          return filters::exponentialSmoothing(w, w_prec,
                                               m_parameters.wrench.filter_alfa);
        });
    m_wrench_in_sensor_prec = wrench_sensor_in_sensor;

    Eigen::Vector6d wrench_tool_in_tool = rdyn::spatialDualTranformation(
        wrench_sensor_in_sensor, m_T_tool_sensor);
    wrench_tool_in_world =
        rdyn::spatialRotation(wrench_tool_in_tool, T_world_tool.linear()) -
        ((m_offset_future.wait_for(0s) != std::future_status::ready)
             ? Eigen::Vector6d::Zero()
             : m_offset_wrench_tool_in_world);

    utils::deadband(m_parameters.wrench.deadband[0],
                    m_parameters.wrench.deadband[1], wrench_tool_in_world);
  } else if (m_ft_source == FTSource::TOPIC) {
    // Expected wrench_on_tool_in_tool
    Eigen::Vector6d wrench_tool_in_tool;
    wrench_tool_in_tool = get_wrench_from_topic();

    if (wrench_tool_in_tool.hasNaN()) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                           *this->get_node()->get_clock(), 1000,
                           "Force sensor contains NaN values. Full measure "
                           "discarded and replaced with zero");
      wrench_tool_in_tool.setZero();
    } else if (wrench_tool_in_tool.cwiseAbs().maxCoeff() > 1e20) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                           *this->get_node()->get_clock(), 1000,
                           "Force sensor contains overflowed values. Full "
                           "measure discarded and replaced with zero");
      wrench_tool_in_tool.setZero();
    }

    // Wrench deadband
    utils::deadband(m_parameters.wrench.deadband[0],
                    m_parameters.wrench.deadband[1], wrench_tool_in_tool);

    if (m_parameters.wrench.notch_filter.enable) {
      std::transform(
          wrench_tool_in_tool.begin(), wrench_tool_in_tool.end(),
          m_low_pass_filters.begin(), wrench_tool_in_tool.begin(),
          [](const double w, eigen_control_toolbox::FilteredScalar &f) {
            f.update(w);
            return f.getUpdatedValue();
          });
    }

    // Exponential filter
    std::transform(wrench_tool_in_tool.begin(), wrench_tool_in_tool.end(),
                   m_wrench_in_sensor_prec.begin(), wrench_tool_in_tool.begin(),
                   [this](const double w, const double w_prec) {
                     return filters::exponentialSmoothing(
                         w, w_prec, m_parameters.wrench.filter_alfa);
                   });
    m_wrench_in_sensor_prec = wrench_tool_in_tool;

    wrench_tool_in_world =
        rdyn::spatialRotation(wrench_tool_in_tool, T_world_tool.linear()) -
        ((m_offset_future.wait_for(0s) != std::future_status::ready)
             ? Eigen::Vector6d::Zero()
             : m_offset_wrench_tool_in_world);
  }

  m_q_prec = m_q;
  m_qp_prec = m_qp;
  m_qpp_prec = m_qpp;

  // ************
  // ** Update **
  // ************

  Eigen::Vector6d cart_vel_error_tool_target_in_world;
  cart_vel_error_tool_target_in_world =
      (twist_tool_world_in_world - m_computed_target_twist_tool_world_in_world)
          .cwiseProduct(m_elastoplastic_model->get_enabled_axis());

  ClikData clik_data{.position_references = full_position_references,
                     .velocity_references = full_velocity_references,
                     .twist_tool_world_in_world = twist_tool_world_in_world,
                     .T_world_tool = T_world_tool,
                     .target_acc_tool_target_in_world =
                         reference_target_acc_tool_world_in_world,
                     .J_world_tool_in_world = J_world_tool_in_world,
                     .target_T_world_tool = reference_target_T_world_tool,
                     .target_twist_tool_world_in_world =
                         reference_target_twist_tool_world_in_world,
                     .wrench_tool_in_world = wrench_tool_in_world,
                     .got_new_odom = got_new_odom};

  std::optional<Eigen::VectorXd> solution_qp = clik(clik_data);
  Eigen::VectorXd tau_cmd(m_full_nax);

  // Calcolo offset
  if (m_offset_future.wait_for(0s) != std::future_status::ready) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         1000, "[Waiting] Computing Offset Force");
    tau_cmd = pin::computeGeneralizedGravity(m_chain_world_tool_model,
                                             m_chain_world_tool_data,
                                             to_pinocchio_config(m_q_in));
    tau_cmd += 10 * (m_initial_q - m_q_in) - 1.0 * m_qp_in;
    tau_cmd.head<3>().setZero();
    bool result{true};
    for (size_t idx = 0; idx < m_nax; ++idx) {

      if (m_used_command_interfaces.at(0)) {
        result &= m_joint_command_interfaces.at(0).at(idx).get().set_value(
            GET_VALUE_FROM_INTERFACE(
                m_joint_state_interfaces.at(0).at(idx).get()));
      }
      if (m_used_command_interfaces.at(1)) {
        result &= m_joint_command_interfaces.at(1).at(idx).get().set_value(0.0);
      } else if (m_used_command_interfaces.at(2)) {
        result &= m_joint_command_interfaces.at(2).at(idx).get().set_value(
            tau_cmd(3 + idx));
      }
    }
    if (!result) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Could not copy state interface position into command interfaces");
      return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
  }

  if (!solution_qp.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot find a solution for the CLIK QP problem. "
                 "Keeping actual position");
    assert(false); // Handled inside clik();
  } else {
    Eigen::VectorXd sol = solution_qp.value();
    // RCLCPP_INFO_STREAM(get_node()->get_logger(), "qepp\n " <<
    // sol.transpose());
    m_admittance_value = sol.tail<6>();
    m_qpp = sol.head(m_full_nax);
    tau_cmd = sol.segment(m_full_nax, m_full_nax);
  }

  std::tie(m_q, m_qp) = utils::rk4_double(
      [](const auto &, const auto &, const auto &u) { return u; }, m_q, m_qp,
      m_qpp, m_dt);

  // RCLCPP_INFO_STREAM(get_node()->get_logger(),
  //  "tau_cmd pre-PD -> " << tau_cmd.transpose());

  // if (m_mobile_base->enabled) {
  //   Eigen::Vector6d qp_base_in_world = Eigen::Vector6d::Zero();
  //   qp_base_in_world = utils::twist_from_base_velocity(m_qp.head<M_SE2>());

  //   Eigen::Vector6d qp_base_in_base = rdyn::spatialRotation(
  //       qp_base_in_world, m_T_world_base.linear().transpose());
  //   // qp_base_in_base = qp_base_in_base.unaryExpr([this](double vel) {
  //   return
  //   // std::abs(vel) < M_VELOCITY_TOLLERANCE ? 0.0 : vel;
  //   // });
  //   m_velocity_base_in_base =
  //   utils::base_velocity_from_twist(qp_base_in_base);

  //   // BEGIN - Check Saturation Base
  //   // If the QP works, this shouldn't be necessary
  //   for (size_t idx = 0; idx < M_SE2; ++idx) {
  //     if (std::abs(m_velocity_base_in_base(idx)) >
  //         m_mobile_base->vel_limits(idx)) {
  //       LOG_ERROR_THROTTLE_COUNT(
  //           this->get_node()->get_logger(), get_node()->get_clock(), 1,
  //           "Saturation of Velocity on base linear direction "
  //               << idx << ": " << m_velocity_base_in_base(idx) << " should be
  //               "
  //               << utils::sgn(m_velocity_base_in_base(idx)) *
  //                      m_mobile_base->vel_limits(idx));
  //       m_velocity_base_in_base(idx) =
  //           utils::sgn(m_velocity_base_in_base(idx)) *
  //           m_mobile_base->vel_limits(idx);
  //     }
  //   }
  //   m_qp.head<M_SE2>() =
  //   utils::base_velocity_from_twist(rdyn::spatialRotation(
  //       utils::twist_from_base_velocity(m_velocity_base_in_base),
  //       m_T_world_base.linear()));
  //   // END - Check Saturation Base
  // }
  m_velocity_base_in_base = m_qp.head<3>();

  // BEGIN - Saturation Manipulator
  for (size_t idx = 0; idx < m_nax; ++idx) {
    double q = m_q(idx + (m_full_nax - m_nax));
    double dq = m_qp(idx + (m_full_nax - m_nax));
    m_q(idx + (m_full_nax - m_nax)) = std::max(
        m_limits.pos_lower(idx) - m_parameters.soft_limits.at(idx),
        std::min(m_limits.pos_upper(idx) + m_parameters.soft_limits.at(idx),
                 m_q(idx + (m_full_nax - m_nax))));
    m_qp(idx + (m_full_nax - m_nax)) =
        std::max(-m_limits.vel(idx),
                 std::min(m_limits.vel(idx), m_qp(idx + (m_full_nax - m_nax))));
    if (!utils::almost_equal(q, m_q(idx + (m_full_nax - m_nax)))) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Saturation at %f of POSITION (cmd: %f) on manipulator joint "
                  "with index %ld",
                  q, m_q(idx + (m_full_nax - m_nax)), idx);
    }
    if (!utils::almost_equal(dq, m_qp(idx + (m_full_nax - m_nax)))) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Saturation of VELOCITY on manipulator joint with index %ld",
                  idx);
    }
  }
  // END - Saturation Manipulator

  Eigen::VectorXd cmd(m_full_nax);

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "q -> " << m_q.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "qp -> " << m_qp.transpose());

  if (m_used_command_interfaces.at(0)) {
    // Consider the case where in gazebo there is no PID on
    // position
    // if (get_node()->get_parameter("use_sim_time").as_bool()) {
    // cmd = m_qpp + m_parameters.clik.joint_task.kv * (m_qp - m_qp_in) +
    // m_parameters.clik.joint_task.kp * (m_q - m_q_in);
    // } else {
    cmd = m_q;
    // }
  } else if (m_used_command_interfaces.at(1)) {
    cmd = m_qp;
  } else if (m_used_command_interfaces.at(2)) {
    tau_cmd += m_parameters.clik.joint_task.kp * (m_q - m_q_in) +
               m_parameters.clik.joint_task.kv * (m_qp - m_qp_in);
    // tau_cmd += -m_parameters.clik.joint_task.kv * m_qp_in;
  }

  // FIXED PD
  // auto JJ = m_chain_base_tool->getJacobian(m_q);
  // tau_cmd = JJ.transpose() *
  //           (m_parameters.clik.joint_task.kp *
  //                utils::get_frame_distance(reference_target_T_world_tool,
  //                                          T_world_tool) -
  //            m_parameters.clik.joint_task.kv * twist_tool_world_in_world);

  // RCLCPP_INFO_STREAM(get_node()->get_logger(),
  //  "tau_cmd post-PD -> " << tau_cmd.transpose());

  // ***********
  // ** Write **
  // ***********
  bool is_value_set{true};
  if (m_used_command_interfaces.at(0)) {
    for (size_t ax = 0; ax < m_nax; ++ax) {
      is_value_set &= m_joint_command_interfaces.at(0).at(ax).get().set_value(
          cmd(ax + (m_full_nax - m_nax)));
    }
  }
  if (m_used_command_interfaces.at(1)) {
    for (size_t ax = 0; ax < m_nax; ++ax) {
      is_value_set &= m_joint_command_interfaces.at(1).at(ax).get().set_value(
          cmd(ax + (m_full_nax - m_nax)));
    }
  }
  if (m_used_command_interfaces.at(2)) {
    for (size_t ax = 0; ax < m_nax; ++ax) {
      is_value_set &= m_joint_command_interfaces.at(2).at(ax).get().set_value(
          tau_cmd(ax + (m_full_nax - m_nax)) /
          (m_parameters.reduction_ratios[ax] *
           m_parameters.motor_torque_constants[ax]));
    }
  }
  if (!is_value_set) {
    RCLCPP_FATAL_STREAM(get_node()->get_logger(),
                        "Could not write on the hardware interface! Halting!");
    this->on_deactivate(rclcpp_lifecycle::State());
    throw std::runtime_error("Controller crashed");
  }

  if (m_mobile_base->enabled) {
    Eigen::Vector6d base_twist_in_base =
        utils::twist_from_base_velocity(m_velocity_base_in_base);

    bool is_mobile_base_write_ok = write_cmd_vel(m_velocity_base_in_base);
    // if (!is_mobile_base_write_ok) {
    //   write_cmd_vel_zero();
    //   RCLCPP_ERROR_STREAM(get_node()->get_logger(),
    //                       "Problem occurred while writing on mobile base
    //                       interfaces! Stopping the movement");
    // }

    m_T_world_base =
        rdyn::spatialIntegration(m_T_world_base, base_twist_in_base, m_dt);
  }

  // *************
  // ** PUBLISH **
  // *************
  auto time_now = this->get_node()->get_clock()->now();
  elastoplastic_msgs::msg::AdaptiveHQPControllerState msg;

  msg.tau_cmd.resize(m_nax);
  std::copy(std::next(tau_cmd.begin(), m_mobile_base->nax()), tau_cmd.end(),
            msg.tau_cmd.begin());

  msg.header.stamp = time_now;
  msg.header.frame_id = m_parameters.frames.map;

  msg.z.reserve(6);
  msg.zp.reserve(6);
  Eigen::Vector6d msg_z = m_elastoplastic_model->z();
  std::copy(msg_z.begin(), msg_z.end(), std::back_inserter(msg.z));
  std::copy(m_zp.begin(), m_zp.end(), std::back_inserter(msg.zp));

  msg.cart_ref_pose = tf2::toMsg(reference_target_T_world_tool);
  msg.cart_ref_twist = tf2::toMsg(reference_target_twist_tool_world_in_world);

  msg.cart_actual_cmd_pose =
      tf2::toMsg(m_chain_world_tool->getTransformation(m_q));
  msg.cart_actual_cmd_twist =
      tf2::toMsg(m_chain_world_tool->getTwistTool(m_q, m_qp));
  msg.cart_actual_cmd_acc =
      tf2::toMsg(m_chain_world_tool->getDTwistTool(m_q, m_qp, m_qpp));

  msg.cart_actual_pose =
      tf2::toMsg(m_chain_world_tool->getTransformation(m_q_in));
  msg.cart_actual_twist =
      tf2::toMsg(m_chain_world_tool->getTwistTool(m_q_in, m_qp_in));

  std::tie(msg.reset_buffer_state, msg.reset_buffer_fill) =
      m_elastoplastic_model->get_reset_buffer_status();

  // Admittance state msg
  msg.admittance_state.wrench_base =
      utils::toWrenchStampedMsg(wrench_tool_in_world);
  msg.admittance_state.wrench_base.header.frame_id = m_parameters.frames.map;
  msg.admittance_state.wrench_base.header.stamp = time_now;

  Eigen::Affine3d fk = m_computed_target_T_world_tool;
  msg.admittance_state.admittance_position = tf2::eigenToTransform(fk);
  msg.admittance_state.admittance_position.header.stamp = time_now;
  msg.admittance_state.admittance_position.header.frame_id =
      m_parameters.frames.map;
  msg.admittance_state.admittance_position.child_frame_id =
      m_parameters.frames.tool;

  msg.admittance_state.admittance_velocity.twist =
      tf2::toMsg(m_computed_target_twist_tool_world_in_world);
  msg.admittance_state.admittance_velocity.header.stamp = time_now;
  msg.admittance_state.admittance_velocity.header.frame_id =
      m_parameters.frames.map;

  msg.admittance_state.admittance_acceleration.twist =
      tf2::toMsg(m_computed_target_acc_tool_world_in_world);
  msg.admittance_state.admittance_acceleration.header.stamp = time_now;
  msg.admittance_state.admittance_acceleration.header.frame_id =
      m_parameters.frames.map;

  msg.admittance_state.joint_state.header.stamp = time_now;
  msg.admittance_state.joint_state.name.reserve(m_joint_names.size());
  msg.admittance_state.joint_state.position.reserve(m_joint_names.size());
  msg.admittance_state.joint_state.velocity.reserve(m_joint_names.size());
  msg.admittance_state.joint_state.effort.reserve(m_joint_names.size());
  std::copy(m_joint_names.begin(), m_joint_names.end(),
            std::back_inserter(msg.admittance_state.joint_state.name));
  std::copy(m_q.begin(), m_q.end(),
            std::back_inserter(msg.admittance_state.joint_state.position));
  std::copy(m_qp.begin(), m_qp.end(),
            std::back_inserter(msg.admittance_state.joint_state.velocity));
  std::copy(m_qpp.begin(), m_qpp.end(),
            std::back_inserter(msg.admittance_state.joint_state.effort));

  msg.joint_reference.name.reserve(m_joint_names.size());
  msg.joint_reference.position.reserve(m_joint_names.size());
  msg.joint_reference.velocity.reserve(m_joint_names.size());
  std::ranges::copy(full_position_references,
                    std::back_inserter(msg.joint_reference.position));
  std::ranges::copy(full_velocity_references,
                    std::back_inserter(msg.joint_reference.velocity));

  msg.admittance_state.selected_axes.data.reserve(6);
  std::copy(m_elastoplastic_model->get_enabled_axis().begin(),
            m_elastoplastic_model->get_enabled_axis().end(),
            std::back_inserter(msg.admittance_state.selected_axes.data));
  msg.admittance_state.ft_sensor_frame.data = m_parameters.frames.sensor;
  msg.admittance_state.ref_trans_base_ft = tf2::eigenToTransform(
      m_chain_base_sensor->getTransformation(m_q.tail(m_nax)));
  msg.admittance_state.rot_base_control =
      tf2::toMsg(Eigen::Quaterniond(m_T_tool_sensor.linear()));
  msg.admittance_state.stiffness.data.reserve(6);
  msg.admittance_state.damping.data.reserve(6);

  auto [K, D] = m_elastoplastic_model->compute_variable_matrices(T_world_tool);
  Eigen::Vector6d K_diag = K.diagonal();
  std::copy(K_diag.begin(), K_diag.end(),
            std::back_inserter(msg.admittance_state.stiffness.data));
  std::copy(D.diagonal().begin(), D.diagonal().end(),
            std::back_inserter(msg.admittance_state.damping.data));

  utils::toWrenchMsg(m_admittance_value, msg.virtual_force);

  if (m_elastoplastic_model->is_plastic()) {
    msg.mode =
        elastoplastic_msgs::msg::AdaptiveHQPControllerState::MODE_PLASTIC;
  } else if (m_elastoplastic_model->to_restore()) {
    msg.mode =
        elastoplastic_msgs::msg::AdaptiveHQPControllerState::MODE_RESTORE;
  } else {
    msg.mode =
        elastoplastic_msgs::msg::AdaptiveHQPControllerState::MODE_ELASTIC;
  }

  if (m_rt_pub_full_state->trylock()) {
    m_rt_pub_full_state->msg_ = msg;
    m_rt_pub_full_state->unlockAndPublish();
  }

  // std::for_each(command_interfaces_.begin(), command_interfaces_.end(),
  // [&](const auto& rif) {
  //   RCLCPP_INFO_STREAM(get_node()->get_logger(), "Interface: " <<
  //   rif.get_name() << " - " << GET_VALUE_FROM_INTERFACE(rif));
  // });

  return controller_interface::return_type::OK;
}

} // namespace elastoplastic

PLUGINLIB_EXPORT_CLASS(elastoplastic::AdaptiveHQP,
                       controller_interface::ChainableControllerInterface);
