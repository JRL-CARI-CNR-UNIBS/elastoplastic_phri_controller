#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"

#include "control_toolbox/filters.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "urdfdom_headers/urdf_model/model.h"

#include "rclcpp/qos.hpp"

#include <algorithm>
#include <chrono>

void toWrenchMsg(const Eigen::Vector6d& v, geometry_msgs::msg::Wrench& msg) {
  // msg.force = tf2::toMsg2(v.head<3>());
  // msg.torque = tf2::toMsg2(v.tail<3>());
  msg.force.x = v(0);
  msg.force.y = v(1);
  msg.force.z = v(2);
  msg.torque.x = v(3);
  msg.torque.y = v(4);
  msg.torque.z = v(5);
}

geometry_msgs::msg::Wrench toWrenchMsg(const Eigen::Vector6d& v) {
  geometry_msgs::msg::Wrench msg;
  toWrenchMsg(v, msg);
  return msg;
}

geometry_msgs::msg::WrenchStamped toWrenchStampedMsg(const Eigen::Vector6d& v) {
  geometry_msgs::msg::WrenchStamped msg;
  toWrenchMsg(v, msg.wrench);
  return msg;
}

bool write_cmd_vel(std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ifs,
                   const Eigen::Vector3d& v) {
  bool b = true;
  for (int idx = 0; idx < 3; ++idx) {
    b &= ifs.at(idx).get().set_value(v(idx));
  }
  return b;
}

namespace elastoplastic {

using namespace std::chrono_literals;


controller_interface::CallbackReturn ElastoplasticController::on_init() {
  m_param_listener = std::make_shared<elastoplastic_controller::ParamListener>(this->get_node());
  RCLCPP_DEBUG(get_node()->get_logger(), "Elastoplastic controller correctly loaded");
  return controller_interface::CallbackReturn::SUCCESS;
}

void ElastoplasticController::configure_after_robot_description_callback(const std_msgs::msg::String::SharedPtr msg) {
  if (m_robot_description_configuration == RDStatus::OK) {
    RCLCPP_DEBUG(get_node()->get_logger(), "New robot_description ignored");
    return;
  }

  std::string robot_description = msg->data;
  if (robot_description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing robot_description by controller_manager");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Robot description obtained correctly");
  }

  if (not get_node()->has_parameter("robot_description")) {
    get_node()->declare_parameter("robot_description", robot_description);
  }

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(robot_description);
  if (not urdf_model) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot create URDF model from robot_description provided by controller_manager");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "URDF model created");

  Eigen::Vector3d gravity({m_parameters.gravity.at(0), m_parameters.gravity.at(1), m_parameters.gravity.at(2)});
  m_chain_base_tool = rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.tool, gravity);
  m_chain_base_sensor = rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.sensor, gravity);
  if (not m_chain_base_tool) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot create rdyn chain from base (%s) to tool (%s)",
                 m_parameters.frames.base.c_str(), m_parameters.frames.tool.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  if (not m_chain_base_sensor) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot create rdyn chain from base (%s) to sensor (%s)",
                 m_parameters.frames.base.c_str(), m_parameters.frames.sensor.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  if (m_mobile_base.enabled) {
    urdf::ModelInterfaceSharedPtr mobile_base_model = urdf::parseURDF(utils::MOBILE_BASE_URDF);
    rdyn::ChainPtr chain_world_base = rdyn::createChain(*mobile_base_model, "x_base", "mount_link", {0, 0, -9.806});

    m_chain_world_tool = rdyn::joinChains(chain_world_base, m_chain_base_tool);
  } else {
    m_chain_world_tool = rdyn::createChain(*urdf_model, m_parameters.frames.map, m_parameters.frames.tool, gravity);
  }

  if (not m_chain_world_tool) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot create rdyn chain from world to tool (%s)", m_parameters.frames.tool.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "RDyn chains created");

  m_limits.pos_upper.resize(m_nax);
  m_limits.pos_lower.resize(m_nax);
  m_limits.vel.resize(m_nax);
  m_limits.acc.resize(m_nax);

  if (m_parameters.soft_limits.size() != m_nax) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "soft_limits parameters size != " << m_nax);
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  for (size_t ax = 0; ax < m_nax; ++ax) {
    m_limits.pos_upper(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->upper - m_parameters.soft_limits.at(ax);
    m_limits.pos_lower(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->lower + m_parameters.soft_limits.at(ax);

    if (utils::almost_zero(m_limits.pos_upper(ax)) && utils::almost_zero(m_limits.pos_lower(ax))) {
      m_limits.pos_upper(ax) = std::numeric_limits<double>::infinity();
      m_limits.pos_lower(ax) = -std::numeric_limits<double>::infinity();
      RCLCPP_WARN(get_node()->get_logger(), "Upper and Lower limits are both equal to 0, set +/- infinity");
    }

    m_limits.vel(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->velocity;
    m_limits.acc(ax) = m_parameters.acceleration_limits_coeff * m_limits.vel(ax);
    RCLCPP_DEBUG(get_node()->get_logger(), "Limits joint %ld: upper = %5.2f, lower = %5.2f, vel = %5.2f, acc = %5.2f", ax,
                 m_limits.pos_upper(ax), m_limits.pos_lower(ax), m_limits.vel(ax), m_limits.acc(ax));
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Kinematics limits: OK");

  std::string what;
  m_joint_names.resize(m_parameters.joints.size() + m_mobile_base.nax());
  std::ranges::copy(m_mobile_base.base_joint_names(), m_joint_names.begin());
  std::ranges::copy(m_parameters.joints, std::next(m_joint_names.begin(), m_mobile_base.nax()));
  m_chain_base_tool->setInputJointsName(m_parameters.joints, what);
  m_chain_base_sensor->setInputJointsName(m_parameters.joints, what);
  m_chain_world_tool->setInputJointsName(m_joint_names, what);

  m_robot_description_configuration = RDStatus::OK;
}


controller_interface::CallbackReturn ElastoplasticController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  m_parameters = m_param_listener->get_params();

  if (m_parameters.debug.log) {
    this->get_node()->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }

  // The parameter update_rate, if not defined, is provided by the controller_manager
  auto update_rate = this->get_node()->get_parameter("update_rate").as_int();
  m_dt = 1.0 / double(update_rate);
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), "dt: " << m_dt);
  if (m_dt < M_MINIMUM_SAMPLING_TIME) {
    RCLCPP_FATAL(this->get_node()->get_logger(), "dt: %.6f, too low. Minimum sampling time: %.6f", m_dt, M_MINIMUM_SAMPLING_TIME);
    return controller_interface::CallbackReturn::ERROR;
  }
  m_elastoplastic_model = std::make_unique<ElastoplasticModel>(utils::get_model_data(m_parameters, update_rate));

  m_mobile_base.enabled = m_parameters.mobile_base.enabled;

  m_full_nax = m_mobile_base.enabled ? m_parameters.joints.size() + m_mobile_base.nax() : m_parameters.joints.size();
  m_nax = m_parameters.joints.size();
  RCLCPP_DEBUG(this->get_node()->get_logger(), "Full NAx: %ld, Manipulator NAx: %ld", m_full_nax, m_nax);
  m_q.resize(m_full_nax);
  m_qp.resize(m_full_nax);
  m_qpp.resize(m_full_nax);

  m_q_prec.resize(m_full_nax);
  m_qp_prec.resize(m_full_nax);
  m_qpp_prec.resize(m_full_nax);

  if (std::ranges::min(m_parameters.impedance.inertia) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Inertia has negative values!");
    return controller_interface::CallbackReturn::FAILURE;
  }

  using namespace std::placeholders;
  m_mobile_base_pose_updated = false;
  m_sub_mobile_base_odometry = this->get_node()->create_subscription<nav_msgs::msg::Odometry>(
    m_parameters.mobile_base.odom, 1, std::bind(&ElastoplasticController::get_odometry_callback, this, _1));
  if (m_mobile_base.enabled) {
    m_mobile_base_pose_updated = true;
    m_pub_cmd_vel =
      this->get_node()->create_publisher<geometry_msgs::msg::Twist>(m_parameters.cmd_vel_topic, rclcpp::SystemDefaultsQoS());
  }

  if (m_parameters.wrench.source == "ft_sensor") {
    m_ft_source = FTSource::FT_SENSOR;
  } else {
    m_ft_source = FTSource::TORQUE;
    m_invert_torque = m_parameters.wrench.invert_torque ? -1 : 1;
  }

  if (m_ft_source == FTSource::FT_SENSOR) {
    m_ft_sensor = std::make_unique<semantic_components::ForceTorqueSensor>(m_parameters.ft_sensor_name);
  }

  m_pub_full_state = this->get_node()->create_publisher<elastoplastic_msgs::msg::ElastoplasticControllerState>(
    "~/full_state", rclcpp::SensorDataQoS());

  m_state_interfaces_names.reserve(m_required_interface_types.size());
  m_command_interfaces_names.reserve(m_required_interface_types.size());

  for (const auto& interface : m_required_interface_types) {
    auto it = std::ranges::find(m_parameters.state_interfaces, interface);
    if (it == m_parameters.state_interfaces.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing State interfaces from parameters");
      return controller_interface::CallbackReturn::FAILURE;
    } else {
      m_state_interfaces_names.push_back(*it);
      RCLCPP_INFO(get_node()->get_logger(), "State interface name: %s", (*it).c_str());
    }
  }
  if (m_ft_source == FTSource::TORQUE) {
    auto it = std::ranges::find(m_parameters.state_interfaces, hardware_interface::HW_IF_TORQUE);
    if (it == m_parameters.state_interfaces.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing State interfaces from parameters");
      return controller_interface::CallbackReturn::FAILURE;
    } else {
      m_state_interfaces_names.push_back(*it);
      RCLCPP_INFO(get_node()->get_logger(), "State interface name: %s", (*it).c_str());
    }
  }

  for (const auto& interface : m_required_interface_types) {
    auto it = std::ranges::find(m_parameters.command_interfaces, interface);
    if (it != m_parameters.command_interfaces.end()) {
      m_command_interfaces_names.push_back(*it);
      RCLCPP_INFO(get_node()->get_logger(), "Command interface name: %s", (*it).c_str());
    }
  }
  if (m_command_interfaces_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing Command interfaces from parameters");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Robot description-related operations
  if (get_node()->has_parameter("robot_description")) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Robot description from parameter");
    std_msgs::msg::String::SharedPtr rd = std::make_shared<std_msgs::msg::String>();
    rd->data = get_node()->get_parameter("robot_description").as_string();
    configure_after_robot_description_callback(rd);
  } else {
#ifdef USE_LATEST_ROS2_CONTROL
    RCLCPP_DEBUG(get_node()->get_logger(), "Robot description from controller manager");
    std_msgs::msg::String::SharedPtr rd = std::make_shared<std_msgs::msg::String>();
    rd->data = this->get_robot_description();
    configure_after_robot_description_callback(rd);
#else
    RCLCPP_DEBUG(get_node()->get_logger(), "Robot description from topic");
    rclcpp::QoS qos(1);
    qos.transient_local();
    m_sub_robot_description = get_node()->create_subscription<std_msgs::msg::String>(
      m_parameters.robot_description_topic, qos,
      std::bind(&ElastoplasticController::configure_after_robot_description_callback, this, std::placeholders::_1));
    m_robot_description_configuration = RDStatus::EMPTY;
#endif
  }

  std::ranges::fill(m_used_command_interfaces, false);
  if (std::ranges::find(m_command_interfaces_names, m_required_interface_types[0]) != m_command_interfaces_names.end()) {
    m_used_command_interfaces.at(0) = true;
  }
  if (std::ranges::find(m_command_interfaces_names, m_required_interface_types[1]) != m_command_interfaces_names.end()) {
    m_used_command_interfaces.at(1) = true;
  }

  m_W.setIdentity(m_full_nax, m_full_nax);

  m_kp_joint_task = m_parameters.clik.joint_task.kp;
  m_kv_joint_task = m_parameters.clik.joint_task.kv;

  m_mobile_base.vel_limits = {m_parameters.mobile_base.max_vel.linear[0], m_parameters.mobile_base.max_vel.linear[1],
                              m_parameters.mobile_base.max_vel.angular};
  m_mobile_base.acc_limits = {m_parameters.mobile_base.max_acc_x, m_parameters.mobile_base.max_acc_y,
                              m_parameters.mobile_base.max_acc_yaw};

  m_logistic = {.max = m_parameters.impedance.logistic.max,
                .slope = m_parameters.impedance.logistic.slope,
                .inflection = m_parameters.impedance.logistic.inflection * m_mobile_base.vel_limits};

  m_carteisan_trj_sub = get_node()->create_subscription<moveit_msgs::msg::CartesianTrajectory>(
    m_parameters.cartesian_trajectory_topic, 1, [this](const moveit_msgs::msg::CartesianTrajectory& msg) {
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "got trajectory");
      if (msg.header.frame_id != m_parameters.frames.map) {
        RCLCPP_WARN_STREAM(get_node()->get_logger(), "Trajectory received but in wrong reference frame. Should be in {"
                                                       << m_parameters.frames.map << "} but instead is in {"
                                                       << msg.header.frame_id << "}. Skipping");
        return;
      }
      m_interpolator = utils::interpolation::Interpolator::from_msg(msg);
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "-> " << m_interpolator.is_empty());
    });

  m_rt_buffer_base_odom.initRT(nav_msgs::msg::Odometry(rosidl_runtime_cpp::MessageInitialization::ALL));

  Eigen::Matrix6d kfA, kfC, kfQ, kfR;
  Eigen::Matrix<double, 6, 3> kfB;
  kfA << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity() * m_dt, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();
  kfB << Eigen::Matrix3d::Identity() * 0.5 * std::pow(m_dt, 2), Eigen::Matrix3d::Identity() * m_dt;
  kfC.setIdentity();
  kfQ.setIdentity();
  kfR.setIdentity() * 1e1;
  m_base_position_filter = state_observer::KalmanFilter(kfA, kfB, kfC, kfQ, kfR);

  // Joint Kalman filter
  Eigen::MatrixXd kfjA(3 * m_nax, 3 * m_nax), kfjB(3 * m_nax, m_nax), kfjC(2 * m_nax, 3 * m_nax), kfjQ(3 * m_nax, 3 * m_nax),
    kfjR(2 * m_nax, 2 * m_nax);
  kfjA.setZero();
  kfjA.topLeftCorner(2 * m_nax, 2 * m_nax) << Eigen::MatrixXd::Identity(m_nax, m_nax),
    Eigen::MatrixXd::Identity(m_nax, m_nax) * m_dt, Eigen::MatrixXd::Zero(m_nax, m_nax), Eigen::MatrixXd::Identity(m_nax, m_nax);
  kfjA.bottomRightCorner(m_nax, m_nax).setIdentity();
  kfjB << Eigen::MatrixXd::Identity(m_nax, m_nax) * 0.5 * std::pow(m_dt, 2), Eigen::MatrixXd::Identity(m_nax, m_nax) * m_dt,
    Eigen::MatrixXd::Zero(m_nax, m_nax);
  kfjC.setZero();
  kfjC.leftCols(2 * m_nax).setIdentity();
  kfjC.bottomRightCorner(m_nax, m_nax).setIdentity();
  kfjQ.setIdentity();
  kfjQ.diagonal() << Eigen::Map<Eigen::VectorXd>(m_parameters.kalman_filter.manipulator.position.data(), m_nax),
    Eigen::Map<Eigen::VectorXd>(m_parameters.kalman_filter.manipulator.velocity.data(), m_nax),
    Eigen::VectorXd::Constant(m_nax, 1e-6);
  kfjR.setIdentity();
  m_joint_filter = state_observer::KalmanFilter(kfjA, kfjB, kfjC, kfjQ, kfjR);

  // Map->base transform handling
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
  m_tf_buffer->setUsingDedicatedThread(true);
  m_tf_buffer->setCreateTimerInterface(
    std::make_shared<tf2_ros::CreateTimerROS>(get_node()->get_node_base_interface(), get_node()->get_node_timers_interface()));
  m_tf_base_pose_recovery_thread = std::make_unique<std::thread>(&ElastoplasticController::update_base_pose_from_tf, this);
  bool can_transform{false};
  do {
    can_transform = m_tf_buffer->canTransform(m_parameters.frames.map, m_parameters.frames.base, tf2::TimePointZero);
  } while (!can_transform);
  m_T_world_base =
    tf2::transformToEigen(m_tf_buffer->lookupTransform(m_parameters.frames.map, m_parameters.frames.base, tf2::TimePointZero));
  if (m_parameters.debug.log) {
    m_node_semaph.acquire();
    m_node_support->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }

  for (int idx = 0; idx < 6; ++idx) {
    m_wrench_notch.push_back(std::make_shared<NotchFilter>(5, 1, get_update_rate()));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void ElastoplasticController::update_base_pose_from_tf() {
  m_node_support = rclcpp::Node::make_shared(
    "__support_node__", fmt::format("{}{}", this->get_node()->get_namespace(), this->get_node()->get_name()));
  m_node_semaph.release();
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer, m_node_support, false);
  m_support_node_exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  m_support_node_exec->add_node(m_node_support);
  m_support_node_exec->spin();
  m_support_node_exec->remove_node(m_node_support);
}

controller_interface::InterfaceConfiguration ElastoplasticController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interface_configuration.names.reserve(m_parameters.joints.size() * m_required_interface_types.size() + 6);

  for (const auto& jnt : m_parameters.joints) {
    state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION));
  }
  for (const auto& jnt : m_parameters.joints) {
    state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
  }

  // for (const auto& jnt : m_parameters.mobile_base.joints) {
  //   state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
  // }

  if (m_ft_source == FTSource::TORQUE) {
    for (const auto& jnt : m_parameters.joints) {
      state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_TORQUE));
    }
  } else if (m_ft_source == FTSource::FT_SENSOR) {
    std::vector<std::string> ft_interfaces = m_ft_sensor->get_state_interface_names();
    state_interface_configuration.names.insert(state_interface_configuration.names.end(), ft_interfaces.begin(),
                                               ft_interfaces.end());
  }

  return state_interface_configuration;
}


controller_interface::InterfaceConfiguration ElastoplasticController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interface_configuration;
  command_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interface_configuration.names.reserve(m_parameters.joints.size() * m_command_interfaces_names.size());
  if (std::ranges::find(m_command_interfaces_names, m_required_interface_types[0]) != m_command_interfaces_names.end()) {
    for (const auto& jnt : m_parameters.joints) {
      command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, m_required_interface_types[0]));
    }
  }
  if (std::ranges::find(m_command_interfaces_names, m_required_interface_types[1]) != m_command_interfaces_names.end()) {
    for (const auto& jnt : m_parameters.joints) {
      command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, m_required_interface_types[1]));
    }
  }

  if (m_mobile_base.enabled) {
    for (const auto& iface : m_parameters.mobile_base.command_interfaces) {
      // command_interface_configuration.names.emplace_back(fmt::format("{}/{}", iface, hardware_interface::HW_IF_VELOCITY));
      command_interface_configuration.names.emplace_back(iface);
    }
  }

  return command_interface_configuration;
}


controller_interface::CallbackReturn ElastoplasticController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  auto t_start = get_node()->get_clock()->now();
  while (!ready_for_activation() && !m_mobile_base_pose_updated &&
         get_node()->get_clock()->now() - t_start < std::chrono::seconds(10)) {
    RCLCPP_INFO(this->get_node()->get_logger(), "Waiting robot description-related operations");
    get_node()->get_clock()->sleep_for(std::chrono::milliseconds(1000));
  }
  if (m_robot_description_configuration != RDStatus::OK) {
    RCLCPP_ERROR(get_node()->get_logger(), "No robot description found");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (m_param_listener->is_old(m_parameters)) {
    m_parameters = m_param_listener->get_params();
    m_elastoplastic_model = std::make_unique<ElastoplasticModel>(utils::get_model_data(m_parameters, get_update_rate()));
    std_msgs::msg::String::SharedPtr rd = std::make_shared<std_msgs::msg::String>();
    rd->data = this->get_robot_description();
    configure_after_robot_description_callback(rd);
  }

  m_rt_pub_full_state =
    std::make_unique<realtime_tools::RealtimePublisher<elastoplastic_msgs::msg::ElastoplasticControllerState>>(m_pub_full_state);

  m_elastoplastic_model->clear();
  m_delta_elastoplastic_in_world.clear();

  m_joint_state_interfaces.resize(3);
  m_joint_command_interfaces.resize(3);

  for (const auto& interface : m_required_interface_types) {
    auto it = std::ranges::find(m_required_interface_types, interface);
    auto idx = std::distance(m_required_interface_types.begin(), it);
    if (not controller_interface::get_ordered_interfaces(state_interfaces_, m_parameters.joints, interface,
                                                         m_joint_state_interfaces.at(idx))) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing joints state interfaces: %ld names vs %ld interfaces",
                   m_parameters.joints.size(), m_joint_state_interfaces.at(idx).size());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  auto at_least_one_command_interface{false};
  for (const auto& interface : m_required_interface_types) {
    auto it = std::ranges::find(m_required_interface_types, interface);
    auto idx = std::distance(m_required_interface_types.begin(), it);
    if (not controller_interface::get_ordered_interfaces(command_interfaces_, m_parameters.joints, interface,
                                                         m_joint_command_interfaces.at(idx))) {
      continue;
    }
    at_least_one_command_interface = true;
  }
  if (!at_least_one_command_interface) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing at least one joints command interface");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (m_ft_source == FTSource::FT_SENSOR) {
    if (!m_ft_sensor->assign_loaned_state_interfaces(state_interfaces_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Cannot assing state interface to ft_sensor");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  if (m_mobile_base.enabled) {
    if (not controller_interface::get_ordered_interfaces(command_interfaces_, m_parameters.mobile_base.command_interfaces, "",
                                                         m_mobile_base_command_interfaces)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing base controller command interfaces");
      return controller_interface::CallbackReturn::FAILURE;
    }
    // if (!controller_interface::get_ordered_interfaces(state_interfaces_, m_parameters.mobile_base.joints,
    //                                                   hardware_interface::HW_IF_VELOCITY, m_mobile_base_state_interfaces)) {
    //   RCLCPP_ERROR(get_node()->get_logger(), "Missing mobile base joints");
    //   return controller_interface::CallbackReturn::FAILURE;
    // };
  }

  // Joint initialization
  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
  m_qpp.setZero();

  if (m_parameters.debug.log) {
    RCLCPP_WARN(get_node()->get_logger(), "Logger level: [DEBUG]");
  }

  m_last_odom_msg_time = this->get_node()->get_clock()->now();

  if (m_mobile_base.enabled) {
    m_q.head<2>() = m_T_world_base.translation().head<2>();
    m_q(2) = utils::vector_from_affine(m_T_world_base)(5);
    write_cmd_vel(m_mobile_base_command_interfaces, Eigen::Vector3d::Zero());
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
    Eigen::Vector6d offset_wrench;
    offset_wrench.setZero();
    const double offset_force_window = std::round(m_parameters.offset_force_window * get_update_rate());

    if (m_ft_source == FTSource::TORQUE) {
      // Wrench is already in world
      Eigen::Matrix6Xd J = m_chain_base_tool->getJacobian(m_q);
      Eigen::JacobiSVD<Eigen::Matrix6Xd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
      Eigen::VectorXd tau_j;
      for (int idx = 0; idx < offset_force_window; ++idx) {
        std::transform(m_joint_state_interfaces.at(2).begin(), m_joint_state_interfaces.at(2).end(), tau_j.head(m_nax).begin(),
                       [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
        Eigen::Vector6d wr = get_wrench_from_torque(svd, tau_j);
        if (wr.head<3>().norm() < m_parameters.wrench.deadband[0]) {
          wr.head<3>().setZero();
        } else {
          wr.head<3>().normalized() * (wr.head<3>().norm() - m_parameters.wrench.deadband[0]);
        }
        if (wr.tail<3>().norm() < m_parameters.wrench.deadband[1]) {
          wr.tail<3>().setZero();
        } else {
          wr.tail<3>().normalized() * (wr.tail<3>().norm() - m_parameters.wrench.deadband[1]);
        }
        std::transform(wr.begin(), wr.end(), offset_wrench.begin(), offset_wrench.begin(), std::plus<double>{});
        std::this_thread::sleep_for(rclcpp::Rate(get_update_rate()).period());
      }
      offset_wrench /= offset_force_window;
    } else if (m_ft_source == FTSource::FT_SENSOR) {
      // Wrench is in sensor frame
      Eigen::Vector6d offset_wrench_sensor_in_sensor;
      for (int idx = 0; idx < offset_force_window; ++idx) {
        Eigen::Vector6d wr = get_wrench_from_sensor();
        if (wr.head<3>().norm() < m_parameters.wrench.deadband[0]) {
          wr.head<3>().setZero();
        } else {
          wr.head<3>().normalized() * (wr.head<3>().norm() - m_parameters.wrench.deadband[0]);
        }
        if (wr.tail<3>().norm() < m_parameters.wrench.deadband[1]) {
          wr.tail<3>().setZero();
        } else {
          wr.tail<3>().normalized() * (wr.tail<3>().norm() - m_parameters.wrench.deadband[1]);
        }
        std::transform(wr.begin(), wr.end(), offset_wrench_sensor_in_sensor.begin(), offset_wrench_sensor_in_sensor.begin(),
                       std::plus<double>{});
        std::this_thread::sleep_for(rclcpp::Rate(get_update_rate()).period());
      }
      offset_wrench_sensor_in_sensor /= offset_force_window;

      // Transform wrench offset in world
      Eigen::Vector6d offset_wrench_tool_in_tool = rdyn::spatialDualTranformation(
        offset_wrench_sensor_in_sensor, m_chain_base_tool->getTransformation(m_q.tail(m_nax)).inverse() *
                                          m_chain_base_sensor->getTransformation(m_q.tail(m_nax)));

      offset_wrench = rdyn::spatialRotation(offset_wrench_tool_in_tool,
                                            m_chain_world_tool->getTransformation(m_q).linear()); // offset_wrench_tool_in_world
    }
    m_offset_wrench_tool_in_world = offset_wrench;
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Wrench Offset computed: " << m_offset_wrench_tool_in_world.transpose());
    return true;
  });

  if (m_ft_source == FTSource::FT_SENSOR) {
    Eigen::Affine3d T_base_tool = m_chain_base_tool->getTransformation(m_q.tail(m_nax));
    Eigen::Affine3d T_base_sensor = m_chain_base_sensor->getTransformation(m_q.tail(m_nax));
    m_T_tool_sensor = T_base_tool.inverse() * T_base_sensor;
  }

  m_base_position_filter.initialize((Eigen::Vector6d() << m_q.head<3>(), m_qp.head<3>()).finished());
  m_joint_filter.initialize(
    (Eigen::VectorXd(3 * m_nax) << m_q.tail(m_nax), m_qp.tail(m_nax), Eigen::VectorXd::Zero(m_nax)).finished());

  RCLCPP_DEBUG(m_node_support->get_logger(), "Activated...");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn ElastoplasticController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
  m_qpp.setZero();

  m_computed_target_T_world_tool = m_chain_world_tool->getTransformation(m_q);
  m_computed_target_acc_tool_world_in_world.setZero();
  m_computed_target_twist_tool_world_in_world.setZero();

  if (m_mobile_base.enabled) {
    Eigen::Vector6d empty = Eigen::Vector6d::Zero();
    geometry_msgs::msg::Twist cmd_vel = tf2::toMsg(empty);
    m_pub_cmd_vel->publish(cmd_vel);
    write_cmd_vel(m_mobile_base_command_interfaces, Eigen::Vector3d::Zero());
  }

  m_rt_pub_full_state->stop();

  m_elastoplastic_model->clear();
  m_delta_elastoplastic_in_world.clear();

  m_joint_state_interfaces.clear();
  m_joint_command_interfaces.clear();
  m_mobile_base_command_interfaces.clear();
  // m_mobile_base_state_interfaces.clear();

  if (m_ft_source == FTSource::FT_SENSOR) {
    m_ft_sensor->release_interfaces();
  }

  m_wrench_in_sensor_prec.setZero();

  m_support_node_exec->cancel();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElastoplasticController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (m_tf_base_pose_recovery_thread->joinable())
    m_tf_base_pose_recovery_thread->join();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElastoplasticController::on_error(const rclcpp_lifecycle::State& previous_state) {
  return ElastoplasticController::on_deactivate(previous_state);
}


std::vector<hardware_interface::CommandInterface> ElastoplasticController::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  m_joint_reference_interfaces_size =
    m_parameters.joints.size() *
    m_required_interface_types.size(); // There must be both position and velocity reference interfaces!

  reference_interfaces_.resize(m_joint_reference_interfaces_size);
  reference_interfaces.reserve(m_joint_reference_interfaces_size);

  size_t idx = 0;
  for (const auto& hwi : m_required_interface_types) {
    for (const auto& jnt : m_parameters.joints) {

      reference_interfaces.emplace_back(hardware_interface::CommandInterface(
        std::string(get_node()->get_name()), fmt::format("{}/{}", jnt, hwi), &reference_interfaces_[idx]));
      ++idx;
    }
  }

  return reference_interfaces;
}


controller_interface::return_type ElastoplasticController::update_reference_from_subscribers(const rclcpp::Time& /*time*/,
                                                                                             const rclcpp::Duration& /*period*/) {
  /* "Joint trajectory available only in chainable mode with joint_trajectory_controller" */

  std::copy(m_q.tail(m_nax).begin(), m_q.tail(m_nax).end(), reference_interfaces_.begin());     // position
  std::fill(std::next(reference_interfaces_.begin(), m_nax), reference_interfaces_.end(), 0.0); // velocity

  return controller_interface::return_type::OK;
}

void ElastoplasticController::get_odometry_callback(const nav_msgs::msg::Odometry& msg) {
  m_rt_buffer_base_odom.writeFromNonRT(msg);
}


controller_interface::return_type ElastoplasticController::update_and_write_commands(const rclcpp::Time& time,
                                                                                     const rclcpp::Duration& period) {
  rclcpp::Time t_start = get_node()->get_clock()->now();

  if (m_offset_future.wait_for(0s) != std::future_status::ready) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "[Waiting] Computing Offset Force");
    bool result{true};
    for (size_t idx = 0; idx < m_nax; ++idx) {
      result &= m_joint_command_interfaces.at(0).at(idx).get().set_value(
        m_joint_state_interfaces.at(0).at(idx).get().get_optional().value());
    }
    if (!result) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not copy state interface position into command interfaces");
      return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
  }


  // **********
  // ** Read **
  // **********

#define ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MOBILE_BASE_
#ifdef ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MOBILE_BASE
  // Base state
  bool got_new_odom = false;
  if (m_mobile_base.enabled) {
    Eigen::Vector6d twist_base_world_in_world, twist_base_world_in_base;

    try {
      geometry_msgs::msg::TransformStamped T_world_base_msg =
        m_tf_buffer->lookupTransform(m_parameters.frames.map, m_parameters.frames.base, tf2::TimePointZero);
      m_T_world_base = tf2::transformToEigen(T_world_base_msg);
    } catch (tf2::LookupException& ex) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Could not get transformation between " << m_parameters.frames.map << " and "
                                                                                            << m_parameters.frames.base
                                                                                            << ". Fallback on computed data");
    } catch (std::exception&) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Error while getting " << m_parameters.frames.map << " and "
                                                                           << m_parameters.frames.base
                                                                           << ". Fallback on computed data");
    }

    // NOTE: Support only inter-process comms
    nav_msgs::msg::Odometry odom_msg;
    [[maybe_unused]] rclcpp::MessageInfo msg_info;
    if (m_sub_mobile_base_odometry->take(odom_msg, msg_info)) {
      Eigen::fromMsg(odom_msg.twist.twist, twist_base_world_in_base);
      got_new_odom = true;
    } else {
      twist_base_world_in_base = utils::twist_from_base_velocity(m_velocity_base_in_base);
    }

    // // Recover twist from odometry
    // nav_msgs::msg::Odometry odom_msg = *(m_rt_buffer_base_odom.readFromRT());
    // if (rclcpp::Time(odom_msg.header.stamp) - m_last_odom_msg_time > std::chrono::duration<double>(m_dt) ||
    // rclcpp::Time(odom_msg.header.stamp) - m_last_odom_msg_time < std::chrono::seconds(0)) {
    // twist_base_world_in_base = utils::twist_from_base_velocity(m_velocity_base_in_base);
    // } else {
    // Eigen::fromMsg(odom_msg.twist.twist, twist_base_world_in_base);
    // }
    // m_last_odom_msg_time = odom_msg.header.stamp;


    // Eigen::Vector4d wheel_vel;
    // std::transform(m_mobile_base_state_interfaces.begin(), m_mobile_base_state_interfaces.end(), wheel_vel.begin(),
    // [](const hardware_interface::LoanedStateInterface& lsi) -> double { return lsi.get_optional().value(); });
    // twist_base_world_in_base = utils::mecanum_direct_kinematics(wheel_vel, m_parameters.mobile_base.wheel_radius,
    // m_parameters.mobile_base.sum_of_lx_and_ly);

    twist_base_world_in_world = rdyn::spatialRotation(twist_base_world_in_base, m_T_world_base.linear());

    // Build state vectors
    Eigen::Vector6d estim_base = m_base_position_filter.predict(m_qpp.head<M_SE2>());
    if (got_new_odom) {
      Eigen::Vector6d base_read;
      base_read.tail<M_SE2>() = utils::base_velocity_from_twist(twist_base_world_in_world);
      base_read.head<2>() = m_T_world_base.translation().head<2>();
      base_read(2) = Eigen::AngleAxisd(m_T_world_base.linear()).angle();
      m_base_position_filter.update(base_read);
      estim_base = m_base_position_filter.get_state();
    }
    // Eigen::Vector6d estim_base = m_base_position_filter.update(base_read, Eigen::Vector3d::Zero());
    m_qp.head<M_SE2>() = estim_base.tail<M_SE2>();
    m_q.head<M_SE2>() = estim_base.head<M_SE2>();
    // m_qp.head<M_SE2>() = utils::base_velocity_from_twist(twist_base_world_in_world);
    // m_q.head<2>() = m_T_world_base.translation().head<2>();
    // m_q(2) = Eigen::AngleAxisd(m_T_world_base.linear()).angle();
  }
#else
  bool got_new_odom = true;
#endif

#define ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MANIPULATOR_
#ifdef ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MANIPULATOR
  // Manipulator State
  Eigen::VectorXd q_qp_in(2 * m_nax), q_qp_out(2 * m_nax);
  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), q_qp_in.head(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), q_qp_in.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
  // Kalman filter
#define ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MANIPULATOR__USE_KALMAN_
#ifdef ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MANIPULATOR__USE_KALMAN
  q_qp_out = m_joint_filter.update(q_qp_in, m_qpp.tail(m_nax));
  m_q.tail(m_nax) = q_qp_out.head(m_nax);
  m_qp.tail(m_nax) = q_qp_out.tail(m_nax);
#else
  m_q.tail(m_nax) = q_qp_in.head(m_nax);
  m_qp.tail(m_nax) = q_qp_in.tail(m_nax);
#endif

#endif

  Eigen::Affine3d T_world_tool = m_chain_world_tool->getTransformation(m_q);
  Eigen::Vector6d twist_tool_world_in_world = m_chain_world_tool->getTwistTool(m_q, m_qp);
  Eigen::VectorXd full_position_references(m_full_nax), full_velocity_references(m_full_nax);

#define USE_CARTESIAN_REFERENCE
#ifndef USE_CARTESIAN_REFERENCE
  /* Joint Reference */
  // Target base
  Eigen::Vector6d target_twist_base_world_in_world;
  Eigen::Vector3d mobile_base_pose_in_world;
  if (m_mobile_base.enabled) {
    Eigen::Vector6d target_twist_base_world_in_base;
    Eigen::fromMsg(*(m_rt_buffer_mobile_base_target.readFromRT()), target_twist_base_world_in_base);
    target_twist_base_world_in_world = move_from_base_to_world(target_twist_base_world_in_base);

    Eigen::Affine3d T_target_world_base;
    T_target_world_base = rdyn::spatialIntegration(m_T_world_base, target_twist_base_world_in_world, m_dt);
    mobile_base_pose_in_world << T_target_world_base.translation().head<2>(),
      Eigen::AngleAxisd(T_target_world_base.linear()).angle();
  } else {
    mobile_base_pose_in_world.setZero();
    target_twist_base_world_in_world.setZero();
  }

  // Target manipulator
  Eigen::VectorXd joint_position_references(m_nax);
  joint_position_references = Eigen::Map<Eigen::VectorXd>(reference_interfaces_.data(), m_parameters.joints.size());
  Eigen::VectorXd joint_velocity_references(m_nax);
  joint_velocity_references = Eigen::Map<Eigen::VectorXd>(std::next(reference_interfaces_.data(), m_nax), m_nax);
  if (m_mobile_base.enabled) {
    full_position_references.head(m_mobile_base.nax()) << mobile_base_pose_in_world;
    full_velocity_references.head(m_mobile_base.nax()) << utils::base_velocity_from_twist(target_twist_base_world_in_world);
  }
  full_position_references.tail(m_nax) << joint_position_references;
  full_velocity_references.tail(m_nax) << joint_velocity_references;

  Eigen::Vector6d target_twist_tool_world_in_world =
    m_chain_world_tool->getJacobian(full_position_references) * full_velocity_references;

#else

  full_position_references.tail(m_nax) = m_initial_q.tail(m_nax);
  full_velocity_references.setZero();

  /* Cartesian reference */
  if (!m_interpolator.is_plan_started() && m_interpolator.is_ready()) {
    RCLCPP_INFO(get_node()->get_logger(), "Start interpolation of cartesian plan");
    m_interpolator.start_plan(get_node()->get_clock()->now());
  }
  // TODO: end plan

  Eigen::Vector6d reference_target_acc_tool_world_in_world;
  Eigen::Vector6d reference_target_twist_tool_world_in_world;
  Eigen::Affine3d reference_target_T_world_tool;
  if (m_interpolator.is_plan_started()) {
    auto status = m_interpolator.interpolate(get_node()->get_clock()->now(), reference_target_acc_tool_world_in_world,
                                             reference_target_twist_tool_world_in_world, reference_target_T_world_tool);
    if (status != utils::interpolation::Interpolator::InterpolationResult::OK) {
      reference_target_acc_tool_world_in_world.setZero();
      reference_target_twist_tool_world_in_world.setZero();
      reference_target_T_world_tool = m_chain_world_tool->getTransformation(m_initial_q);
    }
  } else {
    reference_target_acc_tool_world_in_world.setZero();
    reference_target_twist_tool_world_in_world.setZero();
    reference_target_T_world_tool = m_chain_world_tool->getTransformation(m_initial_q);
  }
  if (m_mobile_base.enabled) {
    full_velocity_references.head<M_SE2>() = utils::base_velocity_from_twist(reference_target_twist_tool_world_in_world);
    full_position_references.head<M_SE2>() = utils::base_velocity_from_twist(utils::vector_from_affine(
      reference_target_T_world_tool * m_chain_base_tool->getTransformation(m_initial_q.tail(m_nax)).inverse()));
  }

#endif

  /* FT state */
  Eigen::Vector6d wrench_tool_in_world;
  Eigen::Matrix6Xd J_world_tool_in_world = m_chain_world_tool->getJacobian(m_q);
  // Damped LS
  if (m_ft_source == FTSource::TORQUE) {
    Eigen::VectorXd tau_j(m_nax);
    Eigen::JacobiSVD<Eigen::Matrix6Xd> svd_torque(J_world_tool_in_world.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::transform(m_joint_state_interfaces.at(2).begin(), m_joint_state_interfaces.at(2).end(), tau_j.head(m_nax).begin(),
                   [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
    wrench_tool_in_world = get_wrench_from_torque(svd_torque, tau_j);
    if (wrench_tool_in_world.head<3>().norm() < m_parameters.wrench.deadband[0]) {
      wrench_tool_in_world.head<3>().setZero();
    } else {
      wrench_tool_in_world.head<3>().normalized() * (wrench_tool_in_world.head<3>().norm() - m_parameters.wrench.deadband[0]);
    }
    if (wrench_tool_in_world.tail<3>().norm() < m_parameters.wrench.deadband[1]) {
      wrench_tool_in_world.tail<3>().setZero();
    } else {
      wrench_tool_in_world.tail<3>().normalized() * (wrench_tool_in_world.tail<3>().norm() - m_parameters.wrench.deadband[1]);
    }
    std::transform(wrench_tool_in_world.begin(), wrench_tool_in_world.end(), m_wrench_in_sensor_prec.begin(),
                   wrench_tool_in_world.begin(), [this](const double w, const double w_prec) {
                     return filters::exponentialSmoothing(w, w_prec, m_parameters.wrench.filter_alfa);
                   });
    m_wrench_in_sensor_prec = wrench_tool_in_world;
  } else {
    Eigen::Vector6d wrench_sensor_in_sensor;
    wrench_sensor_in_sensor = get_wrench_from_sensor();

    if (wrench_sensor_in_sensor.hasNaN()) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *this->get_node()->get_clock(), 1000,
                           "Force sensor contains NaN values. Full measure discarded and replaced with zero");
      wrench_sensor_in_sensor.setZero();
    } else if (wrench_sensor_in_sensor.cwiseAbs().maxCoeff() > 1e20) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *this->get_node()->get_clock(), 1000,
                           "Force sensor contains overflowed values. Full measure discarded and replaced with zero");
      wrench_sensor_in_sensor.setZero();
    }

    // Wrench deadband
    if (wrench_sensor_in_sensor.head<3>().norm() < m_parameters.wrench.deadband[0]) {
      wrench_sensor_in_sensor.head<3>().setZero();
    } else {
      wrench_sensor_in_sensor.head<3>().normalized() *
        (wrench_sensor_in_sensor.head<3>().norm() - m_parameters.wrench.deadband[0]);
    }
    if (wrench_sensor_in_sensor.tail<3>().norm() < m_parameters.wrench.deadband[1]) {
      wrench_sensor_in_sensor.tail<3>().setZero();
    } else {
      wrench_sensor_in_sensor.tail<3>().normalized() *
        (wrench_sensor_in_sensor.tail<3>().norm() - m_parameters.wrench.deadband[1]);
    }

    // Exponential filter
    std::transform(wrench_sensor_in_sensor.begin(), wrench_sensor_in_sensor.end(), m_wrench_in_sensor_prec.begin(),
                   wrench_sensor_in_sensor.begin(), [this](const double w, const double w_prec) {
                     return filters::exponentialSmoothing(w, w_prec, m_parameters.wrench.filter_alfa);
                   });
    m_wrench_in_sensor_prec = wrench_sensor_in_sensor;

    Eigen::Vector6d wrench_tool_in_tool = rdyn::spatialDualTranformation(wrench_sensor_in_sensor, m_T_tool_sensor);
    wrench_tool_in_world = rdyn::spatialRotation(wrench_tool_in_tool, T_world_tool.linear()) - m_offset_wrench_tool_in_world;
  }

  // std::transform(wrench_tool_in_world.begin(), wrench_tool_in_world.end(), m_wrench_notch.begin(),
  // wrench_tool_in_world.begin(),
  // [](const double w, const std::shared_ptr<NotchFilter>& notch) { return notch->update(w); });

  Eigen::VectorXd q_start = m_q;
  Eigen::VectorXd qp_start = m_qp;

  // Update computed trajectory
  m_computed_target_twist_tool_world_in_world =
    m_computed_target_twist_tool_world_in_world + m_computed_target_acc_tool_world_in_world * m_dt;

  m_computed_target_T_world_tool =
    rdyn::spatialIntegration(m_computed_target_T_world_tool, m_computed_target_twist_tool_world_in_world, m_dt);

  // ************
  // ** Update **
  // ************

  Eigen::Vector6d cart_vel_error_tool_target_in_world;
  // cart_vel_error_tool_target_in_world = (twist_tool_world_in_world - reference_target_twist_tool_world_in_world)
  cart_vel_error_tool_target_in_world = (twist_tool_world_in_world - m_computed_target_twist_tool_world_in_world)
                                          .cwiseProduct(m_elastoplastic_model->get_enabled_axis());

  // Eigen::Vector6d d_pose;
  // rdyn::getFrameDistanceQuat(m_computed_target_T_world_tool, reference_target_T_world_tool, d_pose);
  // d_pose.normalize();
  m_zp = m_elastoplastic_model->update_z(cart_vel_error_tool_target_in_world, m_dt);
  bool reset = m_elastoplastic_model->reset(wrench_tool_in_world.cwiseProduct(m_elastoplastic_model->get_enabled_axis()),
                                            cart_vel_error_tool_target_in_world);
  if (reset) {
    RCLCPP_WARN_STREAM(get_node()->get_logger(), "Reset to Elastic Mode");
  }
  m_computed_target_T_world_tool = reset ? T_world_tool : m_computed_target_T_world_tool; // NOTE: useful?

  ClikData clik_data{.position_references = full_position_references,
                     .velocity_references = full_velocity_references,
                     .twist_tool_world_in_world = twist_tool_world_in_world,
                     .T_world_tool = T_world_tool,
                     .target_acc_tool_target_in_world = reference_target_acc_tool_world_in_world,
                     .J_world_tool_in_world = J_world_tool_in_world,
                     .target_T_world_tool = reference_target_T_world_tool,
                     .target_twist_tool_world_in_world = reference_target_twist_tool_world_in_world,
                     .wrench_tool_in_world = wrench_tool_in_world,
                     .got_new_odom = got_new_odom};

  std::optional<Eigen::VectorXd> solution_qp = clik(clik_data);
  if (!solution_qp.has_value()) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Cannot find a solution for the CLIK QP problem. Keeping actual position");
    // RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "\ncart_vel_error_tool_target_in_world\n"
    // << cart_vel_error_tool_target_in_world.transpose()
    // << "\nwrench_tool_in_world\n"
    // << wrench_tool_in_world.transpose() << "\nwrench_tool_in_tool\n");
    // this->on_deactivate(rclcpp_lifecycle::State());
    // throw std::runtime_error("Controller crashed");
    // TODO: trova soluzione più intelligente
    m_qp.setZero();
    m_qpp.setZero();
    m_admittance_value.setZero();
  } else {
    Eigen::VectorXd qepp = solution_qp.value().head(m_full_nax);
    // Eigen::Vector6d xepp = solution_qp.value().tail<M_SE3>();
    m_qpp = qepp;
    std::tie(m_q, m_qp) = utils::rk4_double([](const auto&, const auto&, const auto& u) { return u; }, m_q, m_qp, qepp, m_dt);
  }


  Eigen::Vector6d dist;
  rdyn::getFrameDistanceQuat(T_world_tool, reference_target_T_world_tool, dist);
  if (m_elastoplastic_model->to_restore() && !m_elastoplastic_model->is_plastic() && dist.head<3>().norm() < 1e-3 &&
      dist.tail<3>().norm() < 1.0 && m_parameters.impedance.plastic_restoration) {
    m_elastoplastic_model->restore();
    RCLCPP_INFO(get_node()->get_logger(), "Restore elastic state");
  }

  if (m_mobile_base.enabled) {
    Eigen::Vector6d qp_base_in_world = Eigen::Vector6d::Zero();
    qp_base_in_world = utils::twist_from_base_velocity(m_qp.head<M_SE2>());

    Eigen::Vector6d qp_base_in_base = rdyn::spatialRotation(qp_base_in_world, m_T_world_base.linear().transpose());
    // qp_base_in_base = qp_base_in_base.unaryExpr([this](double vel) { return std::abs(vel) < M_VELOCITY_TOLLERANCE ? 0.0 :
    // vel;
    // });
    m_velocity_base_in_base = utils::base_velocity_from_twist(qp_base_in_base);

    // BEGIN - Check Saturation Base
    // If the QP works, this shouldn't be necessary
    for (size_t idx = 0; idx < M_SE2; ++idx) {
      if (std::abs(m_velocity_base_in_base(idx)) > m_mobile_base.vel_limits(idx)) {
        RCLCPP_WARN_STREAM(this->get_node()->get_logger(),
                           "Saturation of Velocity on base linear direction "
                             << idx << ": " << m_velocity_base_in_base(idx) << " should be "
                             << utils::sgn(m_velocity_base_in_base(idx)) * m_mobile_base.vel_limits(idx));
        m_velocity_base_in_base(idx) = utils::sgn(m_velocity_base_in_base(idx)) * m_mobile_base.vel_limits(idx);
      }
    }
    m_qp.head<M_SE2>() = utils::base_velocity_from_twist(
      rdyn::spatialRotation(utils::twist_from_base_velocity(m_velocity_base_in_base), m_T_world_base.linear()));
    // END - Check Saturation Base
  }

  // BEGIN - Saturation Manipulator
  for (size_t idx = 0; idx < m_nax; ++idx) {
    double q = m_q(idx + (m_full_nax - m_nax));
    double dq = m_qp(idx + (m_full_nax - m_nax));
    m_q(idx + (m_full_nax - m_nax)) =
      std::max(m_limits.pos_lower(idx), std::min(m_limits.pos_upper(idx), m_q(idx + (m_full_nax - m_nax))));
    m_qp(idx + (m_full_nax - m_nax)) =
      std::max(-m_limits.vel(idx), std::min(m_limits.vel(idx), m_qp(idx + (m_full_nax - m_nax))));
    if (!utils::almost_equal(q, m_q(idx + (m_full_nax - m_nax)))) {
      // RCLCPP_WARN(get_node()->get_logger(), "Saturation of POSITION on manipulator joint with index %ld", idx);
    }
    if (!utils::almost_equal(dq, m_qp(idx + (m_full_nax - m_nax)))) {
      // RCLCPP_WARN(get_node()->get_logger(), "Saturation of VELOCITY on manipulator joint with index %ld", idx);
    }
  }
  // END - Saturation Manipulator

  // ***********
  // ** Write **
  // ***********
  bool is_value_set{true};
  if (m_used_command_interfaces.at(0)) {
    for (size_t ax = 0; ax < m_nax; ++ax) {
      is_value_set &= m_joint_command_interfaces.at(0).at(ax).get().set_value(m_q(ax + (m_full_nax - m_nax)));
    }
  }
  if (m_used_command_interfaces.at(1)) {
    for (size_t ax = 0; ax < m_nax; ++ax) {
      is_value_set &= m_joint_command_interfaces.at(1).at(ax).get().set_value(m_qp(ax + (m_full_nax - m_nax)));
    }
  }
  if (!is_value_set) {
    RCLCPP_FATAL_STREAM(get_node()->get_logger(), "Could not write on the hardware interface! Halting!");
    this->on_deactivate(rclcpp_lifecycle::State());
    throw std::runtime_error("Controller crashed");
  }

  if (m_mobile_base.enabled) {
    Eigen::Vector6d base_twist_in_base = utils::twist_from_base_velocity(m_velocity_base_in_base);

    geometry_msgs::msg::Twist cmd_vel = Eigen::toMsg(base_twist_in_base);
    m_pub_cmd_vel->publish(cmd_vel);

    bool is_mobile_base_write_ok = write_cmd_vel(m_mobile_base_command_interfaces, m_velocity_base_in_base);
    if (!is_mobile_base_write_ok) {
      write_cmd_vel(m_mobile_base_command_interfaces, Eigen::Vector3d::Zero());
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "Problem occurred while writing on mobile base interfaces! Stopping the movement");
    }

    m_T_world_base = rdyn::spatialIntegration(m_T_world_base, base_twist_in_base, m_dt);
  }

  m_q_prec = m_q;
  m_qp_prec = m_qp;
  m_qpp_prec = m_qpp;

  // *************
  // ** PUBLISH **
  // *************
  auto time_now = this->get_node()->get_clock()->now();
  elastoplastic_msgs::msg::ElastoplasticControllerState msg;

  msg.header.stamp = time_now;
  msg.header.frame_id = m_parameters.frames.map;

  msg.z.reserve(6);
  msg.zp.reserve(6);
  Eigen::Vector6d msg_z = m_elastoplastic_model->z();
  std::copy(msg_z.begin(), msg_z.end(), std::back_inserter(msg.z));
  std::copy(m_zp.begin(), m_zp.end(), std::back_inserter(msg.zp));

  msg.cart_ref_pose = tf2::toMsg(reference_target_T_world_tool);
  msg.cart_ref_twist = tf2::toMsg(reference_target_twist_tool_world_in_world);

  msg.cart_actual_cmd_pose = tf2::toMsg(m_chain_world_tool->getTransformation(m_q));
  msg.cart_actual_cmd_twist = tf2::toMsg(m_chain_world_tool->getTwistTool(m_q, m_qp));
  msg.cart_actual_cmd_acc = tf2::toMsg(m_chain_world_tool->getDTwistTool(m_q, m_qp, m_qpp));

  double tmp;
  std::tie(msg.reset_buffer_state, tmp) = m_elastoplastic_model->get_reset_buffer_status();

  // Admittance state msg
  geometry_msgs::msg::WrenchStamped msg_wrench_in_world = toWrenchStampedMsg(wrench_tool_in_world);
  msg_wrench_in_world.header.frame_id = m_parameters.frames.map;
  msg_wrench_in_world.header.stamp = time_now;

  geometry_msgs::msg::TransformStamped fk_msg;
  Eigen::Affine3d fk = m_computed_target_T_world_tool;
  fk_msg = tf2::eigenToTransform(fk);
  fk_msg.header.stamp = time_now;
  fk_msg.header.frame_id = m_parameters.frames.map;
  fk_msg.child_frame_id = m_parameters.frames.tool;

  geometry_msgs::msg::TwistStamped fk_vel;
  fk_vel.twist = tf2::toMsg(m_computed_target_twist_tool_world_in_world);
  fk_vel.header.stamp = time_now;
  fk_vel.header.frame_id = m_parameters.frames.map;

  geometry_msgs::msg::TwistStamped fk_acc;
  fk_acc.twist = tf2::toMsg(m_computed_target_acc_tool_world_in_world);
  fk_acc.header.stamp = time_now;
  fk_acc.header.frame_id = m_parameters.frames.map;

  geometry_msgs::msg::Pose cmp_target_T_msg;
  cmp_target_T_msg = tf2::toMsg(m_computed_target_T_world_tool);

  sensor_msgs::msg::JointState jnt_state;
  jnt_state.header.stamp = time_now;
  jnt_state.name.reserve(m_joint_names.size());
  jnt_state.position.reserve(m_joint_names.size());
  jnt_state.velocity.reserve(m_joint_names.size());
  jnt_state.effort.reserve(m_joint_names.size());
  std::copy(m_joint_names.begin(), m_joint_names.end(), std::back_inserter(jnt_state.name));
  std::copy(m_q.begin(), m_q.end(), std::back_inserter(jnt_state.position));
  std::copy(m_qp.begin(), m_qp.end(), std::back_inserter(jnt_state.velocity));
  std::copy(m_qpp.begin(), m_qpp.end(), std::back_inserter(jnt_state.effort));

  msg.admittance_state.admittance_position = fk_msg;
  msg.admittance_state.admittance_velocity = fk_vel;
  msg.admittance_state.admittance_acceleration = fk_acc;
  msg.admittance_state.joint_state = jnt_state;
  msg.admittance_state.wrench_base = msg_wrench_in_world;

  msg.admittance_state.selected_axes.data.reserve(6);
  std::copy(m_elastoplastic_model->get_enabled_axis().begin(), m_elastoplastic_model->get_enabled_axis().end(),
            std::back_inserter(msg.admittance_state.selected_axes.data));
  msg.admittance_state.ft_sensor_frame.data = m_parameters.frames.sensor;
  msg.admittance_state.ref_trans_base_ft = tf2::eigenToTransform(m_chain_base_sensor->getTransformation(m_q.tail(m_nax)));
  msg.admittance_state.rot_base_control = tf2::toMsg(Eigen::Quaterniond(m_T_tool_sensor.linear()));

  msg.admittance_state.stiffness.data.reserve(6);
  msg.admittance_state.damping.data.reserve(6);
  auto [K, D] = m_elastoplastic_model->compute_variable_matrices(T_world_tool);
  Eigen::Vector6d K_diag = K.diagonal();
  std::copy(K_diag.begin(), K_diag.end(), std::back_inserter(msg.admittance_state.stiffness.data));
  std::copy(D.diagonal().begin(), D.diagonal().end(), std::back_inserter(msg.admittance_state.damping.data));

  toWrenchMsg(m_admittance_value, msg.virtual_force);

  if (m_elastoplastic_model->is_plastic()) {
    msg.mode = elastoplastic_msgs::msg::ElastoplasticControllerState::MODE_PLASTIC;
  } else if (m_elastoplastic_model->to_restore()) {
    msg.mode = elastoplastic_msgs::msg::ElastoplasticControllerState::MODE_RESTORE;
  } else {
    msg.mode = elastoplastic_msgs::msg::ElastoplasticControllerState::MODE_ELASTIC;
  }

  if (m_rt_pub_full_state->trylock()) {
    m_rt_pub_full_state->msg_ = msg;
    m_rt_pub_full_state->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

} // namespace elastoplastic

PLUGINLIB_EXPORT_CLASS(elastoplastic::ElastoplasticController, controller_interface::ChainableControllerInterface);
