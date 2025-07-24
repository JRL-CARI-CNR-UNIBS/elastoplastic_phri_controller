#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"
#include "elastoplastic_lugre_controller/sot.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"

#include "control_toolbox/filters.hpp"
#include "eiquadprog/eiquadprog-fast.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "urdfdom_headers/urdf_model/model.h"

#include <algorithm>
#include <chrono>

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
  }
  m_pub_cmd_vel = this->get_node()->create_publisher<geometry_msgs::msg::Twist>(m_parameters.cmd_vel_topic, 1);
  m_pub_timing = this->get_node()->create_publisher<std_msgs::msg::Float64>("~/controller_period", 1);

  if (m_parameters.wrench.source == "ft_sensor") {
    m_ft_source = FTSource::FT_SENSOR;
  } else {
    m_ft_source = FTSource::TORQUE;
    m_invert_torque = m_parameters.wrench.invert_torque ? -1 : 1;
  }

  if (m_ft_source == FTSource::FT_SENSOR) {
    m_ft_sensor = std::make_unique<semantic_components::ForceTorqueSensor>(m_parameters.ft_sensor_name);
  }

  m_pub_controller_mode = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/mode", 5);
  if (m_parameters.debug.pub) {
    m_cmd_pose_pub = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/cmd_q", 5);
    m_pub_z = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/z", 10);
    m_pub_wrench_in_world = this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("~/wrench_in_world", 10);
    m_pub_admittance_force = this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("~/admittance_force", 10);
    m_pub_cart_vel_error = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/cart_vel_error", 10);
    m_pub_twist_in_world = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/twist_in_world", 10);
    m_pub_joint_reference = this->get_node()->create_publisher<sensor_msgs::msg::JointState>("~/joint_references", 10);
    m_pub_fk_world_tool = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/fk_world_tool", rclcpp::QoS(1));
    m_pub_fk_base_tool = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/fk_base_tool", rclcpp::QoS(1));
    m_pub_weights = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/weights", 10);
    m_interp_pose_pub = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/interp_pose", 5);
    m_interp_twist_pub = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/interp_twist", 10);
    m_computed_pose_pub = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/computed_pose", 5);
    m_computed_twist_pub = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/computed_twist", 10);
    m_estim_joint_state = this->get_node()->create_publisher<sensor_msgs::msg::JointState>("~/estimated_joints", 10);
    m_pub_reset_buffer = this->get_node()->create_publisher<std_msgs::msg::Float64>("~/reset_buffer_status", 10);
  }

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

  // Joint initialization
  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
  m_qpp.setZero();


  if (m_mobile_base.enabled) {
    m_pub_cmd_vel->on_activate();
  }

  if (m_parameters.debug.log) {
    RCLCPP_WARN(get_node()->get_logger(), "Logger level: [DEBUG]");
  }

  if (m_parameters.debug.pub) {
    RCLCPP_WARN(get_node()->get_logger(), "Debug-related publishers: ON");
    m_cmd_pose_pub->on_activate();
    m_pub_wrench_in_world->on_activate();
    m_pub_admittance_force->on_activate();
    m_pub_z->on_activate();
    m_pub_cart_vel_error->on_activate();
    m_pub_twist_in_world->on_activate();
    m_pub_joint_reference->on_activate();
    m_pub_fk_world_tool->on_activate();
    m_pub_fk_base_tool->on_activate();
    m_pub_weights->on_activate();
    m_interp_pose_pub->on_activate();
    m_interp_twist_pub->on_activate();
    m_computed_pose_pub->on_activate();
    m_computed_twist_pub->on_activate();
    m_estim_joint_state->on_activate();
    m_pub_reset_buffer->on_activate();
  }

  m_last_odom_msg_time = this->get_node()->get_clock()->now();

  if (m_mobile_base.enabled) {
    m_q.head<2>() = m_T_world_base.translation().head<2>();
    m_q(2) = utils::vector_from_affine(m_T_world_base)(5);
  }

  m_mobile_base.velocity_in_base.setZero();

  m_initial_q = m_q;

  // For debug purposes
  m_q_prec.setZero();
  m_qp_prec.setZero();
  m_qpp_prec.setZero();

  m_wrench_in_sensor_prec.setZero();

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
      Eigen::Vector6d offset_wrench_tool_in_tool =
        rdyn::spatialDualTranformation(offset_wrench_sensor_in_sensor, m_chain_base_tool->getTransformation(m_q).inverse() *
                                                                         m_chain_base_sensor->getTransformation(m_q));

      offset_wrench = rdyn::spatialRotation(offset_wrench_tool_in_tool,
                                            m_chain_world_tool->getTransformation(m_q).linear()); // offset_wrench_tool_in_world
    }
    m_offset_wrench_tool_in_world = offset_wrench;
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Wrench Offset computed: " << m_offset_wrench_tool_in_world.transpose());
    return true;
  });

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
  }

  m_elastoplastic_model->clear();
  m_delta_elastoplastic_in_world.clear();

  m_joint_state_interfaces.clear();
  m_joint_command_interfaces.clear();

  if (m_ft_source == FTSource::FT_SENSOR) {
    m_ft_sensor->release_interfaces();
  }

  m_wrench_in_sensor_prec.setZero();

  m_support_node_exec->cancel();
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

    // Recover twist from odometry
    nav_msgs::msg::Odometry odom_msg = *(m_rt_buffer_base_odom.readFromRT());
    if (rclcpp::Time(odom_msg.header.stamp) - m_last_odom_msg_time > std::chrono::duration<double>(m_dt) ||
        rclcpp::Time(odom_msg.header.stamp) - m_last_odom_msg_time < std::chrono::seconds(0)) {
      twist_base_world_in_base = utils::twist_from_base_velocity(m_mobile_base.velocity_in_base);
    } else {
      Eigen::fromMsg(odom_msg.twist.twist, twist_base_world_in_base);
    }
    m_last_odom_msg_time = odom_msg.header.stamp;

    twist_base_world_in_world = rdyn::spatialRotation(twist_base_world_in_base, m_T_world_base.linear());

    // Build state vectors
    Eigen::Vector6d base_read;
    base_read.tail<M_SE2>() = utils::base_velocity_from_twist(twist_base_world_in_world);
    base_read.head<2>() = m_T_world_base.translation().head<2>();
    base_read(2) = Eigen::AngleAxisd(m_T_world_base.linear()).angle();
    Eigen::Vector6d estim_base = m_base_position_filter.update(base_read, m_qpp.head<M_SE2>());
    m_qp.head<M_SE2>() = estim_base.tail<M_SE2>();
    m_q.head<M_SE2>() = estim_base.head<M_SE2>();
    // m_qp.head<M_SE2>() = utils::base_velocity_from_twist(twist_base_world_in_world);
    // m_q.head<2>() = m_T_world_base.translation().head<2>();
    // m_q(2) = Eigen::AngleAxisd(m_T_world_base.linear()).angle();
  }
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
#endif

#endif

  Eigen::Affine3d T_world_tool = m_chain_world_tool->getTransformation(m_q);
  Eigen::Vector6d twist_tool_world_in_world = m_chain_world_tool->getJacobian(m_q) * m_qp;
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

    Eigen::Affine3d T_base_tool = m_chain_base_tool->getTransformation(m_q.tail(m_nax));
    Eigen::Affine3d T_base_sensor = m_chain_base_sensor->getTransformation(m_q.tail(m_nax));
    Eigen::Affine3d T_tool_sensor = T_base_tool.inverse() * T_base_sensor;

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

    Eigen::Vector6d wrench_tool_in_tool = rdyn::spatialDualTranformation(wrench_sensor_in_sensor, T_tool_sensor);
    wrench_tool_in_world = rdyn::spatialRotation(wrench_tool_in_tool, T_world_tool.linear()) - m_offset_wrench_tool_in_world;
  }

  std::transform(wrench_tool_in_world.begin(), wrench_tool_in_world.end(), m_wrench_notch.begin(), wrench_tool_in_world.begin(),
                 [](const double w, const std::shared_ptr<NotchFilter>& notch) { return notch->update(w); });

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
  cart_vel_error_tool_target_in_world = (twist_tool_world_in_world - m_computed_target_twist_tool_world_in_world)
                                          .cwiseProduct(m_elastoplastic_model->get_enabled_axis());

  ClikData clik_data{.position_references = full_position_references,
                     .velocity_references = full_velocity_references,
                     .twist_tool_world_in_world = twist_tool_world_in_world,
                     .T_world_tool = T_world_tool,
                     .target_acc_tool_target_in_world = reference_target_acc_tool_world_in_world,
                     .J_world_tool_in_world = J_world_tool_in_world,
                     .target_T_world_tool = reference_target_T_world_tool,
                     .target_twist_tool_world_in_world = reference_target_twist_tool_world_in_world,
                     .wrench_tool_in_world = wrench_tool_in_world};

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
  } else {
    Eigen::VectorXd qepp = solution_qp.value().head(m_full_nax);
    Eigen::Vector6d xepp = solution_qp.value().tail<M_SE3>();
    m_qpp = qepp;
    m_qp += qepp * m_dt;
    m_q += m_qp * m_dt; // Symplectic Euler
  }

  m_zp = m_elastoplastic_model->update_z(cart_vel_error_tool_target_in_world, m_dt);
  bool reset = m_elastoplastic_model->reset(wrench_tool_in_world.cwiseProduct(m_elastoplastic_model->get_enabled_axis()),
                                            cart_vel_error_tool_target_in_world);
  if (reset) {
    RCLCPP_WARN_STREAM(get_node()->get_logger(), "Reset to Elastic Mode");
  }
  m_computed_target_T_world_tool = reset ? T_world_tool : m_computed_target_T_world_tool;

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
    m_mobile_base.velocity_in_base = utils::base_velocity_from_twist(qp_base_in_base);

    // BEGIN - Check Saturation Base
    // If the QP works, this shouldn't be necessary
    for (size_t idx = 0; idx < M_SE2; ++idx) {
      if (std::abs(m_mobile_base.velocity_in_base(idx)) > m_mobile_base.vel_limits(idx)) {
        // RCLCPP_WARN_STREAM(this->get_node()->get_logger(),
        // "Saturation of Velocity on base linear direction "
        // << idx << ": " << m_mobile_base.velocity_in_base(idx) << " should be "
        // << utils::sgn(m_mobile_base.velocity_in_base(idx)) * m_mobile_base.vel_limits(idx));
      }
    }
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
    Eigen::Vector6d base_twist_in_base = m_mobile_base.twist_in_base();
    geometry_msgs::msg::Twist cmd_vel = Eigen::toMsg(base_twist_in_base);

    m_pub_cmd_vel->publish(cmd_vel);

    m_T_world_base = rdyn::spatialIntegration(m_T_world_base, base_twist_in_base, m_dt);
  }

  m_q_prec = m_q;
  m_qp_prec = m_qp;
  m_qpp_prec = m_qpp;

  // *************
  // ** PUBLISH **
  // *************
  if (m_parameters.debug.pub) {
    auto time_now = this->get_node()->get_clock()->now();
    std_msgs::msg::Float64MultiArray msg_z;
    msg_z.data.push_back(m_elastoplastic_model->z()(0));
    msg_z.data.push_back(m_elastoplastic_model->z()(1));
    msg_z.data.push_back(m_elastoplastic_model->z()(2));
    msg_z.data.push_back(m_elastoplastic_model->z()(3));
    msg_z.data.push_back(m_elastoplastic_model->z()(4));
    msg_z.data.push_back(m_elastoplastic_model->z()(5));
    msg_z.data.push_back(m_zp(0));
    msg_z.data.push_back(m_zp(1));
    msg_z.data.push_back(m_zp(2));
    msg_z.data.push_back(m_zp(3));
    msg_z.data.push_back(m_zp(4));
    msg_z.data.push_back(m_zp(5));
    m_pub_z->publish(msg_z);

    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = time_now;
    joint_state_msg.name = m_joint_names;
    joint_state_msg.position.resize(m_nax);
    joint_state_msg.velocity.resize(m_nax);
    std::ranges::copy(q_start.tail(m_nax), joint_state_msg.position.begin());
    std::ranges::copy(qp_start.tail(m_nax), joint_state_msg.velocity.begin());
    m_estim_joint_state->publish(joint_state_msg);

    std_msgs::msg::Float64MultiArray cmd_q_msg;
    cmd_q_msg.data.resize(m_q.size());
    std::ranges::copy(m_q, cmd_q_msg.data.begin());
    m_cmd_pose_pub->publish(cmd_q_msg);

    // geometry_msgs::msg::WrenchStamped msg_wrench_in_tool;
    // msg_wrench_in_tool.header.frame_id = m_parameters.frames.tool;
    // msg_wrench_in_tool.header.stamp = time_now;
    // msg_wrench_in_tool.wrench.force.x = wrench_tool_in_tool[0];
    // msg_wrench_in_tool.wrench.force.y = wrench_tool_in_tool[1];
    // msg_wrench_in_tool.wrench.force.z = wrench_tool_in_tool[2];
    // msg_wrench_in_tool.wrench.torque.x = wrench_tool_in_tool[3];
    // msg_wrench_in_tool.wrench.torque.y = wrench_tool_in_tool[4];
    // msg_wrench_in_tool.wrench.torque.z = wrench_tool_in_tool[5];
    // m_pub_wrench_in_tool->publish(msg_wrench_in_tool);

    geometry_msgs::msg::WrenchStamped msg_wrench_in_world;
    msg_wrench_in_world.header.frame_id = m_parameters.frames.map;
    msg_wrench_in_world.header.stamp = time_now;
    msg_wrench_in_world.wrench.force.x = wrench_tool_in_world[0];
    msg_wrench_in_world.wrench.force.y = wrench_tool_in_world[1];
    msg_wrench_in_world.wrench.force.z = wrench_tool_in_world[2];
    msg_wrench_in_world.wrench.torque.x = wrench_tool_in_world[3];
    msg_wrench_in_world.wrench.torque.y = wrench_tool_in_world[4];
    msg_wrench_in_world.wrench.torque.z = wrench_tool_in_world[5];
    m_pub_wrench_in_world->publish(msg_wrench_in_world);

    m_pub_cart_vel_error->publish(tf2::toMsg(cart_vel_error_tool_target_in_world));
    m_pub_twist_in_world->publish(tf2::toMsg(twist_tool_world_in_world));

    sensor_msgs::msg::JointState jref_msg;
    jref_msg.header.stamp = time_now;
    jref_msg.name = m_joint_names;
    jref_msg.position.resize(m_full_nax);
    jref_msg.velocity.resize(m_full_nax);
    std::copy(full_position_references.begin(), full_position_references.end(), jref_msg.position.begin());
    std::copy(full_velocity_references.begin(), full_velocity_references.end(), jref_msg.velocity.begin());
    m_pub_joint_reference->publish(jref_msg);

    geometry_msgs::msg::PoseStamped fk_msg;
    Eigen::Affine3d fk = m_chain_world_tool->getTransformation(m_q);
    fk_msg.header.stamp = time_now;
    fk_msg.header.frame_id = m_parameters.frames.map;
    fk_msg.pose = Eigen::toMsg(fk);
    m_pub_fk_world_tool->publish(fk_msg);

    fk = m_chain_base_tool->getTransformation(m_q.tail(m_nax));
    fk_msg.header.stamp = time_now;
    fk_msg.header.frame_id = m_parameters.frames.base;
    fk_msg.pose = Eigen::toMsg(fk);
    m_pub_fk_base_tool->publish(fk_msg);

    std_msgs::msg::Float64MultiArray weights_msg;
    weights_msg.data = std::vector<double>(m_W.diagonal().begin(), m_W.diagonal().end());
    weights_msg.data.push_back(m_logistic.get(m_mobile_base.velocity_in_base.array()));
    m_pub_weights->publish(weights_msg);

    geometry_msgs::msg::PoseStamped cmp_target_T_msg;
    cmp_target_T_msg.pose = tf2::toMsg(m_computed_target_T_world_tool);
    cmp_target_T_msg.header.stamp = time_now;
    cmp_target_T_msg.header.frame_id = m_parameters.frames.map;
    m_computed_pose_pub->publish(cmp_target_T_msg);

    geometry_msgs::msg::Twist cmp_target_twist_msg;
    cmp_target_twist_msg = tf2::toMsg(m_computed_target_twist_tool_world_in_world);
    m_computed_twist_pub->publish(cmp_target_twist_msg);

    geometry_msgs::msg::PoseStamped interp_msg;
    interp_msg.pose = tf2::toMsg(reference_target_T_world_tool);
    interp_msg.header.stamp = time_now;
    interp_msg.header.frame_id = m_parameters.frames.map;
    m_interp_pose_pub->publish(interp_msg);

    geometry_msgs::msg::Twist target_twist_msg;
    target_twist_msg = tf2::toMsg(reference_target_twist_tool_world_in_world);
    m_interp_twist_pub->publish(target_twist_msg);

    std_msgs::msg::Float64 buffer_msg;
    double tmp;
    std::tie(buffer_msg.data, tmp) = m_elastoplastic_model->get_reset_buffer_status();
    m_pub_reset_buffer->publish(buffer_msg);
  }

  std_msgs::msg::Float64MultiArray mode_msg;
  if (m_elastoplastic_model->is_plastic()) {
    mode_msg.data.push_back(Mode::PLASTIC);
  } else if (m_elastoplastic_model->to_restore()) {
    mode_msg.data.push_back(Mode::RESTORE);
  } else {
    mode_msg.data.push_back(Mode::ELASTIC);
  }
  mode_msg.data.push_back(get_node()->get_clock()->now().seconds());
  m_pub_controller_mode->publish(mode_msg);

  rclcpp::Time t_end = get_node()->get_clock()->now();
  std_msgs::msg::Float64 cycle_time_msg;
  //cycle_time_msg.data = static_cast<double>((t_end - t_start).nanoseconds()) / 1e-3; // As [ms]
  cycle_time_msg.data = period.seconds();
  m_pub_timing->publish(cycle_time_msg);

  return controller_interface::return_type::OK;
}


std::optional<Eigen::VectorXd> ElastoplasticController::clik(const ClikData& a_data) {

  Eigen::Vector6d acc_non_linear_in_world = m_chain_world_tool->getDTwistNonLinearPartTool(m_q, m_qp);
  Eigen::Vector6d enabled_axis = m_elastoplastic_model->get_enabled_axis();

  auto t_start_qp = get_node()->get_clock()->now();
  const unsigned int prb_dim = m_full_nax + M_SE3;

  elastoplastic::Task task_cart_vel(prb_dim, M_SE3);
  elastoplastic::Task task_cart_pos(prb_dim, M_SE3);
  elastoplastic::Task task_minimize_cart_acc(prb_dim, M_SE3);
  elastoplastic::Task task_joint_pos(prb_dim, m_full_nax);
  elastoplastic::Task task_joint_vel(prb_dim, m_full_nax);
  elastoplastic::Task task_minimize_joint_acc(prb_dim, m_full_nax);
  elastoplastic::Task task_admittance(prb_dim, M_SE3);

  /**********************
   ** Task Definitions **
   **********************/

  // Task Cartesian : Minimize cartesian distance from reference twist
  task_cart_vel.A().rightCols(M_SE3) = Eigen::Matrix6d::Identity() * m_dt;
  task_cart_vel.b() = (m_computed_target_twist_tool_world_in_world - a_data.target_twist_tool_world_in_world);

  // Task Cartesian : Minimize difference between the real target and the computed one
  Eigen::Vector6d ref_p_err;
  rdyn::getFrameDistanceQuat(m_computed_target_T_world_tool, a_data.target_T_world_tool, ref_p_err);
  task_cart_pos.A().rightCols<M_SE3>() = Eigen::Matrix6d::Identity() * 0.5 * std::pow(m_dt, 2);
  task_cart_pos.b() << ref_p_err + m_computed_target_twist_tool_world_in_world * m_dt;

  // Task Cartesian:
  elastoplastic::Task task_minimize_cart_vel(prb_dim, M_SE3);
  task_minimize_cart_vel.A().rightCols<M_SE3>() = Eigen::Matrix6d::Identity() * m_dt;
  task_minimize_cart_vel.b() << a_data.twist_tool_world_in_world;

  task_minimize_cart_acc.A().rightCols<M_SE3>().setIdentity();
  task_minimize_cart_acc.b().setZero();

  // Task: Admittance
  auto [K, D] = m_elastoplastic_model->compute_variable_matrices(a_data.T_world_tool);
  auto invM = m_elastoplastic_model->get_inertia_inv();
  Eigen::Vector6d twist_error_tool_world_in_world =
    a_data.twist_tool_world_in_world - m_computed_target_twist_tool_world_in_world;
  Eigen::Vector6d pose_error_tool_world_in_world;
  rdyn::getFrameDistanceQuat(a_data.T_world_tool, m_computed_target_T_world_tool, pose_error_tool_world_in_world);

  Eigen::Matrix6d adm = Eigen::Matrix6d::Identity() + invM * D * m_dt + 0.5 * invM * K * std::pow(m_dt, 2);
  task_admittance.A() << adm * a_data.J_world_tool_in_world, -adm;
  task_admittance.b() << adm * acc_non_linear_in_world + invM * D * twist_error_tool_world_in_world +
                           invM * K *
                             (twist_error_tool_world_in_world * m_dt +
                              m_elastoplastic_model->z() +
                              pose_error_tool_world_in_world.cwiseProduct(Eigen::Vector6d::Ones() - enabled_axis)) -
                           invM * (a_data.wrench_tool_in_world);

  /****************
   ** Task Stack **
   ****************/
  constexpr double STACK_LEVEL_STEP = 1e-3;
  constexpr int STACK_LEVEL_ZERO = 0;
  elastoplastic::Stack sot(prb_dim, STACK_LEVEL_STEP, STACK_LEVEL_ZERO);
  double cart_vel_weight = 1e0;

  /* Variable stack */
  constexpr int CART_POS_LEVEL_OFFSET = 1;
  int cart_pos_level = STACK_LEVEL_ZERO + CART_POS_LEVEL_OFFSET;
  if (!m_elastoplastic_model->is_plastic() && m_elastoplastic_model->to_restore() && m_parameters.impedance.plastic_restoration) {
    RCLCPP_DEBUG_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1, "Is restoring");
    cart_pos_level -= CART_POS_LEVEL_OFFSET;
  }
  sot.insert_task(task_cart_pos, cart_pos_level, 1e2);

  /* Constant stack */
  sot.push_task(task_cart_vel); // TODO: trova modo intelligente per bilanciare cart pos/vel in funzione della velocità
  sot.new_level();
  sot.push_task(task_minimize_cart_acc);

  // Weighting matrix
  m_W = Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) / prb_dim * sot.G().trace();
  if (m_mobile_base.enabled) {
    // auto logis = filters::exponentialSmoothing(m_logistic.get(m_mobile_base.velocity_in_base), m_logis_prec, 0.1);
    // m_logis_prec = logis;
    m_W.diagonal()(2) *= 1e6;
    m_W.diagonal().head<2>() *= 1e3;
    auto logis = 1;
    double weight_coeff = (1.0 + m_parameters.clik.alpha_gain * m_elastoplastic_model->alpha() * logis);
    m_W.diagonal().head<2>() /= weight_coeff;
    // m_W.diagonal().tail(m_nax) *= weight_coeff;
  }

  // Task: Minimize joint acceleration and weighting
  task_minimize_joint_acc.A().leftCols(m_full_nax) = m_W;
  task_minimize_joint_acc.b().setZero();

  // Task: Joint Velocity
  task_joint_vel.A().leftCols(m_full_nax) += -m_kv_joint_task * Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  task_joint_vel.b() += m_kv_joint_task * (a_data.velocity_references - m_qp);
  task_joint_vel.W() = m_W.transpose() * m_W;

  // Task: Joint Position
  task_joint_pos.A().leftCols(m_full_nax) +=
    -m_kp_joint_task * 0.5 * Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * std::pow(m_dt, 2);
  task_joint_pos.b() += m_kp_joint_task * (a_data.position_references - (m_q + m_qp * m_dt));
  task_joint_pos.W() = m_W.transpose() * m_W;

  sot.push_task(task_joint_vel);
  sot.push_task(task_joint_pos);
  sot.push_task(task_minimize_joint_acc);

  /********************
   ** EQ Constraints **
   ********************/
  elastoplastic::EqualitySet eq_set(prb_dim);
  eq_set.push_constraint(task_admittance);
  eq_set.compute_set();


  /***********************
   ** DISEQ Constraints **
   ***********************/
  elastoplastic::InequalityConstraint ineq_qpp_max(prb_dim, m_full_nax, "Joint Acceleration Max");
  elastoplastic::InequalityConstraint ineq_qpp_min(prb_dim, m_full_nax, "Joint Acceleration Min");
  elastoplastic::InequalityConstraint ineq_qp_max(prb_dim, m_full_nax, "Joint Velocity Max");
  elastoplastic::InequalityConstraint ineq_qp_min(prb_dim, m_full_nax, "Joint Velocity Min");
  elastoplastic::InequalityConstraint ineq_q_max(prb_dim, m_nax, "Joint Position Max");
  elastoplastic::InequalityConstraint ineq_q_min(prb_dim, m_nax, "Joint Position Min");

  // Velocity
  ineq_qp_min.CI().leftCols(m_full_nax) << Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  ineq_qp_min.ci().segment(m_mobile_base.nax(), m_nax) = (m_qp.tail(m_nax) + m_limits.vel);

  ineq_qp_max.CI().leftCols(m_full_nax) << -Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  ineq_qp_max.ci().segment(m_mobile_base.nax(), m_nax) = (m_limits.vel - m_qp.tail(m_nax));

  // Acceleration
  ineq_qpp_min.CI().leftCols(m_full_nax) << Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
  ineq_qpp_min.ci().segment(m_mobile_base.nax(), m_nax) = m_limits.acc;

  ineq_qpp_max.CI().leftCols(m_full_nax) << -Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
  ineq_qpp_max.ci().segment(m_mobile_base.nax(), m_nax) = m_limits.acc;

  // Positions
  ineq_q_min.CI().block(0, m_mobile_base.nax(), m_nax, m_nax) << Eigen::MatrixXd::Identity(m_nax, m_nax) * 0.5 * m_dt * m_dt;
  ineq_q_min.ci().head(m_nax) = (m_q.tail(m_nax) + m_qp.tail(m_nax) * m_dt) - m_limits.pos_lower;

  ineq_q_max.CI().block(0, m_mobile_base.nax(), m_nax, m_nax) << -Eigen::MatrixXd::Identity(m_nax, m_nax) * 0.5 * m_dt * m_dt;
  ineq_q_max.ci().head(m_nax) = m_limits.pos_upper - (m_q.tail(m_nax) + m_qp.tail(m_nax) * m_dt);

  // Move base limits to world
  if (m_mobile_base.enabled) {
    // Velocity
    Eigen::Vector6d max_vel_base_in_world = utils::twist_from_base_velocity(m_mobile_base.vel_limits);
    Eigen::Vector6d max_vel_base_in_base = rdyn::spatialRotation(max_vel_base_in_world, m_T_world_base.linear().transpose());
    Eigen::Vector3d max_vel_base = utils::base_velocity_from_twist(max_vel_base_in_base);
    ineq_qp_min.ci().head<M_SE2>() << m_qp.head<M_SE2>() + max_vel_base;
    ineq_qp_max.ci().head<M_SE2>() << max_vel_base - m_qp.head<M_SE2>();

    // Acceleration
    Eigen::Vector6d max_acc_base_in_world = utils::twist_from_base_velocity(m_mobile_base.acc_limits);
    Eigen::Vector6d max_acc_base_in_base = rdyn::spatialRotation(max_acc_base_in_world, m_T_world_base.linear().transpose());
    Eigen::Vector3d max_acc_base = utils::base_velocity_from_twist(max_acc_base_in_base);
    ineq_qpp_min.ci().head<M_SE2>() << max_acc_base;
    ineq_qpp_max.ci().head<M_SE2>() << max_acc_base;
  }

  elastoplastic::InequalitySet ineq_set(prb_dim);
  ineq_set.push_constraint(ineq_q_min);
  ineq_set.push_constraint(ineq_q_max);
  ineq_set.push_constraint(ineq_qp_min);
  ineq_set.push_constraint(ineq_qp_max);
  ineq_set.push_constraint(ineq_qpp_min);
  ineq_set.push_constraint(ineq_qpp_max);

  ineq_set.compute_set();

  /***********
   ** Solve **
   ***********/
  elastoplastic::SolverQP solver(prb_dim, sot, eq_set, ineq_set);
  auto [solutionQP, status] = solver.solve();

  if (status != SolverStatus::EIQUADPROG_FAST_OPTIMAL) {
    Eigen::LLT<Eigen::MatrixXd> chol(sot.G());
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Problem unfeasible. Solver status : " << status << ". Is G Positive Definite: "
                                                               << (chol.info() == Eigen::ComputationInfo::Success));
    return std::nullopt;
  }

  if (solutionQP.hasNaN()) {
    RCLCPP_ERROR(get_node()->get_logger(), "NaN in the solution!");
    RCLCPP_DEBUG_STREAM(m_node_support->get_logger(), "Dump: "
                                                        << "\n## first round sol [qpp(" << m_full_nax << "), slack("
                                                        << prb_dim - m_full_nax << ")]##\n"
                                                        << solutionQP.transpose() << "\n## first round ret ##\n"
                                                        << status << "## G ## " << sot.G() << "\n## F ##" << sot.F().transpose()
                                                        << "\n## eq_set.CE() ## " << eq_set.CE() << "\n ## eq_set.ce() ## "
                                                        << eq_set.ce().transpose() << "\n## CI ## " << ineq_set.CI()
                                                        << "\n## ci ##" << ineq_set.ci().transpose());
    return std::nullopt;
  }

  // Should be useless but...
  // if (ineq_set.violations(solutionQP) != 0) {
  // RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Constraint violated:");
  // auto ineq_violated = ineq_set.which_violations(solutionQP);
  // std::for_each(ineq_violated.begin(), ineq_violated.end(), [this, &solutionQP](const InequalityConstraint& ineq) {
  // RCLCPP_ERROR_STREAM(get_node()->get_logger(),
  // " - " << ineq.description() << " | values: " << ineq.value(solutionQP).transpose());
  // });
  // return std::nullopt;
  // }

  geometry_msgs::msg::WrenchStamped msg_wrench_admittance;
  msg_wrench_admittance.header.frame_id = m_parameters.frames.map;
  msg_wrench_admittance.header.stamp = get_node()->get_clock()->now();
  tf2::toMsg((task_admittance.value(solutionQP) + invM * (a_data.wrench_tool_in_world)).head<3>(),
             msg_wrench_admittance.wrench.force);
  tf2::toMsg((task_admittance.value(solutionQP) + invM * (a_data.wrench_tool_in_world)).tail<3>(),
             msg_wrench_admittance.wrench.torque);
  m_pub_admittance_force->publish(msg_wrench_admittance);

  m_computed_target_acc_tool_world_in_world = solutionQP.tail<M_SE3>();
  return solutionQP;
}

} // namespace elastoplastic

PLUGINLIB_EXPORT_CLASS(elastoplastic::ElastoplasticController, controller_interface::ChainableControllerInterface);
