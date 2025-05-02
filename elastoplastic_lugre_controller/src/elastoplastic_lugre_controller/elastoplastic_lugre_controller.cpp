#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"

#include "rclcpp/logger.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "urdfdom_headers/urdf_model/model.h"

#include "eiquadprog/eiquadprog-fast.hpp"

#include "control_toolbox/filters.hpp"

#include <chrono>
#include <algorithm>

namespace elastoplastic
{

using namespace std::chrono_literals;


controller_interface::CallbackReturn ElastoplasticController::on_init()
{
  m_param_listener = std::make_shared<elastoplastic_controller::ParamListener>(this->get_node());
  return controller_interface::CallbackReturn::SUCCESS;
}


void ElastoplasticController::configure_after_robot_description_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
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
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Cannot create URDF model from robot_description provided by controller_manager");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "URDF model created");

  Eigen::Vector3d gravity({m_parameters.gravity.at(0), m_parameters.gravity.at(1),
      m_parameters.gravity.at(2)});
  m_chain_base_tool = rdyn::createChain(
    *urdf_model, m_parameters.frames.base,
    m_parameters.frames.tool, gravity);
  m_chain_base_sensor = rdyn::createChain(
    *urdf_model, m_parameters.frames.base,
    m_parameters.frames.sensor, gravity);
  if (not m_chain_base_tool) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Cannot create rdyn chain from base (%s) to tool (%s)",
      m_parameters.frames.base.c_str(), m_parameters.frames.tool.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  if (not m_chain_base_sensor) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Cannot create rdyn chain from base (%s) to sensor (%s)",
      m_parameters.frames.base.c_str(), m_parameters.frames.sensor.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  if (m_mobile_base.enabled) {
    const std::string mobile_base_urdf =
      R"(<?xml version='1.0'?>
<robot name='base'>
<link name='x_base'/>
<link name='y_base'/>
<link name='rz_base'/>
<link name='mount_link'/>
<joint name='move_x' type='prismatic'>
  <parent link='x_base'/>
  <child link='y_base'/>
  <origin xyz='0 0 0'/>
  <axis xyz='1 0 0'/>
  <limit lower='-1e10' upper='1e10' effort='1e10' velocity='1e10'/>
</joint>
<joint name='move_y' type='prismatic'>
  <parent link='y_base'/>
  <child link='rz_base'/>
  <origin xyz='0 0 0'/>
  <axis xyz='0 1 0'/>
  <limit lower='-1e10' upper='1e10' effort='1e10' velocity='1e10'/>
</joint>
<joint name='rot_z' type='revolute'>
  <parent link='rz_base'/>
  <child link='mount_link'/>
  <origin xyz='0 0 0'/>
  <axis xyz='0 0 1'/>
  <limit lower='-1e10' upper='1e10' effort='1e10' velocity='1e10'/>
</joint>
</robot>
    )";
    urdf::ModelInterfaceSharedPtr mobile_base_model = urdf::parseURDF(mobile_base_urdf);
    m_chain_world_base =
      rdyn::createChain(*mobile_base_model, "x_base", "mount_link", {0, 0, -9.806});

    m_chain_world_tool = rdyn::joinChains(m_chain_world_base, m_chain_base_tool);
  } else {
    m_chain_world_tool = m_chain_base_tool;
  }

  if (not m_chain_world_tool) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Cannot create rdyn chain from world to tool (%s)",
      m_parameters.frames.tool.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "RDyn chains created");

  m_limits.pos_upper.resize(m_nax);
  m_limits.pos_lower.resize(m_nax);
  m_limits.vel.resize(m_nax);
  m_limits.acc.resize(m_nax);

  for (size_t ax = 0; ax < m_nax; ++ax) {
    m_limits.pos_upper(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->upper;
    m_limits.pos_lower(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->lower;

    if (utils::almost_zero(m_limits.pos_upper(ax)) && utils::almost_zero(m_limits.pos_lower(ax))) {
      m_limits.pos_upper(ax) = std::numeric_limits<double>::infinity();
      m_limits.pos_lower(ax) = -std::numeric_limits<double>::infinity();
      RCLCPP_WARN(
        get_node()->get_logger(), "Upper and Lower limits are both equal to 0, set +/- infinity");
    }

    m_limits.vel(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->velocity;
    m_limits.acc(ax) = 10 * m_limits.vel(ax);
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


controller_interface::CallbackReturn ElastoplasticController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  m_parameters = m_param_listener->get_params();

  if (m_parameters.debug.log) {
    this->get_node()->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }

  m_elastoplastic_model = std::make_unique<ElastoplasticModel>(utils::get_model_data(m_parameters));

  m_mobile_base.enabled = m_parameters.mobile_base.enabled;

  m_full_nax = m_mobile_base.enabled ? m_parameters.joints.size() + m_mobile_base.nax() : m_parameters.joints.size();
  m_nax = m_parameters.joints.size();
  RCLCPP_DEBUG(
    this->get_node()->get_logger(), "Full NAx: %ld, Manipulator NAx: %ld", m_full_nax, m_nax);
  m_q.resize(m_full_nax);
  m_qp.resize(m_full_nax);
  m_qpp.resize(m_full_nax);

  m_q_prec.resize(m_full_nax);
  m_qp_prec.resize(m_full_nax);
  m_qpp_prec.resize(m_full_nax);

  if (m_parameters.clik.task.weights.size() != m_full_nax) {
    RCLCPP_ERROR(get_node()->get_logger(), "weights size is not %ld", m_full_nax);
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (std::ranges::min(m_parameters.impedance.inertia) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Inertia has negative values!");
    return controller_interface::CallbackReturn::FAILURE;
  }

  using namespace std::placeholders;
  m_mobile_base_pose_updated = false;
  if(m_mobile_base.enabled)
  {
    m_sub_mobile_base_target = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(
      m_parameters.mobile_base.input_target_topic, 1,
      std::bind(&ElastoplasticController::get_mobile_base_target_callback, this, _1));
    m_sub_mobile_base_odometry = this->get_node()->create_subscription<nav_msgs::msg::Odometry>(
      m_parameters.mobile_base.odom, 1,
      std::bind(&ElastoplasticController::get_odometry_callback, this, _1));
    m_sub_mobile_base_pose = this->get_node()->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      m_parameters.mobile_base.localization_topic, 2,
        std::bind(&ElastoplasticController::get_localization_callback, this, _1));
  }
  else
  {
    m_mobile_base_pose_updated = true;
  }
  m_pub_cmd_vel = this->get_node()->create_publisher<geometry_msgs::msg::Twist>(m_parameters.cmd_vel_topic, 1);
  m_pub_timing = this->get_node()->create_publisher<std_msgs::msg::Float64>("~/controller_period", 1);

  m_ft_sensor = std::make_unique<semantic_components::ForceTorqueSensor>(m_parameters.ft_sensor_name);

  if (m_parameters.debug.pub) {
    m_pub_z = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/z", 10);
    m_pub_w = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/w", 10);
    m_pub_friction_in_world = this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/friction_in_world", 10);
    m_pub_wrench_in_world = this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/wrench_in_world", 10);
    m_pub_wrench_in_tool = this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/wrench_in_tool", 10);
    m_pub_cart_vel_error = this->get_node()->create_publisher<geometry_msgs::msg::Twist>(
      "~/cart_vel_error", 10);
    m_pub_delta_acceleration = this->get_node()->create_publisher<geometry_msgs::msg::Twist>(
      "~/delta/acceleration", 10);
    m_pub_delta_velocity = this->get_node()->create_publisher<geometry_msgs::msg::Twist>(
      "~/delta/velocity", 10);
    m_pub_delta_pose = this->get_node()->create_publisher<geometry_msgs::msg::Twist>(
      "~/delta/pose",
      10);
    m_clik_components_pub = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/clik_components", rclcpp::QoS(10).durability_volatile().reliable());
    m_clik_correction_pub = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/clik_correction", rclcpp::QoS(10).durability_volatile().reliable());
    m_pub_twist_in_world = this->get_node()->create_publisher<geometry_msgs::msg::Twist>(
      "~/twist_in_world", 10);
    m_xp_pub = this->get_node()->create_publisher<geometry_msgs::msg::Twist>(
      "~/new_twist_in_world",
      10);
    m_pub_joint_reference = this->get_node()->create_publisher<sensor_msgs::msg::JointState>(
      "~/joint_references", 10);
    m_pub_fk_world_tool = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/fk_world_tool", rclcpp::QoS(1));
    m_pub_fk_base_tool = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/fk_base_tool", rclcpp::QoS(1));
    m_pub_next_pose = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/next_pose", 10);
    m_pub_weights = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/weights", 10);
    m_pub_alfa = this->get_node()->create_publisher<std_msgs::msg::Float64>("~/alfa", 10);
    m_interp_pose_pub = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/interp_pose", 5);
    m_interp_twist_pub = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/interp_twist", 10);
    m_computed_pose_pub = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/computed_pose", 5);
    m_computed_twist_pub = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/computed_twist", 10);
  }

  m_state_interfaces_names.reserve(m_allowed_interface_types.size());
  m_command_interfaces_names.reserve(m_allowed_interface_types.size());

  for (const auto & interface : m_allowed_interface_types) {
    auto it = std::ranges::find(m_parameters.state_interfaces, interface);
    if (it == m_parameters.state_interfaces.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing State interfaces from parameters");
      return controller_interface::CallbackReturn::FAILURE;
    } else {
      m_state_interfaces_names.push_back(*it);
    }

    it = std::ranges::find(m_parameters.command_interfaces, interface);
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
  }
  else
  {
#ifdef USE_LATEST_ROS2_CONTROL
    RCLCPP_DEBUG(get_node()->get_logger(), "Robot description from class member");
    std_msgs::msg::String::SharedPtr rd = std::make_shared<std_msgs::msg::String>();
    rd->data = this->get_robot_description();
    configure_after_robot_description_callback(rd);
#else
    RCLCPP_DEBUG(get_node()->get_logger(), "Robot description from topic");
    rclcpp::QoS qos(1);
    qos.transient_local();
    m_sub_robot_description = get_node()->create_subscription<std_msgs::msg::String>(
      m_parameters.robot_description_topic, qos,
      std::bind(
        &ElastoplasticController::configure_after_robot_description_callback, this,
        std::placeholders::_1));
    m_robot_description_configuration = RDStatus::EMPTY;
#endif
  }

  std::ranges::fill(m_used_command_interfaces, false);
  if (std::ranges::find(
      m_command_interfaces_names,
      m_allowed_interface_types[0]) != m_command_interfaces_names.end())
  {
    m_used_command_interfaces.at(0) = true;
  }
  if (std::ranges::find(
      m_command_interfaces_names,
      m_allowed_interface_types[1]) != m_command_interfaces_names.end())
  {
    m_used_command_interfaces.at(1) = true;
  }

  m_W.setIdentity(m_full_nax, m_full_nax);

  m_kp_joint_task = m_parameters.clik.joint_task.kp;
  m_kv_joint_task = m_parameters.clik.joint_task.kv;

  if (m_mobile_base.enabled) {
    m_mobile_base.vel_limits = {m_parameters.mobile_base.max_vel.linear[0], m_parameters.mobile_base.max_vel.linear[1],
                                m_parameters.mobile_base.max_vel.angular};
    m_mobile_base.acc_limits = {m_parameters.mobile_base.max_acc_x, m_parameters.mobile_base.max_acc_y,
                                m_parameters.mobile_base.max_acc_yaw};
  }

  m_logistic = {.max = m_parameters.impedance.logistic.max,
                .slope = m_parameters.impedance.logistic.slope,
                .inflection = m_parameters.impedance.logistic.inflection * m_mobile_base.vel_limits};

  // The parameter update_rate, if not defined, is provided by the controller_manager
  auto update_rate = this->get_node()->get_parameter("update_rate").as_int();
  m_dt = 1.0 / double(update_rate);
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), "dt: " << m_dt);
  if (m_dt < M_MINIMUM_SAMPLING_TIME) {
    RCLCPP_FATAL(this->get_node()->get_logger(), "dt: %.6f, too low. Minimum sampling time: %.6f", m_dt, M_MINIMUM_SAMPLING_TIME);
    return controller_interface::CallbackReturn::ERROR;
  }

  m_carteisan_trj_sub = get_node()->create_subscription<moveit_msgs::msg::CartesianTrajectory>(
    m_parameters.cartesian_trajectory_topic, 1, [this](const moveit_msgs::msg::CartesianTrajectory& msg) {
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "got trajectory");
      m_interpolator = utils::interpolation::Interpolator::from_msg(msg);
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "-> " << m_interpolator.is_empty());
    });

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
ElastoplasticController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interface_configuration.names.reserve(
    m_parameters.joints.size() * m_allowed_interface_types.size() + 6);

  for (const auto & jnt : m_parameters.joints) {
    state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION));
  }
  for (const auto & jnt : m_parameters.joints) {
    state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
  }

  std::vector<std::string> ft_interfaces = m_ft_sensor->get_state_interface_names();
  state_interface_configuration.names.insert(
    state_interface_configuration.names.end(), ft_interfaces.begin(), ft_interfaces.end());

  return state_interface_configuration;
}


controller_interface::InterfaceConfiguration
ElastoplasticController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interface_configuration;
  command_interface_configuration.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interface_configuration.names.reserve(
    m_parameters.joints.size() * m_command_interfaces_names.size());
  if (std::ranges::find(m_command_interfaces_names, m_allowed_interface_types[0]) != m_command_interfaces_names.end()) {
    for (const auto & jnt : m_parameters.joints) {
      command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, m_allowed_interface_types[0]));
    }
  }
  if (std::ranges::find(m_command_interfaces_names, m_allowed_interface_types[1]) != m_command_interfaces_names.end()) {
    for (const auto & jnt : m_parameters.joints) {
      command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, m_allowed_interface_types[1]));
    }
  }

  return command_interface_configuration;
}


controller_interface::CallbackReturn ElastoplasticController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto t_start = get_node()->get_clock()->now();
  while(!ready_for_activation() &&
        !m_mobile_base_pose_updated &&
         get_node()->get_clock()->now() - t_start < std::chrono::seconds(10))
  {
      RCLCPP_INFO(this->get_node()->get_logger(), "Waiting robot description-related operations");
      get_node()->get_clock()->sleep_for(std::chrono::milliseconds(1000));
  }
  if (m_robot_description_configuration != RDStatus::OK) {
      RCLCPP_ERROR(get_node()->get_logger(), "No robot description found");
      return controller_interface::CallbackReturn::FAILURE;
  }

  m_elastoplastic_model->clear();
  m_delta_elastoplastic_in_world.clear();

  m_joint_state_interfaces.resize(2);
  m_joint_command_interfaces.resize(2);

  for (const auto & interface : m_allowed_interface_types) {
    auto it = std::ranges::find(m_allowed_interface_types, interface);
    auto idx = std::distance(m_allowed_interface_types.begin(), it);
    if (not controller_interface::get_ordered_interfaces(state_interfaces_, m_parameters.joints, interface,
                                                         m_joint_state_interfaces.at(idx))) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing joints state interfaces: %ld names vs %ld interfaces",
                   m_parameters.joints.size(), m_joint_state_interfaces.at(idx).size());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  auto at_least_one_command_interface {false};
  for (const auto & interface : m_allowed_interface_types) {
    auto it = std::ranges::find(m_allowed_interface_types, interface);
    auto idx = std::distance(m_allowed_interface_types.begin(), it);
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

  if (!m_ft_sensor->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot assing state interface to ft_sensor");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Joint initialization
  std::transform(
    m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.tail(
      m_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return lsi.get_value();
    });
  std::transform(
    m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.tail(
      m_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return lsi.get_value();
    });
  m_qpp.setZero();


  if (m_mobile_base.enabled) {
    m_pub_cmd_vel->on_activate();
  }

  if(m_parameters.debug.log)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Logger level: [DEBUG]");
  }

  if (m_parameters.debug.pub) {
    RCLCPP_WARN(get_node()->get_logger(), "Debug-related publishers: ON");
    m_pub_friction_in_world->on_activate();
    m_pub_wrench_in_world->on_activate();
    m_pub_wrench_in_tool->on_activate();
    m_pub_w->on_activate();
    m_pub_z->on_activate();
    m_pub_cart_vel_error->on_activate();
    m_pub_delta_acceleration->on_activate();
    m_pub_delta_velocity->on_activate();
    m_pub_delta_pose->on_activate();
    m_clik_components_pub->on_activate();
    m_clik_correction_pub->on_activate();
    m_pub_twist_in_world->on_activate();
    m_xp_pub->on_activate();
    m_pub_joint_reference->on_activate();
    m_pub_fk_world_tool->on_activate();
    m_pub_fk_base_tool->on_activate();
    m_pub_next_pose->on_activate();
    m_pub_weights->on_activate();
    m_pub_alfa->on_activate();
    m_interp_pose_pub->on_activate();
    m_interp_twist_pub->on_activate();
    m_computed_pose_pub->on_activate();
    m_computed_twist_pub->on_activate();
  }

  m_rt_buffer_base_odom.initRT(
    nav_msgs::msg::Odometry(
      rosidl_runtime_cpp::MessageInitialization::
      ALL));
  m_rt_buffer_mobile_base_target.initRT(
    geometry_msgs::msg::Twist(
      rosidl_runtime_cpp::MessageInitialization::
      ZERO));
  m_rt_buffer_base_pose_in_world.initRT(
    geometry_msgs::msg::PoseWithCovarianceStamped(
    rosidl_runtime_cpp::MessageInitialization::ALL));

  m_last_odom_msg_time = this->get_node()->get_clock()->now();
  m_last_localization_msg_time = m_last_odom_msg_time;

  m_qp.head<3>().setZero();
  m_q.head<3>().setZero(); // Updated on first cycle
  tf2::fromMsg(m_rt_buffer_base_pose_in_world.readFromRT()->pose.pose, m_T_world_base);

  m_mobile_base.velocity_in_base.setZero();

  m_start_q = m_q;

  // For debug purposes
  m_q_prec.setZero();
  m_qp_prec.setZero();
  m_qpp_prec.setZero();

  m_wrench_in_sensor_prec.setZero();

  // Per AHQP
  m_future_computed_target_T_world_tool = m_chain_world_tool->getTransformation(m_q);
  m_future_computed_target_acc_tool_world_in_world.setZero();
  m_future_computed_target_twist_tool_world_in_world.setZero();
  m_computed_target_T_world_tool = m_chain_world_tool->getTransformation(m_q);
  m_computed_target_acc_tool_world_in_world.setZero();
  m_computed_target_twist_tool_world_in_world.setZero();

  m_logis_prec = 0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElastoplasticController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::transform(
    m_joint_state_interfaces.at(0).begin(),
    m_joint_state_interfaces.at(0).end(),
    m_q.tail(m_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return lsi.get_value();
    });
  std::transform(
    m_joint_state_interfaces.at(1).begin(),
    m_joint_state_interfaces.at(1).end(),
    m_qp.tail(m_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return lsi.get_value();
    });
  m_qpp.setZero();

  if (m_mobile_base.enabled) {
    Eigen::Vector6d empty = Eigen::Vector6d::Zero();
    geometry_msgs::msg::Twist cmd_vel = tf2::toMsg(empty);
    m_pub_cmd_vel->publish(cmd_vel);
  }

  m_elastoplastic_model->clear();
  m_delta_elastoplastic_in_world.clear();

  m_joint_state_interfaces.clear();
  m_joint_command_interfaces.clear();

  m_ft_sensor->release_interfaces();

  m_wrench_in_sensor_prec.setZero();

  return controller_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::CommandInterface> ElastoplasticController::
on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  m_joint_reference_interfaces_size = m_parameters.joints.size() * m_allowed_interface_types.size(); // There must be both position and velocity reference interfaces!

  reference_interfaces_.resize(m_joint_reference_interfaces_size);
  reference_interfaces.reserve(m_joint_reference_interfaces_size);

  size_t idx = 0;
  for (const auto & hwi : m_allowed_interface_types) {
    for (const auto & jnt : m_parameters.joints) {

      reference_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          std::string(get_node()->get_name()), fmt::format("{}/{}", jnt, hwi),
          &reference_interfaces_[idx]));
      ++idx;
    }
  }

  return reference_interfaces;
}


controller_interface::return_type ElastoplasticController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  /* "Joint trajectory available only in chainable mode with joint_trajectory_controller" */

  std::copy(m_q.tail(m_nax).begin(), m_q.tail(m_nax).end(), reference_interfaces_.begin()); // position
  std::fill(std::next(reference_interfaces_.begin(), m_nax), reference_interfaces_.end(), 0.0); // velocity

  return controller_interface::return_type::OK;
}


void ElastoplasticController::get_mobile_base_target_callback(const geometry_msgs::msg::Twist & msg)
{
  // Always from topic
  m_rt_buffer_mobile_base_target.writeFromNonRT(msg);
}

void ElastoplasticController::get_localization_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  m_mobile_base_pose_updated = true;
  m_rt_buffer_base_pose_in_world.writeFromNonRT(msg);
}

void ElastoplasticController::get_odometry_callback(const nav_msgs::msg::Odometry & msg)
{
  m_rt_buffer_base_odom.writeFromNonRT(msg);
}


controller_interface::return_type ElastoplasticController::update_and_write_commands(
  const rclcpp::Time & /*a_time*/, const rclcpp::Duration & /*a_period*/)
{
  rclcpp::Time t_start = get_node()->get_clock()->now();
  // **********
  // ** Read **
  // **********

#define ELASTOPLASTIC__READ_STATES_FROM_INTERFACES_
#ifdef ELASTOPLASTIC__READ_STATES_FROM_INTERFACES
  /* Actual state */
  // Base state

  // TODO: smooth with a filter?
  Eigen::Vector6d twist_base_world_in_world, twist_base_world_in_base;

  // Recover pose from localization
  geometry_msgs::msg::PoseWithCovarianceStamped localization_msg = *(m_rt_buffer_base_pose_in_world.readFromNonRT());
  if (!(
          rclcpp::Time(localization_msg.header.stamp) - m_last_localization_msg_time > std::chrono::duration<double>(m_dt) ||
          rclcpp::Time(localization_msg.header.stamp) - m_last_localization_msg_time < std::chrono::seconds(0)
          )
      )
  {
      Eigen::fromMsg(localization_msg.pose.pose, m_T_world_base);
  }
  m_last_localization_msg_time = localization_msg.header.stamp;

  // Recover twist from odometry
  nav_msgs::msg::Odometry odom_msg = *(m_rt_buffer_base_odom.readFromRT());
  if (rclcpp::Time(odom_msg.header.stamp) - m_last_odom_msg_time > std::chrono::duration<double>(m_dt) ||
      rclcpp::Time(odom_msg.header.stamp) - m_last_odom_msg_time < std::chrono::seconds(0)) {
    twist_base_world_in_base = twist_from_base_velocity(m_mobile_base.velocity_in_base);
  } else {
    Eigen::fromMsg(odom_msg.twist.twist, twist_base_world_in_base);
  }
  m_last_odom_msg_time = odom_msg.header.stamp;

  twist_base_world_in_world = move_from_base_to_world(twist_base_world_in_base);

  // Build state vectors
  if (m_mobile_base.enabled) {
    m_qp.head<3>() = base_velocity_from_twist(twist_base_world_in_world);
    m_q.head<2>() = m_T_world_base.translation().head<2>();
    m_q(2) = Eigen::AngleAxisd(m_T_world_base.linear()).angle();
  }

  // Manipulator State
  std::transform(
    m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.tail(
      m_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return lsi.get_value();
    });
  std::transform(
    m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.tail(
      m_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return lsi.get_value();
    });
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

  /* Cartesian reference */
  // full_position_references.tail(m_nax) = (m_limits.pos_upper + m_limits.pos_lower) / 2;
  full_position_references.tail(m_nax) = m_start_q;
  full_velocity_references.setZero();
  if (!m_interpolator.is_plan_started() && m_interpolator.is_ready()) {
    m_interpolator.start_plan(get_node()->get_clock()->now());
  }
  // TODO: end plan

  Eigen::Vector6d reference_target_twist_tool_world_in_world;
  Eigen::Affine3d reference_target_T_world_tool;
  if (m_interpolator.is_plan_started()) {
    m_interpolator.interpolate(get_node()->get_clock()->now(), reference_target_twist_tool_world_in_world,
                               reference_target_T_world_tool);
  } else {
    reference_target_twist_tool_world_in_world.setZero();
    reference_target_T_world_tool = T_world_tool;
  }
  if (m_mobile_base.enabled) {
    full_velocity_references.head<3>() = utils::base_velocity_from_twist(reference_target_twist_tool_world_in_world);
    full_position_references.head<3>() = m_T_world_base.translation() + full_velocity_references.head<3>() * m_dt;
  }

#endif

  /* FT state */
  std::array<double, 3> ft_force = m_ft_sensor->get_forces();
  std::array<double, 3> ft_torque = m_ft_sensor->get_torques();
  Eigen::Vector6d wrench_sensor_in_sensor(ft_force[0], ft_force[1], ft_force[2], ft_torque[0], ft_torque[1], ft_torque[2]);

  if (wrench_sensor_in_sensor.hasNaN()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *this->get_node()->get_clock(), 1000,
                         "Force sensor contains NaN values. Full measure discarded and replaced with zero");
    wrench_sensor_in_sensor.setZero();
  } else if (wrench_sensor_in_sensor.cwiseAbs().maxCoeff() > 1e20) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *this->get_node()->get_clock(), 1000,
                         "Force sensor contains overflowed values. Full measure discarded and replaced with zero");
    wrench_sensor_in_sensor.setZero();
  }
  Eigen::VectorXd q_start = m_q;
  Eigen::VectorXd qp_start = m_qp;

  m_computed_target_T_world_tool = m_future_computed_target_T_world_tool;
  m_computed_target_twist_tool_world_in_world = m_future_computed_target_twist_tool_world_in_world;
  m_computed_target_acc_tool_world_in_world = m_future_computed_target_acc_tool_world_in_world;


  // ************
  // ** Update **
  // ************
  Eigen::Affine3d T_base_tool = m_chain_base_tool->getTransformation(m_q.tail(m_nax));
  Eigen::Affine3d T_base_sensor = m_chain_base_sensor->getTransformation(m_q.tail(m_nax));
  Eigen::Affine3d T_tool_sensor = T_base_tool.inverse() * T_base_sensor;

  // Wrench deadband
  std::transform(wrench_sensor_in_sensor.begin(), wrench_sensor_in_sensor.end(), m_parameters.wrench.deadband.begin(),
                 wrench_sensor_in_sensor.begin(), [](const double w, const double deadband) {
                   return std::abs(w) > deadband ? utils::sgn(w) * (std::abs(w) - deadband) : 0.0;
                 });

  // Exponential filter
  std::transform(wrench_sensor_in_sensor.begin(), wrench_sensor_in_sensor.end(), m_wrench_in_sensor_prec.begin(),
                 wrench_sensor_in_sensor.begin(), [this](const double w, const double w_prec) {
                   return filters::exponentialSmoothing(w, w_prec, m_parameters.wrench.filter_alfa);
                 });
  m_wrench_in_sensor_prec = wrench_sensor_in_sensor;

  Eigen::Vector6d cart_vel_error_tool_target_in_world;
  cart_vel_error_tool_target_in_world = twist_tool_world_in_world - m_computed_target_twist_tool_world_in_world;

  Eigen::Vector6d wrench_tool_in_tool = rdyn::spatialDualTranformation(wrench_sensor_in_sensor, T_tool_sensor);
  Eigen::Vector6d wrench_tool_in_world = rdyn::spatialRotation(wrench_tool_in_tool, T_world_tool.linear());

  // TODO: verso della differenza?
  Eigen::Vector6d cart_distance_tool_target_in_world;
  Eigen::AngleAxisd tool_target_ang(T_world_tool.linear() * m_computed_target_T_world_tool.linear().transpose());
  cart_distance_tool_target_in_world.head<3>() << T_world_tool.translation() - m_computed_target_T_world_tool.translation();
  cart_distance_tool_target_in_world.tail<3>() << tool_target_ang.axis() * tool_target_ang.angle();

  // Update Impedance model
  Eigen::Vector6d cart_acc_tool_target_in_world;
  std::tie(cart_acc_tool_target_in_world, m_delta_elastoplastic_in_world.velocity, m_delta_elastoplastic_in_world.position) =
    m_elastoplastic_model->update(cart_distance_tool_target_in_world, cart_vel_error_tool_target_in_world, wrench_tool_in_world,
                                  T_world_tool, m_dt);

  // NOTA: a m_delta_elastoplastic_in_world.position bisogna aggiungere target_T_world_tool
  Eigen::Vector6d twist_next_tool_world_in_world =
    m_delta_elastoplastic_in_world.velocity + m_computed_target_twist_tool_world_in_world;
  Eigen::Affine3d T_next_world_tool = rdyn::spatialIntegration(T_world_tool, twist_next_tool_world_in_world, m_dt);

  Eigen::Matrix6Xd J_world_tool_in_world = m_chain_world_tool->getJacobian(m_q);

  ClikData clik_data{.position_references = full_position_references,
                     .velocity_references = full_velocity_references,
                     .twist_tool_world_in_world = twist_tool_world_in_world,
                     .next_twist_tool_world_in_world = twist_next_tool_world_in_world,
                     .T_world_tool = T_world_tool,
                     .next_T_world_tool = T_next_world_tool,
                     .acc_tool_target_in_world = cart_acc_tool_target_in_world,
                     .J_world_tool_in_world = J_world_tool_in_world,
                     .target_T_world_tool = reference_target_T_world_tool,
                     .target_twist_tool_world_in_world = reference_target_twist_tool_world_in_world};

  Eigen::VectorXd qepp = compute_clik(clik_data);

  if (qepp.hasNaN()) {
    RCLCPP_FATAL(get_node()->get_logger(), "Cannot find a solution for the CLIK QP problem");
    RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "\ncart_vel_error_tool_target_in_world\n"
                                                    << cart_vel_error_tool_target_in_world << "\nwrench_tool_in_world\n"
                                                    << wrench_tool_in_world << "\nwrench_tool_in_tool\n"
                                                    << wrench_tool_in_tool << "\nwrench_sensor_in_sensor\n"
                                                    << wrench_sensor_in_sensor << "\ncart_acc_tool_target_in_world\n"
                                                    << cart_acc_tool_target_in_world);
    this->on_deactivate(rclcpp_lifecycle::State());
    throw std::runtime_error("Controller crashed");
    // return controller_interface::return_type::ERROR;
  }
  Eigen::VectorXd qp = m_qp.tail(m_nax) + qepp.tail(m_nax) * m_dt;

  // Scaling due to joint velocity limits
  // double scaling_vel = 1.0;
  // for(size_t idx = 0; idx < m_nax; idx++)
  // {
  //   scaling_vel = std::max(scaling_vel,std::abs(qp(idx))/m_limits.vel(idx));
  // }
  // if(scaling_vel > 1)
  // {
  //   RCLCPP_WARN(get_node()->get_logger(), "Joint velocity greater than limits (ratio = %4f). Applying scaling", scaling_vel);
  //   qepp = clik(twist_next_tool_world_in_world/scaling_vel);
  //   if (qepp.hasNaN())
  //   {
  //     RCLCPP_FATAL(get_node()->get_logger(), "Cannot find a solution for the CLIK QP problem");
  //     //return controller_interface::return_type::ERROR;
  //     this->on_deactivate(rclcpp_lifecycle::State());
  //     throw std::runtime_error("Controller crashed");
  //   }
  // }

  // Ik integration
  m_q += m_qp * m_dt + 0.5 * qepp * std::pow(m_dt, 2);
  m_qp += qepp * m_dt;

  if (m_mobile_base.enabled) {
    Eigen::Vector6d qp_base_in_world = Eigen::Vector6d::Zero();
    qp_base_in_world = utils::twist_from_base_velocity(m_qp.head<3>());

    Eigen::Vector6d qp_base_in_base = rdyn::spatialRotation(qp_base_in_world, m_T_world_base.linear().transpose());
    // qp_base_in_base = qp_base_in_base.unaryExpr([this](double vel) { return std::abs(vel) < M_VELOCITY_TOLLERANCE ? 0.0 :
    // vel;
    // });
    m_mobile_base.velocity_in_base = utils::base_velocity_from_twist(qp_base_in_base);

    // BEGIN - Check Saturation Base
    // If the QP works, this shouldn't be necessary
    for (size_t idx = 0; idx < m_mobile_base.nax(); ++idx) {
      if (std::abs(m_mobile_base.velocity_in_base(idx)) > m_mobile_base.vel_limits(idx)) {
        RCLCPP_WARN_STREAM(this->get_node()->get_logger(),
                           "Saturation of Velocity on base linear direction "
                             << idx << ": " << m_mobile_base.velocity_in_base(idx) << " should be "
                             << utils::sgn(m_mobile_base.velocity_in_base(idx)) * m_mobile_base.vel_limits(idx));
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
      RCLCPP_WARN(get_node()->get_logger(), "Saturation of POSITION on manipulator joint with index %ld", idx);
    }
    if (!utils::almost_equal(dq, m_qp(idx + (m_full_nax - m_nax)))) {
      RCLCPP_WARN(get_node()->get_logger(), "Saturation of VELOCITY on manipulator joint with index %ld", idx);
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
    geometry_msgs::msg::Twist cmd_vel = Eigen::toMsg(utils::twist_from_base_velocity(m_mobile_base.velocity_in_base));

    m_pub_cmd_vel->publish(cmd_vel);

    Eigen::Vector6d base_twist_in_world = utils::twist_from_base_velocity(m_qp.head<3>());
    m_T_world_base = rdyn::spatialIntegration(m_T_world_base, base_twist_in_world, m_dt);
  }

  m_q_prec = m_q;
  m_qp_prec = m_qp;
  m_qpp_prec = qepp;

  // *************
  // ** PUBLISH **
  // *************
  if (m_parameters.debug.pub) {
    auto time_now = this->get_node()->get_clock()->now();
    std_msgs::msg::Float64MultiArray msg_z;
    msg_z.data.resize(6);
    // msg_z.data = std::vector<double>(m_elastoplastic_model->z().data(), m_elastoplastic_model->z().data() +
    // m_elastoplastic_model->z().size());
    msg_z.data.push_back(m_elastoplastic_model->z());
    m_pub_z->publish(msg_z);

    geometry_msgs::msg::WrenchStamped msg_friction_in_world;
    msg_friction_in_world.header.frame_id = m_parameters.frames.map;
    msg_friction_in_world.header.stamp = time_now;
    msg_friction_in_world.wrench.force.x = m_elastoplastic_model->friction_force()[0];
    msg_friction_in_world.wrench.force.y = m_elastoplastic_model->friction_force()[1];
    msg_friction_in_world.wrench.force.z = m_elastoplastic_model->friction_force()[2];
    msg_friction_in_world.wrench.torque.x = m_elastoplastic_model->friction_force()[3];
    msg_friction_in_world.wrench.torque.y = m_elastoplastic_model->friction_force()[4];
    msg_friction_in_world.wrench.torque.z = m_elastoplastic_model->friction_force()[5];
    m_pub_friction_in_world->publish(msg_friction_in_world);

    geometry_msgs::msg::WrenchStamped msg_wrench_in_tool;
    msg_wrench_in_tool.header.frame_id = m_parameters.frames.tool;
    msg_wrench_in_tool.header.stamp = time_now;
    msg_wrench_in_tool.wrench.force.x = wrench_tool_in_tool[0];
    msg_wrench_in_tool.wrench.force.y = wrench_tool_in_tool[1];
    msg_wrench_in_tool.wrench.force.z = wrench_tool_in_tool[2];
    msg_wrench_in_tool.wrench.torque.x = wrench_tool_in_tool[3];
    msg_wrench_in_tool.wrench.torque.y = wrench_tool_in_tool[4];
    msg_wrench_in_tool.wrench.torque.z = wrench_tool_in_tool[5];
    m_pub_wrench_in_tool->publish(msg_wrench_in_tool);

    geometry_msgs::msg::WrenchStamped msg_wrench_in_world;
    msg_wrench_in_world.header.frame_id = m_parameters.frames.map;
    msg_wrench_in_world.header.stamp =    time_now;
    msg_wrench_in_world.wrench.force.x =  wrench_tool_in_world[0];
    msg_wrench_in_world.wrench.force.y =  wrench_tool_in_world[1];
    msg_wrench_in_world.wrench.force.z =  wrench_tool_in_world[2];
    msg_wrench_in_world.wrench.torque.x = wrench_tool_in_world[3];
    msg_wrench_in_world.wrench.torque.y = wrench_tool_in_world[4];
    msg_wrench_in_world.wrench.torque.z = wrench_tool_in_world[5];
    m_pub_wrench_in_world->publish(msg_wrench_in_world);

    m_pub_cart_vel_error->publish(tf2::toMsg(cart_vel_error_tool_target_in_world));
    m_pub_twist_in_world->publish(tf2::toMsg(twist_tool_world_in_world));

    geometry_msgs::msg::Twist acc_delta_msg;
    acc_delta_msg = toMsg(cart_acc_tool_target_in_world);
    m_pub_delta_acceleration->publish(acc_delta_msg);

    geometry_msgs::msg::Twist vel_delta_msg;
    vel_delta_msg = toMsg(m_delta_elastoplastic_in_world.velocity);
    m_pub_delta_velocity->publish(vel_delta_msg);

    geometry_msgs::msg::Twist pos_delta_msg;
    vel_delta_msg = toMsg(m_delta_elastoplastic_in_world.position);
    m_pub_delta_pose->publish(vel_delta_msg);

    Eigen::Vector6d xp_to_pub = m_chain_world_tool->getJacobian(m_q) * m_qp;
    geometry_msgs::msg::Twist xp_msg = Eigen::toMsg(xp_to_pub);
    m_xp_pub->publish(xp_msg);

    geometry_msgs::msg::PoseStamped next_pose_msg;
    next_pose_msg.pose = toMsg(T_next_world_tool);
    next_pose_msg.header.frame_id = m_parameters.frames.map;
    next_pose_msg.header.stamp = time_now;
    m_pub_next_pose->publish(next_pose_msg);

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

    std_msgs::msg::Float64 alfa_msg;
    alfa_msg.data = m_elastoplastic_model->alpha();
    m_pub_alfa->publish(alfa_msg);

    geometry_msgs::msg::PoseStamped cmp_target_T_msg;
    cmp_target_T_msg.pose = tf2::toMsg(m_computed_target_T_world_tool);
    cmp_target_T_msg.header.stamp = time_now;
    cmp_target_T_msg.header.frame_id = m_parameters.frames.base;
    m_computed_pose_pub->publish(cmp_target_T_msg);

    geometry_msgs::msg::Twist cmp_target_twist_msg;
    cmp_target_twist_msg = tf2::toMsg(m_computed_target_twist_tool_world_in_world);
    m_computed_twist_pub->publish(cmp_target_twist_msg);

    geometry_msgs::msg::PoseStamped interp_msg;
    interp_msg.pose = tf2::toMsg(reference_target_T_world_tool);
    interp_msg.header.stamp = time_now;
    interp_msg.header.frame_id = m_parameters.frames.base;
    m_interp_pose_pub->publish(interp_msg);

    geometry_msgs::msg::Twist target_twist_msg;
    target_twist_msg = tf2::toMsg(reference_target_twist_tool_world_in_world);
    m_interp_twist_pub->publish(target_twist_msg);
  }

  rclcpp::Time t_end = get_node()->get_clock()->now();
  std_msgs::msg::Float64 cycle_time_msg;
  cycle_time_msg.data = (double)(t_end - t_start).nanoseconds(); // As [ms]
  m_pub_timing->publish(cycle_time_msg);


  return controller_interface::return_type::OK;
}

Eigen::VectorXd ElastoplasticController::compute_clik(const ClikData& a_data, const bool a_use_qp) {
  Eigen::Vector6d pose_error_tool_world_in_world;
  rdyn::getFrameDistanceQuat(a_data.next_T_world_tool, a_data.T_world_tool, pose_error_tool_world_in_world);
  Eigen::Vector6d velocity_error_tool_world_in_world = a_data.next_twist_tool_world_in_world -
                                                       a_data.twist_tool_world_in_world;
  Eigen::Vector6d acc_non_linear_in_world = m_chain_world_tool->getDTwistNonLinearPartTool(
      m_q,
      m_qp);

  if (m_parameters.debug.pub) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(18);
    std::copy(
        pose_error_tool_world_in_world.begin(),
        pose_error_tool_world_in_world.end(),
        msg.data.begin());
    std::copy(
        velocity_error_tool_world_in_world.begin(),
        velocity_error_tool_world_in_world.end(),
        std::next(msg.data.begin(), 6));
    std::copy(
        acc_non_linear_in_world.begin(),
        acc_non_linear_in_world.end(),
        std::next(msg.data.begin(), 12));
    m_clik_components_pub->publish(msg);
  }

  if (a_use_qp) {
    return compute_clik_as_qp(a_data,
                              pose_error_tool_world_in_world,
                              velocity_error_tool_world_in_world,
                              acc_non_linear_in_world);
  } else {
    return compute_clik_as_inv(a_data,
                               pose_error_tool_world_in_world,
                               velocity_error_tool_world_in_world,
                               acc_non_linear_in_world);
  }
}


Eigen::VectorXd ElastoplasticController::compute_clik_as_inv(const ClikData &a_data, const Eigen::Vector6d &a_position_error,
                                                             const Eigen::Vector6d &a_twist_error,
                                                             const Eigen::Vector6d &a_acc_non_linear) {
  Eigen::VectorXd gradientW =
    m_kp_joint_task * (a_data.position_references - m_q) + m_kv_joint_task * (a_data.velocity_references - m_qp);
  Eigen::Vector6d correction = m_parameters.clik.kv * (a_twist_error) +
                               m_parameters.clik.kp * (a_position_error) -
                               a_acc_non_linear +
                               a_data.acc_tool_target_in_world -
                               a_data.J_world_tool_in_world * gradientW;

  Eigen::MatrixXd Q_half(m_full_nax, m_full_nax);
  Q_half.diagonal() = Eigen::Map<Eigen::VectorXd>(
                          m_parameters.clik.task.weights.data(), m_parameters.clik.task.weights.size())
                          .cwiseSqrt();
  Q_half.diagonal().head<3>() *= (1 + m_parameters.clik.task.alpha_gain * m_elastoplastic_model->alpha());
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd_q(a_data.J_world_tool_in_world * Q_half,
                                                       Eigen::ComputeThinU | Eigen::ComputeThinV);
  return gradientW + Q_half * svd_q.solve(correction);
}


Eigen::VectorXd ElastoplasticController::compute_clik_as_qp(const ClikData &a_data, const Eigen::Vector6d &a_position_error,
                                                            const Eigen::Vector6d &a_twist_error,
                                                            const Eigen::Vector6d &a_acc_non_linear) {
  /* ********
   * ** QP **
   * ********
   *
   *    min       1/2 * ||W*qpp||^2 + ||s||^2
   * [qpp, s]
   *
   * s.t.      J * qpp = xpp_clik + s
   *           qpp_min <= qpp <= qpp_max
   *
   *
   *    min       1/2 * ||qpp_1 + W * V_null * nu||^2 + 1/2 * ||s||^2
   * [qpp2, s]
   *
   * s.t.      A * (qpp + V_null * nu) = b + s
   *           qpp_min <= qpp <= qpp_max
   *
   * ***************
   * ** Null Task **
   * ***************
   *
   * 0.5 * || kp * (qr - q) + kv * (qpr - qp) ||^2 ==> 0.5 * || A qpp - b ||^2
   *
   * A = - 0.5 * kp * dt^2 - kv * dt
   *
   * -b = kp * (qr - q0 - qp0 * dt) + kv * (qpr - qp0)
   *
   * ==> 0.5 * qpp' * A' * A * qpp - b' * A * qpp + ...
   *
   */

  // DEBUG!!!!
  // m_computed_target_acc_tool_world_in_world.setZero();
  // m_computed_target_twist_tool_world_in_world.setZero();
  // DEBUG!!!!

  const unsigned int prb_dim = m_full_nax + M_CARTESIAN_DIM;

  m_W.diagonal().head(m_full_nax) =
    Eigen::Map<Eigen::VectorXd>(m_parameters.clik.task.weights.data(), m_parameters.clik.task.weights.size());
  if (m_mobile_base.enabled) {
    // auto logis = filters::exponentialSmoothing(m_logistic.get(m_mobile_base.velocity_in_base), m_logis_prec, 0.1);
    // m_logis_prec = logis;
    auto logis = 1;
    // m_W.diagonal().head<2>() *= (1.0 + m_parameters.clik.task.alpha_gain * m_elastoplastic_model->alpha());
    m_W.diagonal().head<2>() *= (1.0 + m_parameters.clik.task.alpha_gain * m_elastoplastic_model->alpha() * logis);
    // RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "logis: " << logis);
  }

  // Weighted Least Squares
  Eigen::MatrixXd G(prb_dim, prb_dim);
  Eigen::VectorXd F(prb_dim);
  G.setZero();
  F.setZero();

  double hweight{1};
  auto hnew_task = [](Eigen::MatrixXd& Gin, Eigen::VectorXd& Fin, double& hweightin) -> void {
    constexpr double step{1e-3};
    Gin *= hweightin;
    Fin *= hweightin;
    hweightin *= step;
  };

  /**********************
   ** Task Definitions **
   **********************/

  // Task Cartesian: Minimize difference between the real target acceleration and the computed one
  G.bottomRightCorner<M_CARTESIAN_DIM, M_CARTESIAN_DIM>() += Eigen::Matrix6d::Identity();
  F.tail<M_CARTESIAN_DIM>() += -a_data.acc_tool_target_in_world.transpose();

  // // Task Cartesian: Minimize difference between the real target and the computed one
  // G.bottomRightCorner<M_CARTESIAN_DIM, M_CARTESIAN_DIM>() +=
  // Eigen::Matrix<double, M_CARTESIAN_DIM, M_CARTESIAN_DIM>::Identity() * std::pow(m_dt, 4.0) * 0.25;
  // Eigen::Vector6d target_pose, computed_target_pose;
  // target_pose << a_data.target_T_world_tool.translation(), a_data.target_T_world_tool.linear().eulerAngles(2, 1, 0);
  // computed_target_pose << m_computed_target_T_world_tool.translation(),
  // m_computed_target_T_world_tool.linear().eulerAngles(2, 1, 0);
  // F.tail<M_CARTESIAN_DIM>() +=
  // 0.5 * (target_pose.transpose() * std::pow(m_dt, 2) - computed_target_pose.transpose() * std::pow(m_dt, 2) +
  // m_computed_target_twist_tool_world_in_world.transpose() * std::pow(m_dt, 3));

  // Task Cartesian: Minimize difference between the real target twist and the computed one
  // G.bottomRightCorner<M_CARTESIAN_DIM, M_CARTESIAN_DIM>() +=
  // Eigen::Matrix<double, M_CARTESIAN_DIM, M_CARTESIAN_DIM>::Identity() * std::pow(m_dt, 2.0);
  // F.tail<M_CARTESIAN_DIM>() +=
  // m_dt * (m_computed_target_twist_tool_world_in_world.transpose() - a_data.target_twist_tool_world_in_world.transpose());
  // hnew_task(G, F, hweight);

  // Task: CLIK
  Eigen::VectorXd xpp_clik = (a_acc_non_linear - m_computed_target_acc_tool_world_in_world -
                              m_parameters.clik.kv * (a_twist_error)-m_parameters.clik.kp * (a_position_error));
  // Eigen::Vector6d y_clik =
  // (-a_acc_non_linear + m_parameters.clik.kv * (a_twist_error) + m_parameters.clik.kp * (a_position_error));

  // Eigen::MatrixXd J_aug(M_CARTESIAN_DIM, prb_dim);
  // J_aug << a_data.J_world_tool_in_world * m_W, -Eigen::Matrix6d::Identity();

  G.topLeftCorner(m_full_nax, m_full_nax) +=
    m_W.transpose() * a_data.J_world_tool_in_world.transpose() * a_data.J_world_tool_in_world * m_W;
  // G = J_aug.transpose() * J_aug;
  F.head(m_full_nax) += xpp_clik.transpose() * a_data.J_world_tool_in_world * m_W;
  // F = -y_clik.transpose() * J_aug;
  hnew_task(G, F, hweight);

  // Task: Minimize joint acceleration and weighting
  G.topLeftCorner(m_full_nax, m_full_nax) += Eigen::MatrixXd::Identity(prb_dim, prb_dim);
  hnew_task(G, F, hweight);

  // ********************
  // ** EQ Constraints **
  // ********************
  // int is_z_enabled = static_cast<int>(m_mobile_base.enabled && !m_parameters.mobile_base.enable_z_rotation);
  Eigen::MatrixXd CE(1, prb_dim); //(M_CARTESIAN_DIM + is_z_enabled, prb_dim);
  Eigen::VectorXd ce(1);          //(M_CARTESIAN_DIM + is_z_enabled);
  CE = Eigen::MatrixXd::Identity(1, prb_dim) *
       utils::K_ABS_EPSILON; // No equality constraints: https://github.com/liuq/QuadProgpp/issues/3
  ce.setConstant(utils::K_ABS_EPSILON);
  // CE.leftCols(m_full_nax) << a_data.J_world_tool_in_world * m_W;
  // ce << xpp_clik;

  // Disable rotation around Z-base axis
  // if (is_z_enabled) {
  // CE.bottomRows(1).setZero();
  // CE(M_CARTESIAN_DIM, 2) = 1.0;
  // ce.tail<1>() << 0.0;
  // }

  // ***********************
  // ** DISEQ Constraints **
  // ***********************
  const int n_ineq = 4 * m_full_nax + 2 * m_nax;
  Eigen::MatrixXd CI = Eigen::MatrixXd::Zero(n_ineq, prb_dim);
  Eigen::VectorXd ci(n_ineq);

  // Inequality dimensions: {velocity     [2 * full_nax],
  //                         acceleration [2 * full_nax],
  //                         position     [2 * nax]}
  // Limits order: {x > x_min,
  //                x < x_max}, x = {qp, qpp, q}
  std::vector<size_t> ineq_idxs {0, 2*m_full_nax, 4*m_full_nax};

  // Velocity
  CI.block(0, 0, m_full_nax, m_full_nax) << Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  CI.block(m_full_nax, 0, m_full_nax, m_full_nax) << -Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;

  ci.segment(ineq_idxs[0] + m_mobile_base.nax(), m_nax) = (m_qp.tail(m_nax) + m_limits.vel);
  ci.segment(ineq_idxs[0] + m_full_nax + m_mobile_base.nax(), m_nax) = (m_limits.vel - m_qp.tail(m_nax));

  // Acceleration
  CI.block(ineq_idxs[1], 0, m_full_nax, m_full_nax) << Eigen::MatrixXd::Identity(
      m_full_nax,
      m_full_nax);
  CI.block(ineq_idxs[1] + m_full_nax, 0, m_full_nax, m_full_nax) << -Eigen::MatrixXd::Identity(
      m_full_nax, m_full_nax);

  ci.segment(ineq_idxs[1] + m_mobile_base.nax(), m_nax) = m_limits.acc;
  ci.segment(ineq_idxs[1] + m_full_nax + m_mobile_base.nax(), m_nax) = m_limits.acc;

  // Positions
  CI.block(ineq_idxs[2], m_full_nax - m_nax, m_nax, m_nax)
      << Eigen::MatrixXd::Identity(m_nax, m_nax) * 0.5 * m_dt * m_dt;
  CI.block(ineq_idxs[2] + m_nax, m_full_nax - m_nax, m_nax, m_nax)
      << -Eigen::MatrixXd::Identity(m_nax, m_nax) * 0.5 * m_dt * m_dt;

  ci.segment(ineq_idxs[2], m_nax) = (m_q.tail(m_nax) + m_qp.tail(m_nax) * m_dt) - m_limits.pos_lower;
  ci.segment(
      ineq_idxs[2] + m_nax,
      m_nax) = m_limits.pos_upper - (m_q.tail(m_nax) + m_qp.tail(m_nax) * m_dt);

  // Move base limits to world
  if (m_mobile_base.enabled) {
    // Velocity
    Eigen::Vector6d max_vel_base_in_world = utils::twist_from_base_velocity(m_mobile_base.vel_limits);
    Eigen::Vector6d max_vel_base_in_base = rdyn::spatialRotation(max_vel_base_in_world, m_T_world_base.linear().transpose());
    Eigen::Vector3d max_vel_base = utils::base_velocity_from_twist(max_vel_base_in_base);
    ci.segment<3>(ineq_idxs[0]) << m_qp.head<3>() + max_vel_base;
    ci.segment<3>(ineq_idxs[0] + m_full_nax) << max_vel_base - m_qp.head<3>();

    // Acceleration
    Eigen::Vector6d max_acc_base_in_world = utils::twist_from_base_velocity(m_mobile_base.acc_limits);
    Eigen::Vector6d max_acc_base_in_base = rdyn::spatialRotation(max_acc_base_in_world, m_T_world_base.linear().transpose());
    ci.segment<3>(ineq_idxs[1]) << utils::base_velocity_from_twist(max_acc_base_in_base);
    ci.segment<3>(ineq_idxs[1] + m_full_nax) << utils::base_velocity_from_twist(max_acc_base_in_base);
  }

  CI.leftCols(m_full_nax) *= m_W;

  Eigen::VectorXd first_sol(prb_dim);
  Eigen::MatrixXd G_for_debug(G); // G is copied since will be modified by the solver
  m_eiquadprog.reset(CE.cols(), CE.rows(), CI.rows());
  // Attentione: eiquadprog-fast non richiede di trasporre le matrici dei vincoli, al contrario di eiquadprog
  eiquadprog::solvers::EiquadprogFast_status solver_status = m_eiquadprog.solve_quadprog(G, F, CE, ce, CI, ci, first_sol);

  if (solver_status != eiquadprog::solvers::EiquadprogFast_status::EIQUADPROG_FAST_OPTIMAL) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Problem unfeasible. Solver status: " << solver_status);
    RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Dump: "
                                                    << "\nacc_non_linear:\n"
                                                    << a_acc_non_linear << "\na_data.acc_tool_target_in_world:\n"
                                                    << a_data.acc_tool_target_in_world << "\na_twist_error:\n"
                                                    << a_twist_error << "\nposition_error:\n"
                                                    << a_position_error << "\nT_next_world_tool\n"
                                                    << a_data.next_T_world_tool.matrix() << "\nT_world_tool\n"
                                                    << a_data.T_world_tool.matrix() << "\nnext_twist_tool_world_in_world\n"
                                                    << a_data.next_twist_tool_world_in_world << "\ntwist_tool_world_in_world\n"
                                                    << a_data.twist_tool_world_in_world
                                                    << "\nm_delta_elastoplastic_in_world.velocity\n"
                                                    << m_delta_elastoplastic_in_world.velocity << "\ndt\n");
    RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Dump: "
                                                    << "## m_W ## " << m_W.diagonal() << "## G ## " << G_for_debug << "\n## F ##"
                                                    << F.transpose() << "\n## CE ## " << CE << "\n ## ce ## " << ce.transpose()
                                                    << "\n## CI ## " << CI << "\n## ci ##" << ci.transpose());
    return Eigen::VectorXd::Constant(1, 1, std::nan("0"));
  }

  if (first_sol.hasNaN()) {
    RCLCPP_ERROR(get_node()->get_logger(), "NaN in the solution!");
    RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Dump: "
                                                    << "\n## first round sol [qpp(" << m_full_nax << "), slack("
                                                    << prb_dim - m_full_nax << ")]##\n"
                                                    << first_sol.transpose() << "\n## first round ret ##\n"
                                                    << solver_status << "## G ## " << G_for_debug << "\n## F ##" << F.transpose()
                                                    << "\n## CE ## " << CE << "\n ## ce ## " << ce.transpose() << "\n## CI ## "
                                                    << CI << "\n## ci ##" << ci.transpose());
    return Eigen::VectorXd::Constant(1, 1, std::nan("0"));
  }

  Eigen::VectorXd qpp_1(m_W * first_sol.head(m_full_nax));
  // Eigen::VectorXd slack_1(first_sol.tail(M_CARTESIAN_DIM));
  m_future_computed_target_acc_tool_world_in_world = first_sol.tail<M_CARTESIAN_DIM>();
  m_future_computed_target_twist_tool_world_in_world =
    m_computed_target_twist_tool_world_in_world + m_future_computed_target_acc_tool_world_in_world * m_dt;
  m_future_computed_target_T_world_tool =
    rdyn::spatialIntegration(m_computed_target_T_world_tool, m_future_computed_target_twist_tool_world_in_world, m_dt);

  // Null space
  Eigen::MatrixXd At(m_full_nax, m_full_nax), As(m_full_nax, m_full_nax);
  Eigen::VectorXd bt(m_full_nax), bs(m_full_nax);

  // * Velocity
  At = -m_kv_joint_task * Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  bt = -m_kv_joint_task * (a_data.velocity_references - m_qp);
  As = At;
  bs = bt;

  // * Position
  At = -m_kp_joint_task * 0.5 * Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * std::pow(m_dt, 2);
  bt = -m_kp_joint_task * (a_data.position_references - (m_q + m_qp * m_dt));
  As += At;
  bs += bt;

  Eigen::JacobiSVD<Eigen::MatrixXd> J_svd(a_data.J_world_tool_in_world * m_W, Eigen::ComputeFullV);
  const unsigned int null_space_dim = m_full_nax - J_svd.nonzeroSingularValues();
  const unsigned int prb_dim_2 = null_space_dim + m_full_nax;
  Eigen::MatrixXd V_null(m_full_nax, null_space_dim);
  Eigen::MatrixXd G2(prb_dim_2, prb_dim_2);
  Eigen::VectorXd F2(prb_dim_2);
  Eigen::MatrixXd CE2(m_full_nax, prb_dim_2), CI2(CI.rows(), prb_dim_2);
  Eigen::VectorXd ce2(m_full_nax), ci2(ci);

  V_null << J_svd.matrixV().rightCols(null_space_dim);
  G2.setZero();
  G2.topLeftCorner(null_space_dim, null_space_dim) << V_null.transpose() * V_null;
  G2.bottomRightCorner(m_full_nax, m_full_nax).setIdentity();
  F2.setZero();
  F2.head(null_space_dim) = qpp_1.transpose() * V_null;
  CE2.leftCols(null_space_dim) << As * m_W * V_null;
  CE2.rightCols(m_full_nax) = -m_W;
  ce2 << -bs + As * qpp_1;
  CI2.setZero();
  CI2.leftCols(null_space_dim) << m_W * V_null * m_dt, -m_W * V_null * m_dt, m_W * V_null, -m_W * V_null,
    m_W.bottomRightCorner(m_nax, m_nax) *
      (Eigen::MatrixXd(m_nax, m_full_nax) << Eigen::MatrixXd::Zero(m_nax, m_mobile_base.nax()),
       Eigen::MatrixXd::Identity(m_nax, m_nax))
        .finished() *
      V_null * 0.5 * m_dt * m_dt,
    -m_W.bottomRightCorner(m_nax, m_nax) *
      (Eigen::MatrixXd(m_nax, m_full_nax) << Eigen::MatrixXd::Zero(m_nax, m_mobile_base.nax()),
       Eigen::MatrixXd::Identity(m_nax, m_nax))
        .finished() *
      V_null * 0.5 * m_dt * m_dt;

  ci2.segment(0, m_full_nax) += qpp_1 * m_dt;               // >= min
  ci2.segment(m_full_nax, m_full_nax) -= qpp_1 * m_dt;      // <= max
  ci2.segment(2 * m_full_nax, m_full_nax) += qpp_1;  // >= min
  ci2.segment(3 * m_full_nax, m_full_nax) -= qpp_1;  // <= max
  ci2.segment(4 * m_full_nax, m_nax) += 0.5 * qpp_1.tail(m_nax) * m_dt * m_dt;       // >= min
  ci2.segment(4 * m_full_nax + m_nax, m_nax) -= 0.5 * qpp_1.tail(m_nax) * m_dt * m_dt; // <= max

  Eigen::VectorXd second_sol(prb_dim_2);
  m_eiquadprog.reset(prb_dim_2, CE2.rows(), CI2.rows());
  // Attentione: eiquadprog-fast non richiede di trasporre le matrici dei vincoli
  eiquadprog::solvers::EiquadprogFast_status solver_status_2 =
    m_eiquadprog.solve_quadprog(G2, F2, CE2, ce2, CI2, ci2, second_sol);

  Eigen::VectorXd return_qpp(m_full_nax);

  if (solver_status_2 == eiquadprog::solvers::EiquadprogFast_status::EIQUADPROG_FAST_OPTIMAL) {
    return_qpp = qpp_1 + m_W * V_null * second_sol.head(null_space_dim);
  } else {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1,
                                "Discarding second task due to infeasibility. Solver status: " << solver_status_2);
    return_qpp = qpp_1;
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "Controller tick");

  return return_qpp;
}


// Questa o quella esatta da rdyn?
// std::array<Eigen::MatrixXd, 6> ElastoplasticController::update_hessian(const Eigen::Matrix6Xd& jacobian, const Eigen::VectorXd& q)
// {
//   for(int idx = 0; idx < jacobian.rows(); idx++)
//   {
//     Eigen::VectorXd yt = jacobian.row(idx) - bfgs_prev.jacobian_p.row(idx);
//     Eigen::VectorXd st = q - m_old_q;
//     double rho = 1 / (yt.transpose() * st);
//     m_hessian.at(idx) =
//         (Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) - rho * st * yt.transpose())
//             * bfgs_prev.hessian_p.at(idx)
//         * (Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) - rho * yt * st.transpose())
//         + rho * st * st.transpose();
//   }
//   bfgs_prev.hessian_p = m_hessian;
//   bfgs_prev.jacobian_p = jacobian;
// }

} // namespace elastoplastic

PLUGINLIB_EXPORT_CLASS(
  elastoplastic::ElastoplasticController,
  controller_interface::ChainableControllerInterface);
