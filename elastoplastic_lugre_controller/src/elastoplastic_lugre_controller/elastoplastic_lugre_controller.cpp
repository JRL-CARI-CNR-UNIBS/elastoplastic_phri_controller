#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <urdfdom_headers/urdf_model/model.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <fmt/color.h>

#include <eigen_matrix_utils/eiquadprog.hpp>


namespace elastoplastic {


controller_interface::CallbackReturn ElastoplasticController::on_init()
{
  m_param_listener = std::make_shared<elastoplastic_controller::ParamListener>(this->get_node());
  return controller_interface::CallbackReturn::SUCCESS;
}


void ElastoplasticController::configure_after_robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if(m_robot_description_configuration == RDStatus::OK)
  {
    RCLCPP_DEBUG(get_node()->get_logger(), "New robot_description ignored");
    return;
  }

  std::string robot_description = msg->data;
  if(robot_description.empty())
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Missing robot_description by controller_manager");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  else
  {
    RCLCPP_INFO(this->get_node()->get_logger(), "Robot description obtained correctly");
  }

  if(not get_node()->has_parameter("robot_description"))
  {
    get_node()->declare_parameter("robot_description", robot_description);
  }

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(robot_description);
  if(not urdf_model)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot create URDF model from robot_description provided by controller_manager");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "URDF model created");

  Eigen::Vector3d gravity({m_parameters.gravity.at(0), m_parameters.gravity.at(1), m_parameters.gravity.at(2)});
  m_chain_base_tool   = rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.tool, gravity);
  m_chain_base_sensor = rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.sensor, gravity);
  if(not m_chain_base_tool)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot create rdyn chain from base (%s) to tool (%s)",m_parameters.frames.base.c_str(), m_parameters.frames.tool.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  if(not m_chain_base_sensor)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot create rdyn chain from base (%s) to sensor (%s)",m_parameters.frames.base.c_str(), m_parameters.frames.sensor.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  if (m_float_base.enabled)
  {
    const std::string float_base_urdf = R"(<?xml version='1.0'?>
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
    urdf::ModelInterfaceSharedPtr float_base_model = urdf::parseURDF(float_base_urdf);
    m_chain_world_base = rdyn::createChain(*float_base_model, "x_base", "mount_link", {0,0,-9.806});

    m_chain_world_tool = rdyn::joinChains(m_chain_world_base, m_chain_base_tool);
  }
  else
  {
    m_chain_world_tool = m_chain_base_tool;
  }

  if(not m_chain_world_tool)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot create rdyn chain from world to tool (%s)", m_parameters.frames.tool.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "RDyn chains created");

  m_limits.pos_upper.resize(m_nax);
  m_limits.pos_lower.resize(m_nax);
  m_limits.vel.resize(m_nax);
  m_limits.acc.resize(m_nax);

  for(size_t ax = 0; ax < m_nax; ++ax)
  {
    m_limits.pos_upper(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->upper;
    m_limits.pos_lower(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->lower;

    if ((m_limits.pos_upper(ax)==0) && (m_limits.pos_lower(ax)==0))
    {
      m_limits.pos_upper(ax)= std::numeric_limits<double>::infinity();
      m_limits.pos_lower(ax)=-std::numeric_limits<double>::infinity();
      RCLCPP_WARN(this->get_node()->get_logger(), "Upper and Lower limits are both equal to 0, set +/- infinity");
    }

    m_limits.vel(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->velocity;
    m_limits.acc(ax) = 10 * m_limits.vel(ax);
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Kinematics limits: OK");

  std::string what;
  m_joint_names.resize(m_parameters.joints.size() + m_float_base.nax());
  //std::merge(m_float_base.base_joint_names().begin(), m_float_base.base_joint_names().end(), m_parameters.joints.begin(), m_parameters.joints.end(), m_joint_names.begin());
  std::ranges::copy(m_float_base.base_joint_names(), m_joint_names.begin());
  std::copy(m_parameters.joints.begin(), m_parameters.joints.end(), std::next(m_joint_names.begin(), m_float_base.nax()));
  m_chain_base_tool  ->setInputJointsName(m_parameters.joints, what);
  m_chain_base_sensor->setInputJointsName(m_parameters.joints, what);
  m_chain_world_tool->setInputJointsName(m_joint_names, what);

  m_robot_description_configuration = RDStatus::OK;
}


controller_interface::CallbackReturn ElastoplasticController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  m_parameters = m_param_listener->get_params();


  m_elastoplastic_model = std::make_unique<ElastoplasticModel>(get_model_data());

  m_float_base.enabled = m_parameters.floating_base.enabled;

  m_full_nax = m_float_base.enabled ? m_parameters.joints.size() + m_float_base.nax() : m_parameters.joints.size();
  m_nax = m_parameters.joints.size();
  m_q.resize(m_full_nax);
  m_qp.resize(m_full_nax);
  m_qpp.resize(m_full_nax);

  m_old_q.resize(m_full_nax);
  m_old_qp.resize(m_full_nax);

  if(m_parameters.clik.task.weights.size() != m_full_nax)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "weights size is not %d", m_full_nax);
    return controller_interface::CallbackReturn::FAILURE;
  }

  if(std::ranges::min(m_parameters.impedance.inertia) < 0)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Inertia has negative values!");
    return controller_interface::CallbackReturn::FAILURE;
  }

  using namespace std::placeholders;
  m_sub_fb_target = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(m_parameters.floating_base.input_target_topic, 1, std::bind(&ElastoplasticController::get_fb_target_callback, this, _1));
  m_sub_base_odometry = this->get_node()->create_subscription<nav_msgs::msg::Odometry>(m_parameters.floating_base.odom, 1, std::bind(&ElastoplasticController::get_odometry_callback, this, _1));

  m_ft_sensor = std::make_unique<semantic_components::ForceTorqueSensor>(m_parameters.ft_sensor_name);

  m_pub_cmd_vel = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 1);

  m_pub_z =                this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray> ("~/z", 10);
  m_pub_w =                this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray> ("~/w", 10);
  m_pub_friction_in_world = this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("~/friction_in_world", 10);
  m_pub_wrench_in_world =   this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("~/wrench_in_world", 10);
  m_pub_cart_vel_error =   this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/cart_vel_error", 10);
  m_pub_delta_acceleration =   this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/delta/acceleration", 10);
  m_pub_delta_velocity = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/delta/velocity", 10);
  m_pub_delta_pose = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/delta/pose", 10);
  m_clik_components_pub = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/clik_components",rclcpp::QoS(10).durability_volatile().reliable());
  m_clik_correction_pub = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/clik_correction", rclcpp::QoS(10).durability_volatile().reliable());
  m_pub_twist_in_world = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/twist_in_world", 10);
  m_xp_pub = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/new_twist_in_world", 10);
  m_pub_joint_reference = this->get_node()->create_publisher<sensor_msgs::msg::JointState>("~/joint_references", 10);

  m_pub_next_pose = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/next_pose", 10);

  m_state_interfaces_names.reserve(m_allowed_interface_types.size());
  m_command_interfaces_names.reserve(m_allowed_interface_types.size());

  for(const auto& interface : m_allowed_interface_types)
  {
    auto it = std::ranges::find(m_parameters.state_interfaces, interface);
    if(it == m_parameters.state_interfaces.end())
    {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Missing State interfaces from parameters");
      return controller_interface::CallbackReturn::FAILURE;
    }
    else
    {
      m_state_interfaces_names.push_back(*it);
    }

    it = std::ranges::find(m_parameters.command_interfaces, interface);
    if(it != m_parameters.command_interfaces.end())
    {
      m_command_interfaces_names.push_back(*it);
      RCLCPP_INFO(get_node()->get_logger(), "Command interface name: %s", (*it).c_str());
    }
  }
  if(m_command_interfaces_names.empty())
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Missing Command interfaces from parameters");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Robot description-related operations
  if(get_node()->has_parameter("robot_description"))
  {
    std_msgs::msg::String::SharedPtr rd = std::make_shared<std_msgs::msg::String>();
    rd->data = get_node()->get_parameter("robot_description").as_string();
    configure_after_robot_description_callback(rd);
  }
  else
  {
    rclcpp::QoS qos(5);
    qos.transient_local();
    m_sub_robot_description = get_node()->create_subscription<std_msgs::msg::String>("/robot_description", qos, std::bind(&ElastoplasticController::configure_after_robot_description_callback, this, std::placeholders::_1));
    m_robot_description_configuration = RDStatus::EMPTY;
  }

  std::ranges::fill(m_used_command_interfaces, false);
  if(std::ranges::find(m_command_interfaces_names, m_allowed_interface_types[0]) != m_command_interfaces_names.end())
  {
    m_used_command_interfaces.at(0) = true;
  }
  if(std::ranges::find(m_command_interfaces_names, m_allowed_interface_types[1]) != m_command_interfaces_names.end())
  {
    m_used_command_interfaces.at(1) = true;
  }

  m_dt = m_parameters.dt;

  m_kp_last_task = m_parameters.clik.task.last.kp;
  m_kv_last_task = m_parameters.clik.task.last.kv;

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration ElastoplasticController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interface_configuration.names.reserve(m_parameters.joints.size() * m_allowed_interface_types.size() + 6);

  for(const auto& jnt : m_parameters.joints)
  {
    state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION));
  }
  for(const auto& jnt : m_parameters.joints)
  {
    state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
  }

  std::vector<std::string> ft_interfaces = m_ft_sensor->get_state_interface_names();
  state_interface_configuration.names.insert(state_interface_configuration.names.end(), ft_interfaces.begin(), ft_interfaces.end());

  return state_interface_configuration;
}


controller_interface::InterfaceConfiguration ElastoplasticController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interface_configuration;
  command_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interface_configuration.names.reserve(m_parameters.joints.size() * m_command_interfaces_names.size());
  if(std::ranges::find(m_command_interfaces_names, m_allowed_interface_types[0]) != m_command_interfaces_names.end())
  {
    for(const auto& jnt : m_parameters.joints)
    {
      command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, m_allowed_interface_types[0]));
    }
  }
  if(std::ranges::find(m_command_interfaces_names, m_allowed_interface_types[1]) != m_command_interfaces_names.end())
  {
    for(const auto& jnt : m_parameters.joints)
    {
      command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, m_allowed_interface_types[1]));
    }
  }

  return command_interface_configuration;
}


controller_interface::CallbackReturn ElastoplasticController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto t_start = get_node()->get_clock()->now();
  do{
    get_node()->get_clock()->sleep_for(std::chrono::milliseconds(100));
  } while(!ready_for_activation() && get_node()->get_clock()->now() - t_start < std::chrono::seconds(5));
  if(m_robot_description_configuration != RDStatus::OK)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No robot description found");
    return controller_interface::CallbackReturn::FAILURE;
  }

  m_elastoplastic_model->clear();
  m_delta_elastoplastic_in_world.clear();

  m_joint_state_interfaces.resize(2);
  m_joint_command_interfaces.resize(2);

  for(const auto& interface : m_allowed_interface_types)
  {
    auto it = std::ranges::find(m_allowed_interface_types, interface);
    auto idx = std::distance(m_allowed_interface_types.begin(), it);
    if(not controller_interface::get_ordered_interfaces(state_interfaces_, m_parameters.joints, interface, m_joint_state_interfaces.at(idx)))
    {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Missing joints state interfaces: %ld names vs %ld interfaces", m_parameters.joints.size(), m_joint_state_interfaces.at(idx).size());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  auto at_least_one_command_interface {false};
  for(const auto& interface : m_allowed_interface_types)
  {
    auto it = std::ranges::find(m_allowed_interface_types, interface);
    auto idx = std::distance(m_allowed_interface_types.begin(), it);
    if(not controller_interface::get_ordered_interfaces(command_interfaces_, m_parameters.joints, interface, m_joint_command_interfaces.at(idx)))
    {
      continue;
    }
    at_least_one_command_interface = true;
  }
  if(!at_least_one_command_interface)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Missing at least one joints command interface");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if(!m_ft_sensor->assign_loaned_state_interfaces(state_interfaces_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot assing state interface to ft_sensor");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Joint initialization
  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.tail(m_nax).begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.tail(m_nax).begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });
  m_qpp.setZero();


  if (m_parameters.floating_base.enabled)
  {
    m_pub_cmd_vel->on_activate();
  }

  if (m_parameters.debug)
  {
    RCLCPP_WARN(this->get_node()->get_logger(), "Debug publishers: ON");
    m_pub_friction_in_world->on_activate();
    m_pub_wrench_in_world->on_activate();
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
  }

  m_rt_buffer_base_odom.initRT(nav_msgs::msg::Odometry(rosidl_runtime_cpp::MessageInitialization::ALL));
  m_rt_buffer_fb_target.initRT(geometry_msgs::msg::Twist(rosidl_runtime_cpp::MessageInitialization::ZERO));

  m_last_odom_msg_time = this->get_node()->get_clock()->now();
  m_T_world_base = Eigen::Affine3d::Identity();


  m_qp.head<3>().setZero();
  m_q.head<3>().setZero(); // TODO: Da cambiare
  m_T_world_tool_initial = m_chain_world_tool->getTransformation(m_q);

  m_float_base.velocity_in_base.setZero();

  m_initial_q = m_q;
  m_initial_qp = Eigen::VectorXd::Zero(m_q.size());

  m_old_q.setZero();
  m_old_qp.setZero();

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn ElastoplasticController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.tail(m_nax).begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.tail(m_nax).begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });
  m_qpp.setZero();

  m_elastoplastic_model->clear();
  m_delta_elastoplastic_in_world.clear();

  m_joint_state_interfaces.clear();
  m_joint_command_interfaces.clear();

  m_ft_sensor->release_interfaces();

  return controller_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::CommandInterface> ElastoplasticController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  m_joint_reference_interfaces_size = m_parameters.joints.size() * m_allowed_interface_types.size(); // There must be both position and velocity reference interfaces!

  reference_interfaces_.resize(m_joint_reference_interfaces_size);
  reference_interfaces.reserve(m_joint_reference_interfaces_size);

  size_t idx = 0;
  for(const auto& hwi : m_allowed_interface_types)
  {
    for(const auto& jnt : m_parameters.joints)
    {

      reference_interfaces.emplace_back(hardware_interface::CommandInterface(std::string(get_node()->get_name()), fmt::format("{}/{}", jnt, hwi),
                                                                          &reference_interfaces_[idx]));
      ++idx;
    }
  }

  return reference_interfaces;
}


controller_interface::return_type ElastoplasticController::update_reference_from_subscribers()
{
  /* "Joint trajectory available only in chainable mode with joint_trajectory_controller" */

  std::copy(m_q.tail(m_nax).begin(), m_q.tail(m_nax).end(), reference_interfaces_.begin()); // position
  std::fill(std::next(reference_interfaces_.begin(), m_nax), reference_interfaces_.end(), 0.0); // velocity

  return controller_interface::return_type::OK;
}


void ElastoplasticController::get_fb_target_callback(const geometry_msgs::msg::Twist& msg)
{
  // Always from topic
  m_rt_buffer_fb_target.writeFromNonRT(msg);
}


void ElastoplasticController::get_odometry_callback(const nav_msgs::msg::Odometry& msg)
{
  m_rt_buffer_base_odom.writeFromNonRT(msg);
}


controller_interface::return_type ElastoplasticController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // **********
  // ** Read **
  // **********

  // Utility
  auto move_from_base_to_world = [this](const Eigen::Vector6d& v){
    return rdyn::spatialRotation(v, m_T_world_base.linear());
  };

#if 0
  /* Actual state */
  // Base state
  nav_msgs::msg::Odometry odom_msg = *(m_rt_buffer_base_odom.readFromRT());
  Eigen::Vector6d twist_base_world_in_world, twist_base_world_in_base;

  // TODO: smooth with a filter?
  // if(rclcpp::Time(odom_msg.header.stamp) >= m_last_odom_msg_time)
  // {
  //   Eigen::fromMsg(odom_msg.pose.pose, m_T_world_base);
  //   Eigen::fromMsg(odom_msg.twist.twist, twist_base_world_in_base);
  //   m_last_odom_msg_time = odom_msg.header.stamp;
  // }
  // else
  // {
    twist_base_world_in_base = m_float_base.velocity_in_base;
  // }

  twist_base_world_in_world = move_from_base_to_world(twist_base_world_in_base);

  // Build state vectors
  if(m_float_base.enabled)
  {
    m_qp.head<3>() = base_velocity_from_twist(twist_base_world_in_world);
    m_q.head<2>() = m_T_world_base.translation().head<2>();
    m_q(2) = Eigen::AngleAxisd(m_T_world_base.linear()).angle();
  }

  // Manipulator State
  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.tail(m_nax).begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.tail(m_nax).begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });
#endif
  Eigen::Affine3d T_world_tool = m_chain_world_tool->getTransformation(m_q); // m_T_world_base * T_base_tool;
  Eigen::Vector6d twist_tool_world_in_world = m_chain_world_tool->getJacobian(m_q) * m_qp;

  /* Target */
  // Target base
  Eigen::Vector6d target_twist_base_world_in_base, target_twist_base_world_in_world;
  Eigen::fromMsg(*(m_rt_buffer_fb_target.readFromRT()), target_twist_base_world_in_base);
  target_twist_base_world_in_world = move_from_base_to_world(target_twist_base_world_in_base);

  Eigen::Affine3d T_target_world_base = rdyn::spatialIntegration(m_T_world_base, target_twist_base_world_in_world, m_dt);

  // Target manipulator
  Eigen::VectorXd joint_position_references(m_nax);
  joint_position_references = Eigen::Map<Eigen::VectorXd>(reference_interfaces_.data(), m_parameters.joints.size());
  Eigen::VectorXd joint_velocity_references(m_nax);
  joint_velocity_references = Eigen::Map<Eigen::VectorXd>(std::next(reference_interfaces_.data(), m_nax), m_nax);
  Eigen::VectorXd full_position_references(m_full_nax), full_velocity_references(m_full_nax);
  full_position_references << T_target_world_base.translation().head<2>(),
                              Eigen::AngleAxisd(T_target_world_base.linear()).angle(),
                              joint_position_references;
  full_velocity_references << base_velocity_from_twist(target_twist_base_world_in_world),
                              joint_velocity_references;

  Eigen::Vector6d target_twist_tool_world_in_world = m_chain_world_tool->getJacobian(full_position_references) * full_velocity_references;
  // Eigen::Vector6d target_twist_tool_world_in_world = m_chain_world_tool->getJacobian(m_q) * full_velocity_references;

  /* FT state */
  std::array<double, 3> ft_force = m_ft_sensor->get_forces();
  std::array<double, 3> ft_torque = m_ft_sensor->get_torques();
  Eigen::Vector6d wrench_sensor_in_sensor(ft_force[0], ft_force[1], ft_force[2],
                                          ft_torque[0], ft_torque[1], ft_torque[2]);
  if(wrench_sensor_in_sensor.hasNaN())
  {
    RCLCPP_WARN_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1000, "Force sensor contains NaN values. Full measure discarded and replaced with zero");
    wrench_sensor_in_sensor.setZero();
  }

  Eigen::VectorXd q_start = m_q;
  Eigen::VectorXd qp_start = m_qp;

  // ************
  // ** Update **
  // ************
  Eigen::Vector6d cart_vel_error_tool_target_in_world = twist_tool_world_in_world - target_twist_tool_world_in_world;

  Eigen::Affine3d T_base_tool = m_chain_base_tool->getTransformation(m_q.tail(m_nax));
  Eigen::Affine3d T_base_sensor = m_chain_base_sensor->getTransformation(m_q.tail(m_nax));
  Eigen::Affine3d T_tool_sensor = T_base_tool.inverse() * T_base_sensor;
  wrench_sensor_in_sensor.head<3>() = wrench_sensor_in_sensor.head<3>().unaryExpr([this](double w){return std::abs(w) > m_parameters.wrench_deadband.at(0)? w : 0.0;});
  wrench_sensor_in_sensor.tail<3>() = wrench_sensor_in_sensor.tail<3>().unaryExpr([this](double w){return std::abs(w) > m_parameters.wrench_deadband.at(1)? w : 0.0;});
  Eigen::Vector6d wrench_tool_in_tool = rdyn::spatialDualTranformation(wrench_sensor_in_sensor, T_tool_sensor);

  Eigen::Vector6d cart_acc_tool_target_in_tool;
  Eigen::Vector6d cart_vel_error_tool_target_in_tool = rdyn::spatialRotation(cart_vel_error_tool_target_in_world, T_world_tool.linear().transpose());
  cart_acc_tool_target_in_tool.head<3>() = m_elastoplastic_model->update(cart_vel_error_tool_target_in_tool.head<3>(),
                                                                    wrench_tool_in_tool.head<3>(),
                                                                    m_dt);

  Eigen::Vector6d cart_acc_tool_target_in_world = Eigen::Vector6d::Zero();
  cart_acc_tool_target_in_world.head<3>() = rdyn::spatialRotation(cart_acc_tool_target_in_tool, T_world_tool.rotation()).head<3>();

  m_delta_elastoplastic_in_world.position += m_delta_elastoplastic_in_world.velocity * m_dt + 0.5 * cart_acc_tool_target_in_world * std::pow(m_dt, 2.0); // Used only for debug
  m_delta_elastoplastic_in_world.velocity += cart_acc_tool_target_in_world * m_dt;

  Eigen::Vector6d twist_next_tool_world_in_world = m_delta_elastoplastic_in_world.velocity + target_twist_tool_world_in_world;
  Eigen::Affine3d T_next_world_tool = rdyn::spatialIntegration(T_world_tool, twist_next_tool_world_in_world, m_dt);

  // Proietta nelle direzioni ortogonali alla traiettoria?
  // Scala la traiettoria (in funzione del della differenza vel_trj - vel_ep)?

  Eigen::Matrix6Xd J_world_tool_in_world = m_chain_world_tool->getJacobian(m_q);

  // Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd_full(J_world_tool_in_world.rightCols(m_nax), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_full(m_chain_world_tool->getJacobian(m_q), Eigen::ComputeFullV);
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), fmt::format("Singular values: {}", svd_full.singularValues()));
  if(svd_full.nonzeroSingularValues() != std::min(svd_full.rows(), svd_full.cols()))
    RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1000, "SINGULARITY POINT (null singular values)");
  else if (svd_full.singularValues()(0)/svd_full.singularValues()(std::min(svd_full.rows(), svd_full.cols())-1) > 1e2)
    RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1000, "SINGULARITY POINT (high conditioning number)");

#define USE_QP
  auto clik = [&, this](const Eigen::VectorXd& p_twist_next_tool_world_in_world) -> Eigen::VectorXd {
    Eigen::VectorXd gradientW = m_kp_last_task*(full_position_references - m_q) + m_kv_last_task*(full_velocity_references - m_qp);
    Eigen::Vector6d pose_error_tool_world_in_world;
    rdyn::getFrameDistanceQuat(T_next_world_tool, T_world_tool, pose_error_tool_world_in_world);
    Eigen::Vector6d velocity_error_tool_world_in_world = p_twist_next_tool_world_in_world - twist_tool_world_in_world;
    Eigen::Vector6d acc_non_linear_in_world = m_chain_world_tool->getDTwistNonLinearPartTool(m_q, m_qp);
    Eigen::Vector6d correction =  m_parameters.clik.kv * (velocity_error_tool_world_in_world)
                                  + m_parameters.clik.kp * (pose_error_tool_world_in_world)
                                  - acc_non_linear_in_world
                                  + cart_acc_tool_target_in_world
                                  - J_world_tool_in_world * gradientW
                                  ;
    if(m_parameters.debug)
    {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.resize(18);
      std::copy(pose_error_tool_world_in_world.begin(), pose_error_tool_world_in_world.end(),msg.data.begin());
      std::copy(velocity_error_tool_world_in_world.begin(), velocity_error_tool_world_in_world.end(), std::next(msg.data.begin(), 6));
      std::copy(acc_non_linear_in_world.begin(), acc_non_linear_in_world.end(), std::next(msg.data.begin(), 12));
      m_clik_components_pub->publish(msg);
      msg.data.clear(); msg.data.resize(6);
      std::copy(correction.begin(), correction.end(), msg.data.begin());
      m_clik_correction_pub->publish(msg);
    }


#ifndef USE_QP
    Eigen::MatrixXd Q_half(m_full_nax, m_full_nax);
    Q_half.diagonal() = Eigen::Map<Eigen::VectorXd>(m_parameters.clik.task.weights.data(), m_parameters.clik.task.weights.size())
                            .cwiseSqrt();
    Q_half.diagonal().head<3>() *= (1 + m_parameters.clik.task.alpha_gain * m_elastoplastic_model->alpha());
    //RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "1/cond: " << svd_full.singularValues()(Eigen::last)/svd_full.singularValues()(0));
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd_q(J_world_tool_in_world * Q_half, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return gradientW + Q_half * svd_q.solve(correction);
#else
    // QP
    /*
     * min       qpp^T  W^(-1)  qpp
     *
     * s.t.      J  qpp  =  xpp
     *
     */

    // int prb_dim = m_full_nax * 1;
    int prb_dim = m_full_nax + (m_full_nax - m_nax);
    Eigen::VectorXd sol(prb_dim);
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(prb_dim, prb_dim);
    Eigen::VectorXd F = Eigen::VectorXd::Zero(prb_dim);
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
    Eigen::MatrixXd W_half;

    W.diagonal().head(m_full_nax) = Eigen::Map<Eigen::VectorXd>(m_parameters.clik.task.weights.data(), m_parameters.clik.task.weights.size());
    //W.diagonal().head<3>() *= (1.0 + m_parameters.clik.task.alpha_gain * m_elastoplastic_model->alpha());
    W_half = W.cwiseSqrt();

    G.topLeftCorner(m_full_nax, m_full_nax).diagonal() = W.diagonal().cwiseInverse();
    //G.diagonal().tail(m_full_nax).setOnes();

    // last task
    Eigen::MatrixXd At, Ae, Aw, As, Ap, Av;
    Eigen::VectorXd bt, be, bw, bs, bp, bv;

    // Velocity
    At = - m_kv_last_task * Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
    bt = At * full_velocity_references;
    Ae = At * m_dt;
    be = bt - At * m_qp;
    Aw = Ae * W_half;
    bw = be;
    As = (Aw * svd_full.matrixV()).rightCols(m_full_nax - m_nax);
    bs = bw;
    Av = As;
    bv = - bs;

    // Position
    At = - m_kp_last_task * Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
    bt = At * full_position_references;
    Ae = 0.5 * At * std::pow(m_dt, 2);
    be = bt - At * (m_q + m_qp * m_dt);
    Aw = Ae * W_half;
    bw = be;
    As = (Aw * svd_full.matrixV()).rightCols(m_full_nax - m_nax);
    bs = bw;
    Ap = As;
    bp = - bs;


    // EQ Constraints
    int num_eq = 6 + m_full_nax;
    //int num_eq = 6;
    Eigen::MatrixXd CE = Eigen::MatrixXd::Zero(num_eq, prb_dim);
    Eigen::VectorXd ce = Eigen::VectorXd::Zero(num_eq);
      // Main constraint
    CE.block(0,0,6, m_full_nax) = (J_world_tool_in_world * W_half);
    ce.segment(0, 6) = acc_non_linear_in_world
         - cart_acc_tool_target_in_world
         - m_parameters.clik.kv * (velocity_error_tool_world_in_world)
         - m_parameters.clik.kp * (pose_error_tool_world_in_world);
      // A p = b
    CE.block(6, m_full_nax, m_full_nax, m_full_nax - m_nax) = Av + Ap;
    ce.segment(6, m_full_nax) = bv + bp;

    // DISEQ Constraints
    // TODO: include weight matrix
    Eigen::MatrixXd CI(2 * m_full_nax + 4 * m_nax, prb_dim); Eigen::VectorXd ci(2 * m_full_nax + 4 * m_nax);
      // Velocity
    CI.block(0, 0, m_full_nax, m_full_nax) = Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
    CI.block(m_nax, 0, m_full_nax, m_full_nax) = - Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
    ci.head<3>() <<
        (m_parameters.floating_base.max_vel.linear[0] - m_qp(0)),
        (m_parameters.floating_base.max_vel.linear[1] - m_qp(1)),
        (m_parameters.floating_base.max_vel.angular   - m_qp(2));
    ci.segment(3, m_nax) = (m_limits.vel - m_qp.tail(m_nax));
    ci.segment(m_full_nax, 3) <<
        (-m_parameters.floating_base.max_vel.linear[0] + m_qp(0)),
        (-m_parameters.floating_base.max_vel.linear[1] + m_qp(1)),
        (-m_parameters.floating_base.max_vel.angular   + m_qp(2));
    ci.segment(m_full_nax + 3, m_nax) = (-m_limits.vel + m_qp.tail(m_nax));
      // Acceleration
    CI.block(2 * m_full_nax, m_full_nax - m_nax, m_nax, m_nax) =           Eigen::MatrixXd::Identity(m_nax, m_nax);
    CI.block(2 * m_full_nax + m_nax, m_full_nax - m_nax, m_nax, m_nax) = - Eigen::MatrixXd::Identity(m_nax, m_nax);
    ci.segment(2 * m_full_nax, m_nax) = m_limits.acc;
    ci.segment(2 * m_full_nax + m_nax, m_nax) = - m_limits.acc;
      // Positions
    CI.block(2 * m_full_nax + 2 * m_nax, m_full_nax - m_nax, m_nax, m_nax) =   0.5 * m_dt * m_dt * Eigen::MatrixXd::Identity(m_nax, m_nax);
    CI.block(2 * m_full_nax + 3 * m_nax, m_full_nax - m_nax, m_nax, m_nax) = - 0.5 * m_dt * m_dt * Eigen::MatrixXd::Identity(m_nax, m_nax);
    ci.segment(2 * m_full_nax + 2 * m_nax, m_nax) =   m_limits.pos_lower - (m_q.tail(m_nax) + m_qp.tail(m_nax) * m_dt);
    ci.segment(2 * m_full_nax + 3 * m_nax, m_nax) = - m_limits.pos_upper + (m_q.tail(m_nax) + m_qp.tail(m_nax) * m_dt);

    double ret = Eigen::solve_quadprog(G,
        F,
        CE.transpose(),
        ce,
        CI.transpose(),
        ci,
        sol);
    assert(ret != std::numeric_limits<double>::infinity());
    Eigen::VectorXd null_space_q(m_full_nax);
    null_space_q << Eigen::VectorXd::Zero(6), sol.tail(m_full_nax - 6);
    Eigen::VectorXd return_q = sol.head(m_full_nax) + svd_full.matrixV() * null_space_q;
    return W_half * return_q;
#endif
  };

  Eigen::VectorXd qepp = clik(twist_next_tool_world_in_world);
  Eigen::VectorXd qp = m_qp.tail(m_nax) + qepp.tail(m_nax) * m_dt;

  // Scaling due to joint velocity limits
  double scaling_vel = 1.0;
  for(size_t idx = 0; idx < m_nax; idx++)
  {
    scaling_vel = std::max(scaling_vel,std::abs(qp(idx))/m_limits.vel(idx));
  }
  if(scaling_vel > 1)
  {
    qepp = clik(twist_next_tool_world_in_world/scaling_vel);
    RCLCPP_WARN(this->get_node()->get_logger(), "Joint velocity greater than limits (ratio = %4f). Applying scaling", scaling_vel);
  }
  m_q  += m_qp * m_dt + 0.5 * qepp * std::pow(m_dt, 2);
  m_qp += qepp * m_dt;


  // Saturation?
  for(size_t idx = 0; idx < m_nax; ++idx)
  {
    double q = m_q(idx + (m_full_nax - m_nax));
    double dq = m_qp(idx + (m_full_nax - m_nax));
    m_q(idx + (m_full_nax - m_nax)) =  std::max(m_limits.pos_lower(idx), std::min(m_limits.pos_upper(idx), m_q(idx + (m_full_nax - m_nax))));
    m_qp(idx + (m_full_nax - m_nax)) = std::max(-m_limits.vel(idx),      std::min(m_limits.vel(idx),       m_qp(idx + (m_full_nax - m_nax))));
    if(q != m_q(idx + (m_full_nax - m_nax)))
      RCLCPP_WARN(this->get_node()->get_logger(), "Saturation of POSITION on manipulator joint with index %d", idx);
    if(dq != m_qp(idx + (m_full_nax - m_nax)))
      RCLCPP_WARN(this->get_node()->get_logger(), "Saturation of VELOCITY on manipulator joint with index %d", idx);
  }

  // ***********
  // ** Write **
  // ***********
  if(m_used_command_interfaces.at(0))
  {
    for(size_t ax = 0; ax < m_nax; ++ax)
    {
      m_joint_command_interfaces.at(0).at(ax).get().set_value(m_q(ax + (m_full_nax - m_nax)));
    }
  }
  if(m_used_command_interfaces.at(1))
  {
    for(size_t ax = 0; ax < m_nax; ++ax)
    {
      m_joint_command_interfaces.at(1).at(ax).get().set_value(m_qp(ax + (m_full_nax - m_nax)));
    }
  }

  Eigen::Vector6d qp_base_in_world = Eigen::Vector6d::Zero();
  if(m_float_base.enabled)
  {
    qp_base_in_world = twist_from_base_velocity(m_qp.head<3>());
  }

  Eigen::Vector6d qp_base_in_base = rdyn::spatialRotation(qp_base_in_world, m_T_world_base.linear().transpose());
  m_float_base.velocity_in_base = qp_base_in_base;
  geometry_msgs::msg::Twist cmd_vel = Eigen::toMsg(qp_base_in_base);
  if(m_float_base.enabled)
  {
    m_pub_cmd_vel->publish(cmd_vel);
  }

  m_T_world_base = rdyn::spatialIntegration(m_T_world_base, qp_base_in_world, m_dt);

  m_old_q = m_q;
  m_old_qp = m_qp;

  // *************
  // ** PUBLISH **
  // *************
  if(m_parameters.debug)
  {
    std_msgs::msg::Float64MultiArray msg_z;
    msg_z.data.resize(3);
    // msg_z.data = std::vector<double>(m_elastoplastic_model->z().data(), m_elastoplastic_model->z().data() + m_elastoplastic_model->z().size());
    std::copy(m_elastoplastic_model->z().begin(), m_elastoplastic_model->z().end(), msg_z.data.begin());
    m_pub_z->publish(msg_z);

    std_msgs::msg::Float64MultiArray msg_w;
    // msg_w.data = std::vector<double>(m_elastoplastic_model->w().data(), m_elastoplastic_model->w().data() + m_elastoplastic_model->w().size());
    msg_w.data.resize(3);
    std::copy(m_elastoplastic_model->z().begin(), m_elastoplastic_model->z().end(), msg_w.data.begin());
    m_pub_w->publish(msg_w);

    geometry_msgs::msg::WrenchStamped msg_friction_in_world;
    msg_friction_in_world.header.frame_id = "world";
    msg_friction_in_world.header.stamp = this->get_node()->get_clock()->now();
    msg_friction_in_world.wrench.force.x = m_elastoplastic_model->friction_force()[0];
    msg_friction_in_world.wrench.force.y = m_elastoplastic_model->friction_force()[1];
    msg_friction_in_world.wrench.force.z = m_elastoplastic_model->friction_force()[2];
    msg_friction_in_world.wrench.torque.x = 0.0;
    msg_friction_in_world.wrench.torque.y = 0.0;
    msg_friction_in_world.wrench.torque.z = 0.0;
    m_pub_friction_in_world->publish(msg_friction_in_world);

    geometry_msgs::msg::WrenchStamped msg_wrench_in_tool;
    msg_wrench_in_tool.header.frame_id = m_parameters.frames.tool;
    msg_wrench_in_tool.header.stamp = this->get_node()->get_clock()->now();
    msg_wrench_in_tool.wrench.force.x =  wrench_tool_in_tool[0];
    msg_wrench_in_tool.wrench.force.y =  wrench_tool_in_tool[1];
    msg_wrench_in_tool.wrench.force.z =  wrench_tool_in_tool[2];
    msg_wrench_in_tool.wrench.torque.x = wrench_tool_in_tool[3];
    msg_wrench_in_tool.wrench.torque.y = wrench_tool_in_tool[4];
    msg_wrench_in_tool.wrench.torque.z = wrench_tool_in_tool[5];
    m_pub_wrench_in_world->publish(msg_wrench_in_tool);

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

    geometry_msgs::msg::Twist xp_msg;
    Eigen::Vector6d xp_to_pub = m_chain_world_tool->getJacobian(m_q) * m_qp;
    m_xp_pub->publish(Eigen::toMsg(xp_to_pub));

    geometry_msgs::msg::PoseStamped next_pose_msg;
    next_pose_msg.pose = toMsg(T_next_world_tool);
    next_pose_msg.header.frame_id = "map";
    m_pub_next_pose->publish(next_pose_msg);

    sensor_msgs::msg::JointState jref_msg;
    jref_msg.header.stamp = get_node()->get_clock()->now();
    jref_msg.name = m_joint_names;
    jref_msg.position.resize(m_full_nax);
    jref_msg.velocity.resize(m_full_nax);
    std::copy(full_position_references.begin(), full_position_references.end(), jref_msg.position.begin());
    std::copy(full_velocity_references.begin(), full_velocity_references.end(), jref_msg.velocity.begin());
    m_pub_joint_reference->publish(jref_msg);

  }

  return controller_interface::return_type::OK;
}

Eigen::VectorXd ElastoplasticController::base_velocity_from_twist(const Eigen::Vector6d& p_w)
{
  return p_w({0, 1, 5});
}

Eigen::Vector6d ElastoplasticController::twist_from_base_velocity(const Eigen::Vector3d& p_v)
{
  return Eigen::Vector6d {p_v(0), p_v(1), 0, 0, 0, p_v(2)};
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

PLUGINLIB_EXPORT_CLASS(elastoplastic::ElastoplasticController, controller_interface::ChainableControllerInterface);
