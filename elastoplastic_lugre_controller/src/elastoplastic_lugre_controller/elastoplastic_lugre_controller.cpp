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
  // m_rt_buffer_base_pose_in_world.writeFromNonRT(msg.pose);
  // m_rt_buffer_base_twist_in_base.writeFromNonRT(msg.twist); // ??
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

  // Eigen::Vector6d target_twist_tool_world_in_world = m_chain_world_tool->getJacobian(full_position_references) * full_velocity_references;
  Eigen::Vector6d target_twist_tool_world_in_world = m_chain_world_tool->getJacobian(m_q) * full_velocity_references;

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
#define INCLUDE_ADMITTANCE
#ifdef INCLUDE_ADMITTANCE
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

  /*Fake impedance*/
  // static Eigen::Vector3d z = Eigen::Vector3d::Zero();
  //cart_acc_tool_target_in_tool.head<3>() = (wrench_tool_in_tool.head<3>() - (/*m_parameters.impedance.lugre.sigma_0 * z*/ + m_parameters.impedance.lugre.sigma_1 * cart_vel_error_tool_target_in_tool.head<3>())) / m_parameters.impedance.inertia[0];
  // z = z + cart_vel_error_tool_target_in_tool.head<3>() * m_dt;

  Eigen::Vector6d cart_acc_tool_target_in_world = Eigen::Vector6d::Zero();
  cart_acc_tool_target_in_world.head<3>() = rdyn::spatialRotation(cart_acc_tool_target_in_tool, T_world_tool.rotation()).head<3>();

  m_delta_elastoplastic_in_world.velocity += cart_acc_tool_target_in_world * m_dt;
  m_delta_elastoplastic_in_world.position += m_delta_elastoplastic_in_world.velocity * m_dt + 0.5 * cart_acc_tool_target_in_world * std::pow(m_dt, 2.0); // Used only for debug

  Eigen::Vector6d twist_next_tool_world_in_world = m_delta_elastoplastic_in_world.velocity + target_twist_tool_world_in_world;
  Eigen::Affine3d T_next_world_tool = rdyn::spatialIntegration(T_world_tool, twist_next_tool_world_in_world, m_dt);

#else
  Eigen::Affine3d T_next_world_tool;
  Eigen::Vector6d twist_next_tool_world_in_world;
  Eigen::Vector6d cart_acc_tool_target_in_world;

  // ============= TEST CLIK ================
  // ** Overwrite velocities and positions **

  static bool ini {false};
  static double initial_position_z, initial_velocity_z;
  static double temp;
  static constexpr int iidd {1};
  if(!ini)
  {
    temp = 0.0;
    ini=true;
    initial_position_z = T_world_tool.translation()(iidd);
    initial_velocity_z = 0.0;
  }

  constexpr double AMP = 0.1;
  constexpr double PULSE = 2 * M_PI * 0.4;
  T_next_world_tool = m_T_world_tool_initial;
  T_next_world_tool.translation()(iidd) = initial_position_z + AMP * (1 - std::cos(PULSE * temp));
  // T_next_world_tool = rdyn::spatialIntegration(T_next_world_tool, target_twist_tool_world_in_world, m_dt);
  twist_next_tool_world_in_world = Eigen::Vector6d::Zero();
  twist_next_tool_world_in_world(iidd) = + AMP * PULSE * std::sin(PULSE * temp);
  twist_next_tool_world_in_world += target_twist_tool_world_in_world;
  cart_acc_tool_target_in_world = Eigen::Vector6d::Zero();
  cart_acc_tool_target_in_world(iidd) = AMP * PULSE * PULSE * std::cos(PULSE * temp);
  temp += m_dt;

  // =========================================
#endif

  Eigen::Matrix6Xd J_world_tool_in_world = m_chain_world_tool->getJacobian(m_q);

  // Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd_manipulator(J_world_tool_in_world.rightCols(m_nax), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd_manipulator(m_chain_base_tool->getJacobian(m_q.tail(m_nax)), Eigen::ComputeThinU | Eigen::ComputeThinV);
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), fmt::format("Singular values: {}", svd_manipulator.singularValues()));
  if(svd_manipulator.nonzeroSingularValues() != std::min(svd_manipulator.rows(), svd_manipulator.cols()))
    RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1000, "SINGULARITY POINT (null singular values)");
  else if (svd_manipulator.singularValues()(0)/svd_manipulator.singularValues()(std::min(svd_manipulator.rows(), svd_manipulator.cols())-1) > 1e2)
    RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1000, "SINGULARITY POINT (high conditioning number)");

#define USE_QP_
#ifndef USE_QP

  prb.value = [&](const cppoptlib::Problem::TVector& x) -> double {
    Eigen::Matrix6Xd J = m_chain_world_tool->getJacobian(x).rightCols(m_nax);
    return std::sqrt((J * J.transpose()).determinant());
  };

  auto task_during_plastic = [&, this](const Eigen::VectorXd& p_q) -> Eigen::VectorXd {
    constexpr double kp = 1.0;
    Eigen::VectorXd q = p_q.tail(m_nax);
    Eigen::VectorXd full(m_full_nax);
    Eigen::VectorXd result = Eigen::VectorXd::Zero(p_q.size());
    for(long int idx = 0; idx < q.size(); ++idx)
    {
      result(idx) = - kp/m_nax * (q(idx) - 0.5*(this->m_limits.pos_upper(idx) + this->m_limits.pos_lower(idx))) / (this->m_limits.pos_upper(idx) - this->m_limits.pos_lower(idx));
    }
    full << result, Eigen::VectorXd::Zero(m_float_base.nax());
    return full;
  };

  // auto task_2_during_elastic = [&, this](const Eigen::VectorXd& lower_task) -> Eigen::VectorXd {
  //   Eigen::Matrix6Xd Jm(6, m_full_nax), Jb(6, m_full_nax);
  //   Jm = Jb = Eigen::Matrix6Xd::Zero(6, m_full_nax);
  //   Jm.rightCols(m_nax) = J_world_tool_in_world.rightCols(m_nax);
  //   Jb.leftCols<3>() = J_world_tool_in_world.leftCols<3>();
  //   Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd_Jm(Jm, Eigen::ComputeThinU | Eigen::ComputeThinV);
  //   Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd_Jb(Jb, Eigen::ComputeThinU | Eigen::ComputeThinV);
  //   Eigen::Matrix6Xd

  //   (twist_next_tool_world_in_world - Jb * target_twist_base_world_in_base - Jm * lower_task)
  //   // Eigen::Matrix6Xd jac =
  // }

  // auto task_during_elastic = [&, this](const Eigen::VectorXd& p_qp, const Eigen::VectorXd& p_q) -> Eigen::VectorXd {
  //   if(p_q.size() <= 6)
  //   {
  //     return Eigen::VectorXd::Zero(p_q.size());
  //   }
  //   constexpr double kp = 1.0; // FIXME: Cambiare di posto
  //   constexpr double kp2 = 1.0; // FIXME: Cambiare di posto
  //   Eigen::VectorXd grad(m_full_nax); Eigen::MatrixXd hess(m_full_nax, m_full_nax);
  //   prb.gradient(m_q, grad);
  //   prb.hessian(m_q, hess);
  //   Eigen::VectorXd full(m_full_nax);
  //   full.head(m_float_base.nax()) = - kp * (p_qp.head(this->m_float_base.nax()) - base_velocity_from_twist(target_twist_base_world_in_base));
  //   // full.tail(m_nax).setZero();
  //   full.tail(m_nax) = - kp2 * (grad.tail(m_nax).transpose() * m_qp.tail(m_nax) * m_dt + 0.5 * m_dt * m_dt * m_qp.tail(m_nax).transpose() * hess.bottomRightCorner(m_nax, m_nax) * m_qp.tail(m_nax));
  //   return full;
  // };

  // WARNING: Esiste un modo più intelligente per fare la selezione?
  // auto task_selector = [&, this]() -> Eigen::VectorXd {
  //   // return m_elastoplastic_model->alpha() > 0? task_during_plastic(m_q) : task_during_elastic(twist_base_world_in_world, m_q);
  //   return task_during_elastic(m_qp.head<3>(), m_q);
  // };
#else
  auto task_minimize_input_QP = [&, this](Eigen::MatrixXd& G, Eigen::VectorXd& F) -> void {
    F.resize(m_full_nax); G.resize(m_full_nax, m_full_nax);
    F = Eigen::VectorXd::Zero(m_full_nax);
    G = Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
  };

  prb.value = [&](const cppoptlib::Problem::TVector& x) -> double {
    Eigen::Matrix6Xd J = m_chain_world_tool->getJacobian(x).rightCols(m_nax);
    return std::sqrt((J * J.transpose()).determinant());
  };

  auto task_manipulability_QP = [&, this](Eigen::MatrixXd& G, Eigen::VectorXd& F) -> void {
    F = Eigen::VectorXd::Zero(m_full_nax); G = Eigen::MatrixXd::Zero(m_full_nax, m_full_nax);
    Eigen::VectorXd grad(m_nax); Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(m_nax, m_nax);
    prb.gradient(m_q, grad);
    prb.hessian(m_q, hess);
    // RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "Hessian m: " << hess );
    // RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "Gradient m: " << grad );
    F.tail(m_nax) = - m_dt * m_dt * ( grad.tail(m_nax).transpose() + m_qp.tail(m_nax).transpose() * hess.bottomRightCorner(m_nax, m_nax) * m_dt);
    G.bottomRightCorner(m_nax, m_nax) = - 0.5 * hess.bottomRightCorner(m_nax, m_nax) * m_dt * m_dt * m_dt * m_dt;
    //F.tail(m_nax) = - m_dt * grad.transpose() * W.bottomRightCorner(m_nax, m_nax);
    //G.bottomRightCorner(m_nax, m_nax) = - 0.5 * hess * W.bottomRightCorner(m_nax, m_nax);
  };

  auto task_during_elastic_QP = [&, this](Eigen::MatrixXd& G, Eigen::VectorXd& F) -> void {
    F.resize(m_full_nax); F = Eigen::VectorXd::Zero(m_full_nax);
    G.resize(m_full_nax, m_full_nax); G = Eigen::MatrixXd::Zero(m_full_nax, m_full_nax);
    Eigen::Matrix<double, 3, 6> S = Eigen::Matrix<double, 3, 6>::Zero(3,6);
    S(0,0) = 1.0;
    S(1,1) = 1.0;
    S(2,5) = 1.0;
    Eigen::Matrix6Xd Jb = J_world_tool_in_world.leftCols<3>();
    F.head<3>() = 2 * m_dt * (m_qp.head<3>().transpose() * Jb.transpose() * S.transpose() * S * Jb
             - target_twist_base_world_in_world.transpose() * S.transpose() * S * Jb);
    G.topLeftCorner<3,3>() = m_dt * m_dt * Jb.transpose() * S.transpose() * S * Jb;
  };

  // auto task_center_cog_QP = [&, this](Eigen::MatrixXd& G, Eigen::VectorXd& F, Eigen::MatrixXd& W) -> void {
    // F.resize(m_full_nax); F = Eigen::VectorXd::Zero(m_full_nax);
    // G.resize(m_full_nax, m_full_nax); G = Eigen::MatrixXd::Zero(m_full_nax, m_full_nax);
    // auto links = m_chain_base_tool->getLinks();
    // Eigen::Vector3d cog = Eigen::Vector3d::Zero();
    // double mass = 0;
    // for(const auto link : links)
    // {
      // cog += link->getCog() * link->getMass();
      // mass += link->getMass();
    // }
    // cog /= mass;
  // };

  auto task_tool_over_base_2_QP = [&, this](Eigen::MatrixXd& G, Eigen::VectorXd& F) -> void {
    F = Eigen::VectorXd::Zero(m_full_nax);
    G = Eigen::MatrixXd::Zero(m_full_nax, m_full_nax);
    Eigen::Matrix6Xd J = m_chain_world_tool->getJacobian(m_q);
    G.bottomRightCorner(m_nax, m_nax) = J.rightCols(m_nax).topRows<3>().transpose() * J.rightCols(m_nax).topRows<3>() * m_dt * m_dt;
    Eigen::Vector3d v = (J.rightCols(m_nax) * m_q.tail(m_nax)).head<3>() + 1000 * (m_chain_world_tool->getTransformation(m_q).translation() - m_chain_world_base->getTransformation(m_q.head(3)).translation()).normalized();
    F.tail(m_nax) = 2 * v.transpose() * J.rightCols(m_nax).topRows<3>() * m_dt;
  };

  auto task_tool_over_base_QP = [&, this](Eigen::MatrixXd& G, Eigen::VectorXd& F) -> void {
    F = Eigen::VectorXd::Zero(m_full_nax);
    G = Eigen::MatrixXd::Zero(m_full_nax, m_full_nax);
    Eigen::Matrix66d conversion_matrix, conversion_matrix_diff;
    Eigen::Affine3d T_base_tool = m_chain_base_tool->getTransformation(m_q.tail(m_nax));
    Eigen::Vector6d w_tool_base_in_base = m_chain_base_tool->getTwistTool(m_q.tail(m_nax), m_qp.tail(m_nax));
    Eigen::Vector3d rpy = T_base_tool.rotation().eulerAngles(0,1,2); // To RPY

    using std::sin, std::cos, std::pow;
    conversion_matrix = Eigen::Matrix66d::Identity();
    conversion_matrix_diff = Eigen::Matrix66d::Zero();

    double& x = rpy(0);
    double& y = rpy(1);
    conversion_matrix.bottomRightCorner(3,3) <<
        1,  std::sin(x) * std::sin(y) / std::cos(y), -std::sin(y) * std::cos(x) / std::cos(y),
        0,  std::cos(x)                            ,  std::sin(x),
        0, -std::sin(x) / std::cos(y)              ,  std::cos(x)/std::cos(y);

    // Eigen::Vector3d d_rpy = conversion_matrix.bottomRightCorner(3,3) * rpy;
    Eigen::Vector3d d_rpy = conversion_matrix.bottomRightCorner(3,3) * w_tool_base_in_base.tail<3>();
    double& dx = d_rpy(0);
    double& dy = d_rpy(1);
    conversion_matrix_diff.bottomRightCorner(3,3) <<
        0, sin(x) * pow(sin(y),2) * dy / pow(cos(y),2) + sin(x)*dy + sin(y) * cos(x) * dx / cos(y), sin(x) * sin(y) * dx / cos(y) - pow(sin(y), 2) * cos(x) * dy / pow(cos(y),2) - cos(x) * dy,
        0, - sin(x) * dx                                                                          , cos(x) * dx,
        0, - sin(x) * sin(y) * dy / pow(cos(y),2) - cos(x) * dx / cos(y)                          , sin(x) * dx / cos(y) + sin(y) * cos(x) * dy / pow(cos(y), 2);

    Eigen::Matrix6Xd J_G = m_chain_base_tool->getJacobian(m_q.tail(m_nax));
    Eigen::Matrix6Xd J_A = conversion_matrix * J_G;
    Eigen::Vector6d Jpqp = conversion_matrix_diff * J_G * m_qp.tail(m_nax) + conversion_matrix * m_chain_base_tool->getDTwistNonLinearPartTool(m_q.tail(m_nax), m_qp.tail(m_nax));
    Eigen::Vector6d fk_q;
    fk_q << T_base_tool.translation(), rpy;
    F.tail(m_nax) = fk_q.transpose() * J_A * m_dt * m_dt
        + m_qp.tail(m_nax) * J_A.transpose() * J_A * std::pow(m_dt, 3)
        + 0.5 * Jpqp.transpose() * J_A * std::pow(m_dt, 4)
        - Eigen::Vector6d{0,0,T_base_tool.translation()(2),rpy(0),rpy(1),rpy(2)}.transpose() * J_A * std::pow(m_dt, 2);
    G.bottomRightCorner(m_nax, m_nax) = 0.25 * std::pow(m_dt, 4) * J_A.transpose() * J_A;
  };

  auto task_last_q_QP = [&, this](Eigen::MatrixXd& G, Eigen::VectorXd& F) -> void {
    Eigen::MatrixXd KP = Eigen::MatrixXd::Identity(m_full_nax,m_full_nax) * m_kp_last_task;
    F.resize(m_full_nax); G.resize(m_full_nax, m_full_nax);
    // F = - 2 * m_initial_q.transpose() * KP;
    F = m_dt * m_dt * (m_q - m_initial_q + m_qp * m_dt).transpose() * KP;
    G = KP * std::pow(m_dt, 4) * 0.5;
  };

  auto task_last_qp_QP = [&, this](Eigen::MatrixXd& G, Eigen::VectorXd& F) -> void {
    Eigen::MatrixXd KV = Eigen::MatrixXd::Identity(m_full_nax,m_full_nax) * m_kv_last_task;
    F.resize(m_full_nax); G.resize(m_full_nax, m_full_nax);
    // F = - 2 * m_initial_qp.transpose() * KV;
    F = 2 * (m_qp - m_initial_qp).transpose() * KV * m_dt;
    G = KV * m_dt * m_dt;
  };

  auto task_minimize_velocity_QP = [&, this](Eigen::MatrixXd& G, Eigen::VectorXd& F) -> void {
    F.resize(m_full_nax);
    G.resize(m_full_nax, m_full_nax);
    F = 2 * m_qp.transpose() * m_dt;
    G = Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt * m_dt;
  };

  auto joint_vel_constraints_QP = [&, this](Eigen::MatrixXd& CI, Eigen::VectorXd& ci) -> void {
    CI.resize(2 * m_full_nax, m_full_nax); // N righe, q.size() colonne
    ci.resize(2 * m_full_nax);
    CI.block(0,0,m_full_nax, m_full_nax) = - Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
    CI.block(m_full_nax, 0, m_full_nax, m_full_nax) = Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
    ci.head<3>() << (m_parameters.floating_base.max_vel.linear[0] - m_qp(0))/m_dt,
                    (m_parameters.floating_base.max_vel.linear[1] - m_qp(1))/m_dt,
                    (m_parameters.floating_base.max_vel.angular   - m_qp(2))/m_dt;
    ci.segment(3, m_nax) = (m_limits.vel - m_qp.tail(m_nax))/m_dt;
    ci.segment(m_full_nax, 3) << (-m_parameters.floating_base.max_vel.linear[0] + m_qp(0))/m_dt,
                                 (-m_parameters.floating_base.max_vel.linear[1] + m_qp(1))/m_dt,
                                 (-m_parameters.floating_base.max_vel.angular   + m_qp(2))/m_dt;
    ci.segment(m_full_nax + 3, m_nax) = (-m_limits.vel + m_qp.tail(m_nax))/m_dt;
  };

#endif
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
        // - J_world_tool_in_world * task_selector()
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
    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "1/cond: " << svd_manipulator.singularValues()(Eigen::last)/svd_manipulator.singularValues()(0));
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd_q(J_world_tool_in_world * Q_half, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return gradientW + Q_half * svd_q.solve(correction);
#else
    // QP
    int dim = m_full_nax * 1;
    Eigen::VectorXd sol(dim);
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd F = Eigen::VectorXd::Zero(dim);
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(dim, dim);
    W.diagonal() = Eigen::Map<Eigen::VectorXd>(m_parameters.clik.task.weights.data(), m_parameters.clik.task.weights.size());
    G.block(0,0,m_full_nax, m_full_nax) = W.transpose() * J_world_tool_in_world.transpose() * J_world_tool_in_world * W;
    F.segment(0, m_full_nax) = - 2 * (- acc_non_linear_in_world
                              + m_parameters.clik.kv * (velocity_error_tool_world_in_world)
                              + m_parameters.clik.kp * (pose_error_tool_world_in_world)
                              + cart_acc_tool_target_in_world
                            ).transpose() * J_world_tool_in_world * W;
    // W.diagonal() = Eigen::Map<Eigen::VectorXd>(m_parameters.clik.task.weights.data(), m_parameters.clik.task.weights.size());
    // G = W;
    // F = Eigen::VectorXd::Zero(dim);

    // Acceleration minimization
    Eigen::MatrixXd G1; Eigen::VectorXd F1;
    Eigen::MatrixXd W1 = Eigen::MatrixXd::Identity(m_full_nax,m_full_nax);
    // task_minimize_velocity_QP(G1, F1);
    // task_minimize_input_QP(G1, F1);
    // G1 = W1;
    // G.block(0,0,m_full_nax, m_full_nax) += G1 ;
    // F.segment(0, m_full_nax) += F1.transpose();

    // Secondary task
    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "manip: " << std::sqrt((J_world_tool_in_world * J_world_tool_in_world.transpose()).determinant()));
    Eigen::MatrixXd G2; Eigen::VectorXd F2;
    Eigen::MatrixXd W2 = Eigen::MatrixXd::Identity(m_full_nax,m_full_nax);
    // task_during_elastic_QP(G2, F2);
    // G.block(0,0,m_full_nax, m_full_nax) += G2 * m_parameters.clik.task.priorities[0];
    // F.segment(0, m_full_nax) += F2 * m_parameters.clik.task.priorities[0];
    // task_manipulability_QP(G2, F2);
    // G.block(0,0,m_full_nax, m_full_nax) += G2 * m_parameters.clik.task.priorities[0];
    // F.segment(0, m_full_nax) += W2 * F2;
    // task_tool_over_base_QP(G2, F2);
    // task_tool_over_base_2_QP(G2, F2);

    // Last task
    double wl = m_parameters.clik.task.priorities[1];
    Eigen::MatrixXd GLp(dim, dim), GLv(dim, dim); Eigen::VectorXd FLp(dim), FLv(dim);
    task_last_q_QP(GLp, FLp);
    task_last_qp_QP(GLv, FLv);
    GLp *= wl; FLp *= wl;
    GLv *= wl; FLv *= wl;
    G.block(0,0,m_full_nax, m_full_nax) += GLp;
    G.block(0,0,m_full_nax, m_full_nax) += GLv;
    F.segment(0, m_full_nax) += FLp;
    F.segment(0, m_full_nax) += FLv;

    // EQ Constraints
    Eigen::MatrixXd CE; Eigen::VectorXd ce;
    CE = Eigen::MatrixXd::Zero(dim, dim);
    ce = Eigen::VectorXd::Zero(dim);
    // CE = J_world_tool_in_world;
    // ce = -  (- acc_non_linear_in_world
    //          + m_parameters.clik.kv * (velocity_error_tool_world_in_world)
    //          + m_parameters.clik.kp * (pose_error_tool_world_in_world)
    //          + cart_acc_tool_target_in_world);

    // DISEQ Constraints
    Eigen::MatrixXd CI; Eigen::VectorXd ci;
    // CI = Eigen::MatrixXd::Zero(dim, dim);
    // ci = Eigen::VectorXd::Zero(dim);
    // joint_vel_constraints_QP(CI, ci);

    double ret = Eigen::solve_quadprog(G,
        F,
        CE.transpose(),
        ce,
        CI.transpose(),
        ci,
        sol);
    assert(ret != std::numeric_limits<double>::infinity());
    return sol.head(m_full_nax);
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

    // geometry_msgs::msg::WrenchStamped msg_wrench_in_tool;
    // msg_wrench_in_tool.header.frame_id = m_parameters.frames.tool;
    // msg_wrench_in_tool.header.stamp = this->get_node()->get_clock()->now();
    // msg_wrench_in_tool.wrench.force.x =  wrench_tool_in_tool[0];
    // msg_wrench_in_tool.wrench.force.y =  wrench_tool_in_tool[1];
    // msg_wrench_in_tool.wrench.force.z =  wrench_tool_in_tool[2];
    // msg_wrench_in_tool.wrench.torque.x = wrench_tool_in_tool[3];
    // msg_wrench_in_tool.wrench.torque.y = wrench_tool_in_tool[4];
    // msg_wrench_in_tool.wrench.torque.z = wrench_tool_in_tool[5];
    // m_pub_wrench_in_world->publish(msg_wrench_in_tool);

    // m_pub_cart_vel_error->publish(tf2::toMsg(cart_vel_error_tool_target_in_world));
    // m_pub_twist_in_world->publish(tf2::toMsg(twist_tool_world_in_world));

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
std::array<Eigen::MatrixXd, 6> ElastoplasticController::update_hessian(const Eigen::Matrix6Xd& jacobian, const Eigen::VectorXd& q)
{
  for(int idx = 0; idx < jacobian.rows(); idx++)
  {
    Eigen::VectorXd yt = jacobian.row(idx) - bfgs_prev.jacobian_p.row(idx);
    Eigen::VectorXd st = q - m_old_q;
    double rho = 1 / (yt.transpose() * st);
    m_hessian.at(idx) =
        (Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) - rho * st * yt.transpose())
            * bfgs_prev.hessian_p.at(idx)
        * (Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) - rho * yt * st.transpose())
        + rho * st * st.transpose();
  }
  bfgs_prev.hessian_p = m_hessian;
  bfgs_prev.jacobian_p = jacobian;
}

} // namespace elastoplastic

PLUGINLIB_EXPORT_CLASS(elastoplastic::ElastoplasticController, controller_interface::ChainableControllerInterface);
