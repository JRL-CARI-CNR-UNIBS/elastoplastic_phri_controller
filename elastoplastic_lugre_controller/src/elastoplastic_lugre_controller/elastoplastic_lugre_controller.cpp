#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <urdfdom_headers/urdf_model/model.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <fmt/color.h>

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

//   const std::string float_base_urdf = R"(<?xml version='1.0'?>
// <link name='x_base'/>
// <link name='y_base'/>
// <link name='rz_base'/>
// <link name='mount_link'/>
// <joint name='x2y' type='prismatic'>
//   <parent link='x_base'/>
//   <child link='y_base'/>
//   <origin xyz='0 0 0'/>
//   <axis xyz='0 1 0'/>
//   <limit lower='-1e10' upper='1e10' effort='1e10' velocity='1e10'/>
// </joint>
// <joint name='y2rz' type='prismatic'>
//   <parent link='y_base'/>
//   <child link='rz_base'/>
//   <origin xyz='0 0 0'/>
//   <axis xyz='0 0 1'/>
//   <limit lower='-1e10' upper='1e10' effort='1e10' velocity='1e10'/>
// </joint>
// <joint name='rz2mount' type='revolute'>
//   <parent link='rz_base'/>
//   <child link='mount_link'/>
//   <origin xyz='0 0 0'/>
//   <axis xyz='1 0 0'/>
//   <limit lower='-1e10' upper='1e10' effort='1e10' velocity='1e10'/>
// </joint>
//   )";
//   urdf::ModelInterfaceSharedPtr float_base_model = urdf::parseURDF(float_base_urdf);
//   rdyn::ChainPtr float_base_chain = rdyn::createChain(*float_base_model, "x_base", "mount_link", {0,0,-9.806});

//   m_chain_world_tool = rdyn::joinChains(float_base_chain, m_chain_base_tool);

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
  m_chain_base_tool  ->setInputJointsName(m_parameters.joints, what);
  m_chain_base_sensor->setInputJointsName(m_parameters.joints, what);

  m_robot_description_configuration = RDStatus::OK;
}

controller_interface::CallbackReturn ElastoplasticController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  m_parameters = m_param_listener->get_params();

  m_elastoplastic_model = std::make_unique<ElastoplasticModel>(get_model_data());

  m_float_base.enabled = m_parameters.floating_base.enabled;

  m_nax = m_parameters.joints.size();
  m_q.resize(m_nax);
  m_qp.resize(m_nax);
  m_qpp.resize(m_nax);

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
  m_pub_pos_correction =   this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/pose_correction", 10);
  m_pub_vel_correction  =   this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/vel_correction", 10);
  m_clik_pub = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/clik_errors",rclcpp::QoS(10).durability_volatile().reliable());
  m_pub_twist_in_world = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/twist_in_world", 10);

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
  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.begin(), [](const hardware_interface::LoanedStateInterface& lsi){
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
    m_pub_pos_correction->on_activate();
    m_pub_vel_correction ->on_activate();
    m_clik_pub->on_activate();
    m_pub_twist_in_world->on_activate();
  }

  // geometry_msgs::msg::PoseWithCovariance init_pose_msg;
  // init_pose_msg.pose.orientation.w = 1.0;
  // m_rt_buffer_base_pose_in_world.initRT(init_pose_msg);
  // m_rt_buffer_base_twist_in_base.initRT(geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ZERO));
  m_rt_buffer_base_odom.initRT(nav_msgs::msg::Odometry(rosidl_runtime_cpp::MessageInitialization::ALL));
  m_rt_buffer_fb_target.initRT(geometry_msgs::msg::Twist(rosidl_runtime_cpp::MessageInitialization::ZERO));

  m_last_odom_msg_time = this->get_node()->get_clock()->now();
  m_T_world_base = Eigen::Affine3d::Identity();

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn ElastoplasticController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.begin(), [](const hardware_interface::LoanedStateInterface& lsi){
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

  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), 0.0);

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

  nav_msgs::msg::Odometry odom_msg = *(m_rt_buffer_base_odom.readFromRT());
  geometry_msgs::msg::PoseWithCovariance msg_pose_base_in_world = odom_msg.pose;
  Eigen::Vector6d twist_base_world_in_world;
  if(rclcpp::Time(odom_msg.header.stamp) >= m_last_odom_msg_time)
  {
    Eigen::fromMsg(msg_pose_base_in_world.pose, m_T_world_base);
    m_last_odom_msg_time = odom_msg.header.stamp;
  }

  Eigen::VectorXd joint_position_references(m_parameters.joints.size());
  joint_position_references = Eigen::Map<Eigen::VectorXd>(reference_interfaces_.data(), m_parameters.joints.size());
  Eigen::VectorXd joint_velocity_references(m_parameters.joints.size());
  joint_velocity_references = Eigen::Map<Eigen::VectorXd>(std::next(reference_interfaces_.data(), m_parameters.joints.size()), m_parameters.joints.size());
  Eigen::Vector6d target_twist_tool_base_in_base = m_chain_base_tool->getJacobian(joint_position_references) * joint_velocity_references;

  auto from_base_to_world = [this](const Eigen::Vector6d& v){
    return rdyn::spatialRotation(v, m_T_world_base.linear());
  };

  Eigen::Vector6d target_twist_tool_base_in_world = from_base_to_world(target_twist_tool_base_in_base);

  Eigen::Vector6d target_twist_base_world_in_base, target_twist_base_world_in_world;
  Eigen::fromMsg(*(m_rt_buffer_fb_target.readFromRT()), target_twist_base_world_in_base);
  target_twist_base_world_in_world = from_base_to_world(target_twist_base_world_in_base);

  std::array<double, 3> ft_force = m_ft_sensor->get_forces();
  std::array<double, 3> ft_torque = m_ft_sensor->get_torques();
  Eigen::Vector6d wrench_sensor_in_sensor(ft_force[0], ft_force[1], ft_force[2],
                                          ft_torque[0], ft_torque[1], ft_torque[2]);
  if(wrench_sensor_in_sensor.hasNaN())
  {
    RCLCPP_WARN_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1000, "Force sensor contains NaN values. Full measure discarded and replaced with zero");
    wrench_sensor_in_sensor.setZero();
  }

  // Get Floating Base state
  Eigen::Vector6d twist_base_world_in_base;
  Eigen::fromMsg(odom_msg.twist.twist, twist_base_world_in_base);
  twist_base_world_in_world = from_base_to_world(twist_base_world_in_base);

  Eigen::VectorXd target_twist_tool_world_in_world = target_twist_tool_base_in_world + target_twist_base_world_in_world;

  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });

  Eigen::VectorXd q_start = m_q;
  Eigen::VectorXd qp_start = m_qp;

  // ************
  // ** Update **
  // ************
  Eigen::Affine3d  T_base_tool = m_chain_base_tool->getTransformation(m_q);
  Eigen::Affine3d  T_world_tool = m_T_world_base * T_base_tool;
  Eigen::Matrix6Xd J_base_tool_in_base = m_chain_base_tool->getJacobian(m_q);
  Eigen::Vector6d  twist_tool_base_in_base = J_base_tool_in_base * m_qp;
  Eigen::Vector6d  twist_tool_world_in_base = twist_tool_base_in_base + twist_base_world_in_base;

  Eigen::Vector6d  twist_tool_world_in_world = from_base_to_world(twist_tool_world_in_base);

  Eigen::Vector6d cart_vel_error_tool_target_in_world = twist_tool_world_in_world - target_twist_tool_world_in_world;
  Eigen::Vector6d cart_acc_tool_target_in_world = Eigen::Vector6d::Zero();

  Eigen::Affine3d T_world_target = rdyn::spatialIntegration(T_world_tool, target_twist_tool_world_in_world, period.seconds());

  Eigen::Vector6d cart_error_target_tool_in_world;
  rdyn::getFrameDistance(T_world_tool, T_world_target, cart_error_target_tool_in_world);

  Eigen::Affine3d T_base_sensor = m_chain_base_sensor->getTransformation(m_q);
  Eigen::Affine3d T_tool_sensor = T_base_tool.inverse() * T_base_sensor;
  wrench_sensor_in_sensor.head<3>() = wrench_sensor_in_sensor.head<3>().unaryExpr([this](double w){return std::abs(w) > m_parameters.wrench_deadband.at(0)? w : 0.0;});
  wrench_sensor_in_sensor.tail<3>() = wrench_sensor_in_sensor.tail<3>().unaryExpr([this](double w){return std::abs(w) > m_parameters.wrench_deadband.at(1)? w : 0.0;});
  Eigen::Vector6d wrench_tool_in_tool = rdyn::spatialDualTranformation(wrench_sensor_in_sensor, T_tool_sensor);


  /*
 *                                   +-----------------------+
 *         +---+                     |                       |
 *    xpp  | 1 |      +---+ [-]      |     Elastoplastic     |   cart_vel_error_tool_target_in_tool    +---+ [-]
 * <-------+ - +<-----+ + +<---------+                       +<----------------------------------------+ + +<-------
 *         | m |      +-+-+          |      Controller       |                                         +-+-+  twist_tool_world_in_tool = FK(q,qp)
 *         +---+        ^            |                       |                                           ^
 *                      |            +-----------------------+                                           |
 *                      |                                                                                |
 *                      |                                                                                |
 *                   external                                                             target_twist_tool_world_in_tool
 *                    force
 */

  // cart_vel_error_tool_target_in_world.head<3>() = cart_vel_error_tool_target_in_world.head<3>().unaryExpr([this](double w){return std::abs(w) > 1e-3? w : 0.0;});

  Eigen::Vector6d cart_acc_tool_target_in_tool;
  Eigen::Vector6d cart_vel_error_tool_target_in_tool = rdyn::spatialRotation(cart_vel_error_tool_target_in_world, T_world_tool.linear().transpose());
  cart_acc_tool_target_in_tool.head<3>() = m_elastoplastic_model->update(cart_vel_error_tool_target_in_tool.head<3>(),
                                                                         wrench_tool_in_tool.head<3>(),
                                                                         period.seconds());
  cart_acc_tool_target_in_world.head<3>() = rdyn::spatialRotation(cart_acc_tool_target_in_tool, T_world_tool.rotation()).head<3>();

  /*
   *
   *                                                                     +--------------+
   * T_world_target                                       +---+          |              |
   * target_twist_tool_world_in_world  +----------------->+ + +--------->+    CLIK()    |
   *                                                      +-+-+          |              |
   *                                                        ^            +--------------+
   *          m_elastoplastic_target_tool_in_world.position |
   *                                              .velocity |     +-----+
   *                                                        |     |  1  |
   *                                                        +-----+  -  +<-----+ cart_acc_tool_target_in_world
   *                                                              |  s2 |
   *                                                              +-----+
   */

  m_delta_elastoplastic_in_world.position += m_delta_elastoplastic_in_world.velocity * period.seconds() + 0.5 * cart_acc_tool_target_in_world * std::pow(period.seconds(), 2.0);
  m_delta_elastoplastic_in_world.velocity += cart_acc_tool_target_in_world * period.seconds();

  // m_delta_elastoplastic_in_world.position = Eigen::Vector6d::Zero();
  // m_delta_elastoplastic_in_world.velocity = Eigen::Vector6d::Zero();

  Eigen::Affine3d T_next_world_tool = T_world_target;
  T_next_world_tool.translation() += m_delta_elastoplastic_in_world.position.head<3>();
  Eigen::Vector6d twist_next_tool_world_in_world = m_delta_elastoplastic_in_world.velocity + target_twist_tool_world_in_world;

  // ============= TEST CLIK ================
  // ** Overwrite velocities and positions **
  /*
  static bool ini {false};
  static double pi, vi;
  static double temp;
  if(!ini)
  {
    temp = 0.0;
    ini=true;
    pi = T_world_tool.translation()(1);
    vi = twist_tool_world_in_world(1);
  }

  constexpr double AMP = 0.1;
  constexpr double PULSE = 2 * M_PI * 0.4;
  T_next_world_tool = T_world_tool;
  T_next_world_tool.translation()(1) = pi + AMP * (1 - std::cos(PULSE * temp));
  twist_next_tool_world_in_world.tail<3>() = Eigen::Vector3d::Zero();
  twist_next_tool_world_in_world(1) = + AMP * PULSE * std::sin(PULSE * temp);
  twist_next_tool_world_in_world(0) = 0.0;
  twist_next_tool_world_in_world(2) = 0.0;
  temp += period.seconds();
  */
  // =========================================

  // Inverse Kinematics
  Eigen::Matrix6Xd J_G_base_tool_in_base = m_chain_base_tool->getJacobian(m_q);
  Eigen::Matrix6Xd J_G_world_tool_in_base(6, J_G_base_tool_in_base.cols() + m_float_base.nax());
  J_G_world_tool_in_base.block(0,0,6,J_G_base_tool_in_base.cols()) << J_G_base_tool_in_base;
  if(m_float_base.enabled)
  {
    J_G_world_tool_in_base.block(0, J_G_base_tool_in_base.cols(), 6, m_float_base.nax()) << m_float_base.jacobian();
  }

  // Eigen::Vector6d  distance_next_world_tool_now;
  // Eigen::Matrix66d B_AG;
  // rdyn::getFrameDistanceQuatJac(T_world_tool, T_next_world_tool, distance_next_world_tool_now, B_AG);
  // Eigen::Matrix6Xd J_A_world_tool = B_AG.inverse() * J_G_world_tool;

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd(J_G_world_tool_in_base, Eigen::ComputeThinU | Eigen::ComputeThinV);
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), fmt::format("Singular values: {}", svd.singularValues()));
  if(svd.nonzeroSingularValues() != std::min(svd.rows(), svd.cols()))
    RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1000, "SINGULARITY POINT (null singular values)");
  else if (svd.singularValues()(0)/svd.singularValues()(std::min(svd.rows(), svd.cols())-1) > 1e2)
    RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1000, "SINGULARITY POINT (high conditioning number)");

  auto task_during_plastic = [&, this](const Eigen::VectorXd& p_q) -> Eigen::VectorXd {
    constexpr double kp = 1.0;
    Eigen::VectorXd q = p_q.head(m_nax);
    Eigen::VectorXd full(m_nax + m_float_base.nax());
    Eigen::VectorXd result = Eigen::VectorXd::Zero(p_q.size());
    for(long int idx = 0; idx < q.size(); ++idx)
    {
      result(idx) = - kp/m_nax * (q(idx) - 0.5*(this->m_limits.pos_upper(idx) + this->m_limits.pos_lower(idx))) / (this->m_limits.pos_upper(idx) - this->m_limits.pos_lower(idx));
    }
    full << result, Eigen::VectorXd::Zero(m_float_base.nax());
    return full;
  };

  auto task_during_elastic = [&, this](const Eigen::VectorXd& p_qp, const Eigen::VectorXd& p_q) -> Eigen::VectorXd {
    constexpr double kp = 1.0; // FIXME: Cambiare di posto
    Eigen::VectorXd full(m_nax + m_float_base.nax());
    Eigen::VectorXd result(m_float_base.nax());
    if(this->m_float_base.enabled)
    {
      result = - kp * (p_qp.tail(this->m_float_base.nax()) - target_twist_base_world_in_base(this->m_float_base.idxs()));
    }
    else
    {
      result.setZero();
    }
    full << p_q,
            result;
    return full;
  };

  // WARNING: Esiste un modo più intelligente per fare la selezione?
  // WARNING: come gestisco i giunti che non vengono usati dal task?
  auto task_selector = [&, this]() -> Eigen::VectorXd {
    return m_elastoplastic_model->alpha() > 0? task_during_plastic(m_q) : task_during_elastic(twist_base_world_in_world, m_q);
  };

  auto clik = [&, this](const Eigen::VectorXd& p_twist_next_tool_world_in_world) -> Eigen::VectorXd {
    Eigen::Vector6d pose_error_tool_world_in_world;
    rdyn::getFrameDistance(T_next_world_tool, T_world_tool, pose_error_tool_world_in_world);
    /* DEBUG */
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(6);
    std::copy(pose_error_tool_world_in_world.begin(), pose_error_tool_world_in_world.end(),msg.data.begin());
    Eigen::Vector6d dtwist = p_twist_next_tool_world_in_world - twist_tool_world_in_world;
    std::copy(dtwist.begin(), std::next(dtwist.begin(), 3), std::next(msg.data.begin(), 3));
    m_clik_pub->publish(msg);
    /* ***** */
    Eigen::Vector6d acc_non_linear = m_chain_base_tool->getDTwistNonLinearPartTool(m_q, m_qp); // NOTE: Sicuro non ci sia nulla della base?
    Eigen::VectorXd svd_solve = svd.solve(m_parameters.clik.kd * (p_twist_next_tool_world_in_world - twist_tool_world_in_world)
                                          + m_parameters.clik.kp * (pose_error_tool_world_in_world)
                                          - acc_non_linear
                                          + cart_acc_tool_target_in_world
                                          // - J_G_world_tool * task_selector()
                                          );
    // return task_selector() + svd_solve;
    return svd_solve;
  };

  /* VELOCITY + NO_TASK CLIK
   * ************************ */

   // auto clik = [&, this]() -> Eigen::VectorXd {
   //   Eigen::Vector6d pose_error_tool_world_in_world;
   //   rdyn::getFrameDistanceQuat(T_next_world_tool, T_world_tool, pose_error_tool_world_in_world);
   //   std_msgs::msg::Float64MultiArray msg;
   //   msg.data.resize(6);
   //   std::copy(pose_error_tool_world_in_world.begin(), pose_error_tool_world_in_world.end(),msg.data.begin());
   //   m_clik_pub->publish(msg);
   //   Eigen::VectorXd svd_solve = svd.solve(twist_next_tool_world_in_world
   //                                         + m_parameters.clik.kp * (pose_error_tool_world_in_world)
   //                                         - J_G_world_tool * task_selector()
   //       );
   //   return task_selector() + svd_solve;
   //   // return svd_solve;
   // };
   // Eigen::VectorXd qep_clik = clik();
   // Eigen::VectorXd qepp = Eigen::VectorXd::Zero(6); // temporary, only when missing mobile base
   // m_qp = qep_clik.head(m_nax);
   // m_q += qep_clik.head(m_nax) * period.seconds();


  Eigen::VectorXd qepp = clik(twist_next_tool_world_in_world);
  Eigen::VectorXd qp = m_qp + qepp.head(m_nax) * period.seconds();

  // Scaling due to joint velocity limits
  double scaling_vel = 1.0;
  for(size_t idx = 0; idx < m_nax; idx++)
  {
    scaling_vel = std::max(scaling_vel,std::abs(qp(idx))/m_limits.vel(idx));
  }
  if(scaling_vel > 1)
  {
    qepp = clik(twist_next_tool_world_in_world/scaling_vel);
  }

  m_q  += m_qp * period.seconds() + 0.5 * qepp.head(m_nax) * std::pow(period.seconds(), 2);
  m_qp += qepp.head(m_nax) * period.seconds();

  // Saturation?
  for(size_t idx = 0; idx < m_nax; ++idx)
  {
    m_q(idx) =  std::max(m_limits.pos_lower(idx), std::min(m_limits.pos_upper(idx), m_q(idx)));
    m_qp(idx) = std::max(-m_limits.vel(idx),      std::min(m_limits.vel(idx),       m_qp(idx)));
  }

  // ***********
  // ** Write **
  // ***********
  if(m_used_command_interfaces.at(0))
  {
    for(size_t ax = 0; ax < m_nax; ++ax)
    {
      m_joint_command_interfaces.at(0).at(ax).get().set_value(m_q(ax));
    }
  }
  if(m_used_command_interfaces.at(1))
  {
    for(size_t ax = 0; ax < m_nax; ++ax)
    {
      m_joint_command_interfaces.at(1).at(ax).get().set_value(m_qp(ax));
    }
  }

  Eigen::Vector6d qp_base_in_world = Eigen::Vector6d::Zero();
  // for(int eidx : m_float_base.idxs())
  for (size_t eidx = 0; eidx < m_float_base.nax(); ++eidx)
  {
    qp_base_in_world(m_float_base.idxs()[eidx]) = twist_base_world_in_world(m_float_base.idxs()[eidx]) + qepp.tail(m_float_base.nax())(eidx) * period.seconds();
  }

  Eigen::Vector6d qp_base_in_base = rdyn::spatialRotation(qp_base_in_world, m_T_world_base.inverse().linear());
  geometry_msgs::msg::Twist cmd_vel = Eigen::toMsg(qp_base_in_base);
  if(m_float_base.enabled)
  {
    m_pub_cmd_vel->publish(cmd_vel);
  }

  m_T_world_base.prerotate(Eigen::AngleAxisd(*(qp_base_in_world.end()-1) * period.seconds(), Eigen::Vector3d::UnitZ())).translate(qp_base_in_world.head<3>() * period.seconds());

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

    std_msgs::msg::Float64MultiArray q_msg;
    std_msgs::msg::Float64MultiArray qp_msg;
    q_msg.data.resize(m_nax);
    qp_msg.data.resize(m_nax);
    Eigen::VectorXd d_q = m_q - q_start;
    Eigen::VectorXd d_qp = m_qp - qp_start;
    std::copy(d_q.begin(), d_q.end(), q_msg.data.begin());
    std::copy(d_qp.begin(), d_qp.end(), qp_msg.data.begin());
    m_pub_pos_correction->publish(q_msg);
    m_pub_vel_correction->publish(qp_msg);
  }

  return controller_interface::return_type::OK;
}

} // namespace elastoplastic

PLUGINLIB_EXPORT_CLASS(elastoplastic::ElastoplasticController, controller_interface::ChainableControllerInterface);
