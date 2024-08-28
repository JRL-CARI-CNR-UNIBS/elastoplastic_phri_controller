#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <urdfdom_headers/urdf_model/model.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <fmt/core.h>

namespace elastoplastic {

namespace views = std::ranges::views;


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

  m_nax = m_parameters.joints.size();

  m_q.resize(m_nax);
  m_qp.resize(m_nax);
  m_qpp.resize(m_nax);

  if(std::ranges::min(m_parameters.impedance.inertia) < 0)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Inertia has negative values!");
    return controller_interface::CallbackReturn::FAILURE;
  }
  m_inertia_inv = Eigen::Map<Eigen::Vector3d>(m_parameters.impedance.inertia.data(), m_parameters.impedance.inertia.size()).cwiseInverse();

  using namespace std::placeholders;
  m_sub_fb_target = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(m_parameters.floating_base.input_target_topic, 1, std::bind(&ElastoplasticController::get_fb_target_callback, this, _1));
  m_sub_base_odometry = this->get_node()->create_subscription<nav_msgs::msg::Odometry>(m_parameters.floating_base.odom, 1, std::bind(&ElastoplasticController::get_odometry_callback, this, _1));

  m_ft_sensor = std::make_unique<semantic_components::ForceTorqueSensor>(m_parameters.ft_sensor_name);

  m_pub_cmd_vel = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 1);
  m_float_base.enabled = m_parameters.floating_base.enabled;

  m_pub_z =                this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray> ("~/z", 10);
  m_pub_w =                this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray> ("~/w", 10);
  m_pub_friction_in_world = this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("~/friciton_in_world", 10);
  m_pub_wrench_in_world =   this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("~/wrench_in_world", 10);
  m_pub_cart_vel_error =   this->get_node()->create_publisher<geometry_msgs::msg::Twist>("~/cart_vel_error", 10);
  m_pub_pos_correction =   this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/pose_correction", 10);
  m_pub_vel_correction  =   this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/vel_correction", 10);

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

  m_state.clear();
  m_reset_window.clear();
  m_elastoplastic_target_tool_in_world.clear();

  m_joint_state_interfaces.resize(2);
  m_joint_command_interfaces.resize(2);

  for(const auto& interface : m_allowed_interface_types)
  {
    auto it = std::ranges::find(m_allowed_interface_types, interface);
    auto idx = std::distance(m_allowed_interface_types.begin(), it);
    //RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "state_interfaces_: " << state_interfaces_.size());
    if(not controller_interface::get_ordered_interfaces(state_interfaces_, m_parameters.joints, interface, m_joint_state_interfaces.at(idx)))
    {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Missing joints state interfaces");
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
      // RCLCPP_ERROR(this->get_node()->get_logger(), "Missing joints command interfaces");
      // return controller_interface::CallbackReturn::FAILURE;
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
  }

  geometry_msgs::msg::PoseWithCovariance init_pose_msg;
  init_pose_msg.pose.orientation.w = 1.0;
  m_rt_buffer_base_pose_in_world.initRT(init_pose_msg);
  m_rt_buffer_base_twist_in_base.initRT(geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ZERO));
  m_rt_buffer_fb_target.initRT(geometry_msgs::msg::Twist(rosidl_runtime_cpp::MessageInitialization::ZERO));


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

  m_state.clear();
  m_reset_window.clear();
  m_elastoplastic_target_tool_in_world.clear();

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
  // geometry_msgs::msg::Twist* msg = m_rt_buffer_twist_tool_in_base.readFromRT();
  // reference_interfaces_.at(0) = msg->linear.x;
  // reference_interfaces_.at(1) = msg->linear.y;
  // reference_interfaces_.at(2) = msg->linear.z;
  // reference_interfaces_.at(3) = msg->angular.x;
  // reference_interfaces_.at(4) = msg->angular.y;
  // reference_interfaces_.at(5) = msg->angular.z;
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
  m_rt_buffer_base_pose_in_world.writeFromNonRT(msg.pose);
  m_rt_buffer_base_twist_in_base.writeFromNonRT(msg.twist); // ??
}


controller_interface::return_type ElastoplasticController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // **********
  // ** Read **
  // **********

  geometry_msgs::msg::PoseWithCovariance msg_pose_base_in_world = *(m_rt_buffer_base_pose_in_world.readFromRT());
  Eigen::Vector6d twist_base_world_in_world;
  Eigen::Affine3d T_world_base;
  Eigen::fromMsg(msg_pose_base_in_world.pose, T_world_base);

  Eigen::VectorXd joint_position_references(m_parameters.joints.size());
  joint_position_references = Eigen::Map<Eigen::VectorXd>(reference_interfaces_.data(), m_parameters.joints.size());
  Eigen::VectorXd joint_velocity_references(m_parameters.joints.size());
  joint_velocity_references = Eigen::Map<Eigen::VectorXd>(std::next(reference_interfaces_.data(), m_parameters.joints.size()), m_parameters.joints.size());
  Eigen::Vector6d target_twist_tool_base_in_base = m_chain_base_tool->getTwistTool(joint_position_references, joint_velocity_references);

  auto from_base_to_world = [&T_world_base](const Eigen::Vector6d& v){
    return rdyn::spatialRotation(v, T_world_base.linear());
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
  Eigen::fromMsg(m_rt_buffer_base_twist_in_base.readFromRT()->twist, m_float_base.twist_in_base);
  twist_base_world_in_world = from_base_to_world(m_float_base.twist_in_base);

  Eigen::VectorXd target_twist_tool_world_in_world = target_twist_tool_base_in_world + target_twist_base_world_in_world;

  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.begin(), [](const hardware_interface::LoanedStateInterface& lsi){
    return lsi.get_value();
  });

  Eigen::Vector6d m_q_copy = m_q;
  Eigen::Vector6d m_qp_copy = m_qp;

  // ************
  // ** Update **
  // ************
  // FIXME: m_q or get from state interface?
  Eigen::Affine3d  T_base_tool = m_chain_base_tool->getTransformation(m_q);
  Eigen::Affine3d  T_world_tool = T_world_base * T_base_tool;
  Eigen::Matrix6Xd J_base_tool_in_base = m_chain_base_tool->getJacobian(m_q);
  Eigen::Vector6d  twist_tool_base_in_base = J_base_tool_in_base * m_qp;
  Eigen::Vector6d  twist_tool_world_in_base = twist_tool_base_in_base + m_float_base.twist_in_base;

  Eigen::Vector6d  twist_tool_world_in_world = from_base_to_world(twist_tool_world_in_base);

  Eigen::Vector6d cart_vel_error_tool_target_in_world = twist_tool_world_in_world - target_twist_tool_world_in_world;
  Eigen::Vector6d cart_acc_tool_target_in_world;

  Eigen::Affine3d T_world_target = rdyn::spatialIntegration(T_world_tool, target_twist_tool_world_in_world, period.seconds());

  Eigen::Vector6d cart_error_target_tool_in_world;
  rdyn::getFrameDistance(T_world_tool, T_world_target, cart_error_target_tool_in_world);

  Eigen::Affine3d T_base_sensor = m_chain_base_sensor->getTransformation(m_q);
  Eigen::Affine3d T_tool_sensor = T_base_tool.inverse() * T_base_sensor;
  Eigen::Vector6d wrench_tool_in_tool = rdyn::spatialDualTranformation(wrench_sensor_in_sensor, T_tool_sensor);
  Eigen::Vector6d wrench_tool_in_world = rdyn::spatialRotation(wrench_tool_in_tool, T_world_tool.linear());
  Eigen::Vector6d wrench_tool_in_world_filtered;
  wrench_tool_in_world_filtered.head<3>() = wrench_tool_in_world.head<3>().unaryExpr([this](double w){return std::abs(w) > m_parameters.wrench_deadband.at(0)? w : 0.0;});
  wrench_tool_in_world_filtered.tail<3>() = wrench_tool_in_world.tail<3>().unaryExpr([this](double w){return std::abs(w) > m_parameters.wrench_deadband.at(1)? w : 0.0;});

  auto compute_alpha = [this](const double z) -> double
  {
    const double& z_ba = m_parameters.impedance.lugre.z_ba;
    const double& z_ss = m_parameters.impedance.lugre.z_ss;
    if (std::abs(z) < z_ba)
    {
      return 0.0;
    }
    else if (std::abs(z) >= z_ss)
    {
      return 1.0;
    }
    else
    {
      return 0.5*std::sin(M_PI*((z-(z_ba+z_ss)/2)/(z_ss-z_ba)))+0.5;
    }
  };

  auto compute_dalpha = [this](const double z) -> double
  {
    const double& z_ba = m_parameters.impedance.lugre.z_ba;
    const double& z_ss = m_parameters.impedance.lugre.z_ss;
    if (std::abs(z) < z_ba)
    {
      return 0.0;
    }
    else if (std::abs(z) >= z_ss)
    {
      return 0.0;
    }
    else
    {
      return 0.5*std::cos(M_PI*((z-(z_ba+z_ss)/2)/(z_ss-z_ba)));
    }
  };

  const auto& impedance = m_parameters.impedance;

  Eigen::Vector4d alpha_with_r;
  alpha_with_r << m_state.z, m_state.r;
  Eigen::Vector3d alpha = Eigen::Vector3d::Constant(compute_alpha(alpha_with_r.norm()));
  ModelState d_dt;
  d_dt.r = compute_dalpha(m_state.z.norm());
  d_dt.z = cart_vel_error_tool_target_in_world.head<3>() - alpha.cwiseProduct(cart_vel_error_tool_target_in_world.head<3>().cwiseProduct(m_state.z)) / impedance.lugre.z_ss;
  d_dt.w = alpha.cwiseProduct(m_state.z - m_state.w) / impedance.tau_w;

  Eigen::Vector3d friction_force = impedance.lugre.sigma_0 * (m_state.z - m_state.w)
                                 + impedance.lugre.sigma_1 * d_dt.z
                                 + impedance.lugre.sigma_2 * cart_vel_error_tool_target_in_world.head<3>();

  cart_acc_tool_target_in_world.head(3) = m_inertia_inv.head(3).cwiseProduct(wrench_tool_in_world_filtered.head(3) - friction_force);
  cart_acc_tool_target_in_world.tail(3) = Eigen::Vector3d::Zero();

  m_state.z += d_dt.z * period.seconds();
  m_state.w += d_dt.w * period.seconds();
  m_state.r += d_dt.r * period.seconds();

  // Reset condition
  if(alpha.maxCoeff() > 0)
  {
    const size_t window_reset_size = (size_t) std::ceil(impedance.reset_condition.reset_window_size/(period.seconds()*1e3));
    m_reset_window.emplace_back(wrench_tool_in_world_filtered.head<3>().transpose() * cart_vel_error_tool_target_in_world.head<3>());
    if(m_reset_window.size() > window_reset_size)
    {
      m_reset_window.pop_front();
    }
    const double reset_value = std::accumulate(m_reset_window.begin(),
                                               m_reset_window.end(),
                                               0.0,
                                                 [&period](const double d, const double x) -> double
                                                 {
                                                   return d + x*period.seconds();
                                                 }
                                               );

    if(m_reset_window.size() >= window_reset_size &&
        reset_value < impedance.reset_condition.reset_threshold)
    {
      m_state.clear();
      m_reset_window.clear();
    }
    else
    {
      if(!m_reset_window.empty())
      {
        m_reset_window.clear();
      }
    }
  }

  /*
   *
   *                                                                     +-------------+
   * T_world_target                                       +---+          |             |
   * target_twist_tool_world_in_world  +----------------->+ + +--------->+    IK()     |
   *                                                      +-+-+          |             |
   *                                                        ^            +-------------+
   *                                               position |
   *                                               velocity |     +-----+
   *                                                        |     |  1  |
   *                                                        +-----+  -  +<-----+ m_elastoplastic_target_tool_in_world
   *                                                              |  s2 |
   *                                                              +-----+
   *
   *
   *
  */

  m_elastoplastic_target_tool_in_world.position += m_elastoplastic_target_tool_in_world.velocity * period.seconds() + 0.5 * cart_acc_tool_target_in_world * std::pow(period.seconds(), 2.0);
  m_elastoplastic_target_tool_in_world.velocity += cart_acc_tool_target_in_world * period.seconds();
  m_elastoplastic_target_tool_in_world.position.unaryExpr([](const double d){ return d < 1e-9? 0.0 : d;});
  m_elastoplastic_target_tool_in_world.velocity.unaryExpr([](const double d){ return d < 1e-9? 0.0 : d;});

  Eigen::Affine3d T_next_world_tool              = T_world_target * Eigen::Translation3d(m_elastoplastic_target_tool_in_world.position.head<3>());
  Eigen::Vector6d twist_next_tool_world_in_world = m_elastoplastic_target_tool_in_world.velocity + target_twist_tool_world_in_world;

  // Inverse Kinematics
  Eigen::Matrix6Xd J_G_base_tool = m_chain_base_tool->getJacobian(m_q);
  Eigen::Matrix6Xd J_G_world_tool(6, J_G_base_tool.cols() + m_float_base.nax());
  J_G_world_tool.block(0,0,6,J_G_base_tool.cols()) << J_G_base_tool;
  if(m_float_base.enabled)
  {
    J_G_world_tool.block(0, J_G_base_tool.cols(), 6, m_float_base.nax()) << m_float_base.jacobian();
  }

  // Eigen::Vector6d  distance_next_world_tool_now;
  // Eigen::Matrix66d B_AG;
  // rdyn::getFrameDistanceQuatJac(T_world_tool, T_next_world_tool, distance_next_world_tool_now, B_AG);
  // Eigen::Matrix6Xd J_A_world_tool = B_AG.inverse() * J_G_world_tool;

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd(J_G_world_tool, Eigen::ComputeThinU | Eigen::ComputeThinV);
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), fmt::format("Singular values: {}", svd.singularValues()));
  if(svd.nonzeroSingularValues() != std::min(svd.rows(), svd.cols()))
    RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1000, "SINGULARITY POINT (null singular values)");
  else if (svd.singularValues()(0)/svd.singularValues()(std::min(svd.rows(), svd.cols())-1) > 1e2)
    RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *this->get_node()->get_clock(), 1000, "SINGULARITY POINT (high conditioning number)");

  auto task_only_elastic = [&, this](const Eigen::VectorXd& p_q) -> Eigen::VectorXd {
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

  auto task_only_plastic = [&, this](const Eigen::VectorXd& p_qp) -> Eigen::VectorXd {
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
    full << Eigen::VectorXd::Zero(m_nax), result;
    return full;
  };

  // WARNING: Esiste un modo più intelligente per fare la selezione?
  // WARNING: come gestisco i giunti che non vengono usati dal task?
  auto task_selector = [&, this]() -> Eigen::VectorXd {
    return alpha.maxCoeff() > 0? task_only_plastic(m_q) : task_only_elastic(twist_base_world_in_world);
  };

  const Eigen::Vector6d Kd = Eigen::Vector6d::Constant(1.0); // FIXME: Cambia di posto
  const Eigen::Vector6d Kp = Eigen::Vector6d::Constant(1.0); // FIXME: Cambia di posto
  auto clik = [&, this]() -> Eigen::VectorXd {
    Eigen::Vector6d pose_error_tool_world_in_world;
    // rdyn::getFrameDistanceQuat(Eigen::Affine3d::Identity(),
    //     rdyn::spatialIntegration(Eigen::Affine3d::Identity(), target_twist_tool_world_in_world, period.seconds()),
    //     pose_error_tool_world_in_world);
    rdyn::getFrameDistance(T_world_tool, T_next_world_tool, pose_error_tool_world_in_world);
    Eigen::Vector6d acc_non_linear = m_chain_base_tool->getDTwistNonLinearPartTool(m_q, m_qp); // NOTE: Sicuro non ci sia nulla della base?
    Eigen::Vector6d x_task_2 = J_G_world_tool * task_selector();
    Eigen::VectorXd svd_solve = svd.solve(Kd.cwiseProduct((twist_next_tool_world_in_world - twist_tool_world_in_world))
                                          + Kp.cwiseProduct(pose_error_tool_world_in_world)
                                          - acc_non_linear
                                          - x_task_2);
    return task_selector() + svd_solve;
  };

  Eigen::VectorXd qepp = clik();

  // // Scaling due to joint velocity limits
  // double scaling_vel = 1.0;
  // for(size_t idx = 0; idx < m_nax; idx++)
  // {
  //   scaling_vel = std::max(scaling_vel,std::abs(m_qp(idx))/m_limits.vel(idx));
  // }
  // if(scaling_vel > 1)
  // {
  //   qextp = solve_tasks(scaling_vel);
  // }

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

  Eigen::Vector6d qep = Eigen::Vector6d::Zero();
  // for(int eidx : m_float_base.idxs())
  for (size_t eidx = 0; eidx < m_float_base.nax(); ++eidx)
  {
    qep(m_float_base.idxs()[eidx]) = twist_base_world_in_world(m_float_base.idxs()[eidx]) + qepp.tail(m_float_base.nax())(eidx) * period.seconds();
  }
  geometry_msgs::msg::Twist cmd_vel = Eigen::toMsg(qep);
  if(m_float_base.enabled)
  {
    m_pub_cmd_vel->publish(cmd_vel);
  }

  // *************
  // ** PUBLISH **
  // *************
  if(m_parameters.debug)
  {
    std_msgs::msg::Float64MultiArray msg_z;
    msg_z.data = std::vector<double>(m_state.z.data(), m_state.z.data() + m_state.z.size());
    m_pub_z->publish(msg_z);

    std_msgs::msg::Float64MultiArray msg_w;
    msg_w.data = std::vector<double>(m_state.w.data(), m_state.w.data() + m_state.w.size());
    m_pub_w->publish(msg_w);

    geometry_msgs::msg::WrenchStamped msg_friction_in_world;
    msg_friction_in_world.header.frame_id = m_parameters.frames.base;
    msg_friction_in_world.header.stamp = this->get_node()->get_clock()->now();
    msg_friction_in_world.wrench.force.x = friction_force[0];
    msg_friction_in_world.wrench.force.y = friction_force[1];
    msg_friction_in_world.wrench.force.z = friction_force[2];
    msg_friction_in_world.wrench.torque.x = 0.0;
    msg_friction_in_world.wrench.torque.y = 0.0;
    msg_friction_in_world.wrench.torque.z = 0.0;
    m_pub_friction_in_world->publish(msg_friction_in_world);

    geometry_msgs::msg::WrenchStamped msg_wrench_in_world;
    msg_wrench_in_world.header.frame_id = m_parameters.frames.base;
    msg_wrench_in_world.header.stamp = this->get_node()->get_clock()->now();
    msg_wrench_in_world.wrench.force.x =  wrench_tool_in_world[0];
    msg_wrench_in_world.wrench.force.y =  wrench_tool_in_world[1];
    msg_wrench_in_world.wrench.force.z =  wrench_tool_in_world[2];
    msg_wrench_in_world.wrench.torque.x = wrench_tool_in_world[3];
    msg_wrench_in_world.wrench.torque.y = wrench_tool_in_world[4];
    msg_wrench_in_world.wrench.torque.z = wrench_tool_in_world[5];
    m_pub_wrench_in_world->publish(msg_wrench_in_world);

    m_pub_cart_vel_error->publish(tf2::toMsg(cart_vel_error_tool_target_in_world));

    std_msgs::msg::Float64MultiArray q_msg;
    std_msgs::msg::Float64MultiArray qp_msg;
    q_msg.data.resize(m_nax);
    qp_msg.data.resize(m_nax);
    Eigen::VectorXd d_q = m_q - m_q_copy;
    Eigen::VectorXd d_qp = m_qp - m_qp_copy;
    std::copy(d_q.begin(), d_q.end(), q_msg.data.begin());
    std::copy(d_qp.begin(), d_qp.end(), qp_msg.data.begin());
    m_pub_pos_correction->publish(q_msg);
    m_pub_vel_correction->publish(qp_msg);
  }

  return controller_interface::return_type::OK;
}

} // namespace elastoplastic

PLUGINLIB_EXPORT_CLASS(elastoplastic::ElastoplasticController, controller_interface::ChainableControllerInterface);
