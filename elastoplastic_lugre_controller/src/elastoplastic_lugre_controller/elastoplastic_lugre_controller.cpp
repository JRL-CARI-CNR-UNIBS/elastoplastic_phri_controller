#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <urdfdom_headers/urdf_model/model.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <fmt/core.h>

namespace elastoplastic {

controller_interface::CallbackReturn ElastoplasticController::on_init()
{
  m_param_listener = std::make_shared<elastoplastic_controller::ParamListener>(this->get_node());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElastoplasticController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  m_parameters = m_param_listener->get_params();

  std::string robot_description = this->get_node()->get_parameter("robot_description").as_string();
  if(robot_description == "")
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Missing robot_description by controller_manager");
    return controller_interface::CallbackReturn::FAILURE;
  }
  else
  {
    RCLCPP_INFO(this->get_node()->get_logger(), "%s", "robot_description obtained correctly");
  }

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(robot_description);
  if(not urdf_model)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot create URDF model from robot_description provided by controller_manager");
    return controller_interface::CallbackReturn::FAILURE;
  }

  Eigen::Vector3d gravity({m_parameters.gravity.at(0), m_parameters.gravity.at(1), m_parameters.gravity.at(2)});
  m_chain_base_tool   = rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.tool, gravity);
  m_chain_base_sensor = rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.sensor, gravity);
  if(not m_chain_base_tool)
  {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot create rdyn chain from base (%s) to tool (%s)",m_parameters.frames.base.c_str(), m_parameters.frames.tool.c_str());
      return controller_interface::CallbackReturn::FAILURE;
  }
  if(not m_chain_base_sensor)
  {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot create rdyn chain from base (%s) to sensor (%s)",m_parameters.frames.base.c_str(), m_parameters.frames.sensor.c_str());
      return controller_interface::CallbackReturn::FAILURE;
  }

  // TODO: Limits velocity, acceleration, force

  std::string what;
  m_nax = m_parameters.joints.size();
  m_chain_base_tool  ->setInputJointsName(m_parameters.joints, what);
  m_chain_base_sensor->setInputJointsName(m_parameters.joints, what);

  m_q.resize(m_nax);
  m_qp.resize(m_nax);
  m_qpp.resize(m_nax);

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

  if(std::ranges::min(m_parameters.impedance.inertia) < 0)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Inertia has negative values!");
    return controller_interface::CallbackReturn::FAILURE;
  }
  m_inertia_inv = Eigen::Map<Eigen::Vector3d>(m_parameters.impedance.inertia.data(), m_parameters.impedance.inertia.size()).cwiseInverse();


  if(!m_float_base.init(m_parameters.floating_base_joints.names, m_parameters.floating_base_joints.enabled))
  {
    RCLCPP_FATAL(this->get_node()->get_logger(), "You should not be here due to parameters validation");
    return controller_interface::CallbackReturn::ERROR;
  }

  using namespace std::placeholders;
  m_sub_target_twist_tool_in_base = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(m_parameters.target_joint_trajectory_topic, 1, std::bind(&ElastoplasticController::get_target_callback, this, _1));
  m_sub_fb_target = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(m_parameters.floating_base_joints.target_topic, 1, std::bind(&ElastoplasticController::get_fb_target_callback, this, _1));
  m_sub_base_pose_in_world = this->get_node()->create_subscription<nav_msgs::msg::Odometry>(m_parameters.floating_base_joints.odom, 1, std::bind(&ElastoplasticController::get_pose_in_world_callback, this, _1));

  m_ft_sensor = std::make_unique<semantic_components::ForceTorqueSensor>(m_parameters.ft_sensor_name);

  m_pub_z =                this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray> (fmt::format("{}/{}", this->get_node()->get_namespace(), "z"), 10);
  m_pub_w =                this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray> (fmt::format("{}/{}", this->get_node()->get_namespace(), "w"), 10);
  m_pub_friction_in_base = this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(fmt::format("{}/{}", this->get_node()->get_namespace(), "friciton_in_base"), 10);
  m_pub_wrench_in_base =   this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(fmt::format("{}/{}", this->get_node()->get_namespace(), "wrench_in_base"), 10);

  m_has_interfaces.state.position() = not (std::ranges::find(m_parameters.state_interfaces, hardware_interface::HW_IF_POSITION) == m_parameters.state_interfaces.end());
  m_has_interfaces.state.velocity() = not (std::ranges::find(m_parameters.state_interfaces, hardware_interface::HW_IF_VELOCITY) == m_parameters.state_interfaces.end());
  m_has_interfaces.state.effort  () = not (std::ranges::find(m_parameters.state_interfaces, hardware_interface::HW_IF_EFFORT)   == m_parameters.state_interfaces.end());

  m_has_interfaces.command.position() = not (std::ranges::find(m_parameters.command_interfaces, hardware_interface::HW_IF_POSITION) == m_parameters.command_interfaces.end());
  m_has_interfaces.command.velocity() = not (std::ranges::find(m_parameters.command_interfaces, hardware_interface::HW_IF_VELOCITY) == m_parameters.command_interfaces.end());
  m_has_interfaces.state.effort    () = not (std::ranges::find(m_parameters.command_interfaces, hardware_interface::HW_IF_EFFORT)   == m_parameters.command_interfaces.end());

  if(not m_has_interfaces.check())
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "No State or Command interfaces required from parameters");
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ElastoplasticController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interface_configuration.names.reserve(m_parameters.joints.size() * std::ranges::count(m_has_interfaces.state, true) + m_float_base.enabled().size() + 6);

  for(const auto& jnt : m_parameters.joints)
  {
    if(m_has_interfaces.state.position()) state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION));
    if(m_has_interfaces.state.velocity()) state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
    if(m_has_interfaces.state.effort  ()) state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_EFFORT))  ;
  }
  for(size_t idx : m_float_base.enabled())
  {
    state_interface_configuration.names.emplace_back(fmt::format("{}/{}/{}", m_float_base.ns(), m_float_base.name(idx),m_parameters.floating_base_joints.interfaces.state_interface));
  }

  std::vector<std::string> ft_interfaces = m_ft_sensor->get_state_interface_names();
  state_interface_configuration.names.insert(state_interface_configuration.names.end(), ft_interfaces.begin(), ft_interfaces.end());

  return state_interface_configuration;
}

controller_interface::InterfaceConfiguration ElastoplasticController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interface_configuration;
  command_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interface_configuration.names.reserve(m_parameters.joints.size() * std::ranges::count(m_has_interfaces.command, true) + m_float_base.enabled().size());
  for(const auto& jnt : m_parameters.joints)
  {
    if(m_has_interfaces.command.position()) command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION));
    if(m_has_interfaces.command.velocity()) command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
  }
  for(size_t idx : m_float_base.enabled())
  {
    command_interface_configuration.names.emplace_back(fmt::format("{}/{}/{}", m_float_base.ns(), m_float_base.name(idx), m_parameters.floating_base_joints.interfaces.command_interface));
  }

  return command_interface_configuration;
}

controller_interface::CallbackReturn ElastoplasticController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  m_q  .setZero();
  m_qp .setZero();
  m_qpp.setZero();

  m_state.clear();
  m_reset_window.clear();
  m_elastoplastic_target_tool_in_world.clear();

  // Init RT buffers?

  auto state_interface = m_joint_state_interfaces.begin();
  for(const auto& interface : m_allowed_interface_types)
  {
    if(not controller_interface::get_ordered_interfaces(state_interfaces_, m_parameters.joints, interface, *state_interface))
    {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Missing joints state interfaces are required");
      return controller_interface::CallbackReturn::FAILURE;
    }
    state_interface = std::next(state_interface);
  }
  auto command_interface = m_joint_command_interfaces.begin();
  for(const auto& interface : m_allowed_interface_types)
  {
    if(not controller_interface::get_ordered_interfaces(command_interfaces_, m_parameters.joints, interface, *command_interface))
    {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Missing joints command interfaces are required");
      return controller_interface::CallbackReturn::FAILURE;
    }
    command_interface = std::next(command_interface);
  }

  auto fb_state_interface = m_fb_state_interfaces.begin();
  if(not controller_interface::get_ordered_interfaces(state_interfaces_, m_parameters.joints, m_parameters.floating_base_joints.interfaces.state_interface, *fb_state_interface))
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Missing floating base joints state interfaces are required");
    return controller_interface::CallbackReturn::FAILURE;
  }
  auto fb_command_interface = m_fb_command_interfaces.begin();
  if(not controller_interface::get_ordered_interfaces(command_interfaces_, m_parameters.joints, m_parameters.floating_base_joints.interfaces.command_interface, *fb_command_interface))
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Missing floating base joints command interfaces are required");
    return controller_interface::CallbackReturn::FAILURE;
  }

  m_ft_sensor->assign_loaned_state_interfaces(state_interfaces_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElastoplasticController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  m_q  .setZero();
  m_qp .setZero();
  m_qpp.setZero();

  m_state.clear();
  m_reset_window.clear();
  m_elastoplastic_target_tool_in_world.clear();

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> ElastoplasticController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  reference_interfaces.reserve(6);
  reference_interfaces_.resize(6);
  for(size_t idx = 0; idx < 6; idx++)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), fmt::format("{}/{}", m_parameters.reference_interfaces.at(idx), hardware_interface::HW_IF_VELOCITY), &reference_interfaces_[idx]));
  }

  return reference_interfaces;
}

controller_interface::return_type ElastoplasticController::update_reference_from_subscribers()
{
  geometry_msgs::msg::Twist* msg = m_rt_buffer_twist_tool_in_base.readFromRT();
  reference_interfaces_.at(0) = msg->linear.x;
  reference_interfaces_.at(1) = msg->linear.y;
  reference_interfaces_.at(2) = msg->linear.z;
  reference_interfaces_.at(3) = msg->angular.x;
  reference_interfaces_.at(4) = msg->angular.y;
  reference_interfaces_.at(5) = msg->angular.z;


  return controller_interface::return_type::OK;
}

void ElastoplasticController::get_target_callback(const geometry_msgs::msg::Twist& msg)
{
  m_rt_buffer_twist_tool_in_base.writeFromNonRT(msg);
}

void ElastoplasticController::get_fb_target_callback(const geometry_msgs::msg::Twist& msg)
{
  // Always from topic
  m_rt_buffer_fb_target.writeFromNonRT(msg);
}

void ElastoplasticController::get_pose_in_world_callback(const nav_msgs::msg::Odometry& msg)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg2;
  msg2.header = msg.header;
  msg2.pose = msg.pose;
  m_rt_buffer_base_pose_in_world.writeFromNonRT(msg2);
}

controller_interface::return_type ElastoplasticController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // WARNING: Porta tutto in world

  // **********
  // ** Read **
  // **********
  geometry_msgs::msg::PoseWithCovarianceStamped* msg_pose_base_in_world = m_rt_buffer_base_pose_in_world.readFromRT();
  Eigen::Vector6d twist_base_world_in_world;
  Eigen::Affine3d T_world_base;
  Eigen::fromMsg(msg_pose_base_in_world->pose.pose, T_world_base);

  Eigen::Vector6d target_twist_tool_base_in_base = Eigen::Map<Eigen::Vector6d>(reference_interfaces_.data(), reference_interfaces_.size());

  auto from_base_to_world = [&T_world_base](const Eigen::Vector6d& v){
    return rdyn::spatialRotation(v, T_world_base.linear());
  };

  Eigen::Vector6d target_twist_tool_base_in_world = from_base_to_world(target_twist_tool_base_in_base);


  Eigen::Vector6d target_twist_base_world_in_base, target_twist_base_world_in_world;
  Eigen::fromMsg(*(m_rt_buffer_fb_target.readFromRT()), target_twist_base_world_in_base);
  target_twist_base_world_in_world = from_base_to_world(target_twist_base_world_in_base);

  std::array<double, 3> input_force_in_sensor = m_ft_sensor->get_forces();
  Eigen::Vector6d wrench_sensor_in_sensor= Eigen::Map<Eigen::Vector6d>(input_force_in_sensor.data(), input_force_in_sensor.size());

  // Actual state
  for(size_t idx = 0, jdx = 0; jdx < m_float_base.enabled().size(); ++idx)
  {
    m_float_base.twist()(idx) = m_float_base.enabled(idx) ? m_fb_state_interfaces.at(0).at(jdx).get().get_value() : 0.0;
    ++jdx;
  }
  twist_base_world_in_world = from_base_to_world(m_float_base.twist());

  Eigen::VectorXd target_twist_tool_world_in_world = target_twist_tool_base_in_world + target_twist_base_world_in_world;

  // ************
  // ** Update **
  // ************
  // FIXME: m_q or get from state interface?
  Eigen::Affine3d  T_base_tool = m_chain_base_tool->getTransformation(m_q);
  Eigen::Affine3d  T_world_tool = T_world_base * T_base_tool;
  Eigen::Matrix6Xd J_base_tool_in_base = m_chain_base_tool->getJacobian(m_q);
  Eigen::Vector6d  twist_tool_base_in_base = J_base_tool_in_base * m_qp;
  Eigen::Vector6d  twist_tool_world_in_base = twist_tool_base_in_base + m_float_base.twist();
  Eigen::Vector6d  twist_tool_world_in_world = from_base_to_world(twist_tool_world_in_base);

  Eigen::Vector6d cart_vel_error_tool_target_in_world = twist_tool_world_in_world - target_twist_tool_world_in_world;
  Eigen::Vector6d cart_acc_tool_target_in_world = Eigen::Vector6d::Zero();

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
    const double& z_ss = m_parameters.impedance.lugre.z_ba;
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
    const double& z_ss = m_parameters.impedance.lugre.z_ba;
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

  Eigen::Vector3d friction_force = impedance.lugre.sigma_0 * (m_state.z - m_state.w) +
                                   impedance.lugre.sigma_1 * d_dt.z +
                                   impedance.lugre.sigma_2 * cart_vel_error_tool_target_in_world.head<3>();

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

  m_elastoplastic_target_tool_in_world.position += m_elastoplastic_target_tool_in_world.velocity * period.seconds() + 0.5 * cart_acc_tool_target_in_world * std::pow(period.seconds(), 2.0);
  m_elastoplastic_target_tool_in_world.velocity += cart_acc_tool_target_in_world * period.seconds();

  Eigen::Affine3d T_next_world_tool              = T_world_target * Eigen::Translation3d(m_elastoplastic_target_tool_in_world.position.head<3>());
  Eigen::Vector6d twist_next_tool_world_in_world = m_elastoplastic_target_tool_in_world.velocity + target_twist_tool_world_in_world;

  // Inverse Kinematics
  Eigen::Matrix6Xd J_G_base_tool = m_chain_base_tool->getJacobian(m_q);
  Eigen::Matrix6Xd J_G_world_tool; J_G_world_tool << J_G_base_tool, m_float_base.jacobian();

  Eigen::Vector6d  distance_next_world_tool_now;
  Eigen::Matrix66d jacobian_quat;
  rdyn::getFrameDistanceQuatJac(T_world_tool, T_next_world_tool, distance_next_world_tool_now, jacobian_quat);
  Eigen::Matrix6Xd J_A_world_tool = jacobian_quat.inverse() * J_G_world_tool;

  // Eigen::Matrix6Xd J_A_extended_base_tool(6, m_nax + m_float_base.enabled().size());
  // J_A_extended_base_tool << J_A_world_tool, m_float_base.jacobian();

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_A_world_tool, Eigen::ComputeThinU | Eigen::ComputeThinV);

  if (svd.singularValues()(svd.cols()-1)==0)
    RCLCPP_ERROR(this->get_node()->get_logger(), "SINGULARITY POINT");
  else if (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2)
    RCLCPP_ERROR(this->get_node()->get_logger(), "SINGULARITY POINT");

  auto task_only_plastic = [&, this](const Eigen::VectorXd& p_q) -> Eigen::VectorXd {
    constexpr double kp = 1.0;
    Eigen::VectorXd q = p_q.head(m_nax);
    Eigen::VectorXd full(m_nax + m_float_base.enabled().size());
    Eigen::VectorXd result = Eigen::VectorXd::Zero(p_q.size());
    for(long int idx = 0; idx < q.size(); ++idx)
    {
      result(idx) = - kp/m_nax * (q(idx) - 0.5*(this->m_limits.pos_upper(idx) + this->m_limits.pos_lower(idx))) / (this->m_limits.pos_upper(idx) - this->m_limits.pos_lower(idx));
    }
    full << result, Eigen::VectorXd::Zero(m_float_base.enabled().size());
    return full;
  };

  auto task_only_elastic = [&, this](const Eigen::VectorXd& p_qp) -> Eigen::VectorXd {
    constexpr double kp = 1.0; // FIXME: Cambiare di posto
    Eigen::VectorXd full(m_nax + m_float_base.enabled().size());
    Eigen::VectorXd result = - kp * (p_qp.tail(this->m_float_base.enabled().size()) - target_twist_base_world_in_base(this->m_float_base.enabled()));
    full << Eigen::VectorXd::Zero(m_nax), result;
    return full;
  };

  // WARNING: Esiste un modo più intelligente per fare la selezione?
  // WARNING: come gestisco i giunti che non vengono usati dal task?
  auto task_selector = [&, this]() -> Eigen::VectorXd { // WARNING: Il task elastico non restituisce un'accelerazione?
    return alpha.maxCoeff() > 0? task_only_plastic(m_q) : task_only_elastic(twist_base_world_in_world);
  };

  const Eigen::Vector6d Kd = Eigen::Vector6d::Constant(1.0); // FIXME: Cambia di posto
  const Eigen::Vector6d Kp = Eigen::Vector6d::Constant(1.0); // FIXME: Cambia di posto
  auto solve_tasks = [&, this]() -> Eigen::VectorXd {
    Eigen::Vector6d pose_error_tool_world_in_world;
    rdyn::getFrameDistanceQuat(Eigen::Affine3d::Identity(), rdyn::spatialIntegration(Eigen::Affine3d::Identity(), target_twist_base_world_in_world, period.seconds()), pose_error_tool_world_in_world);
    Eigen::Vector6d acc_non_linear = m_chain_base_tool->getDTwistNonLinearPartTool(m_q, m_qp); // NOTE: Sicuro non ci sia nulla della base?
    Eigen::Vector6d x_task_2 = J_A_world_tool * task_selector();
    return task_selector() + svd.solve(Kd.cwiseProduct(twist_next_tool_world_in_world - twist_tool_world_in_world) + Kp.cwiseProduct(pose_error_tool_world_in_world) - acc_non_linear - x_task_2);
  };

  Eigen::VectorXd qepp = solve_tasks();

  // Scaling due to joint velocity limits
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
  for(size_t ax = 0; ax < m_nax; ++ax)
  {
    if(m_has_interfaces.command.position()) m_joint_command_interfaces.at(0).at(ax).get().set_value(m_q(ax));
    if(m_has_interfaces.command.velocity()) m_joint_command_interfaces.at(1).at(ax).get().set_value(m_qp(ax));
  }

  Eigen::VectorXd qep = Eigen::VectorXd::Zero(m_float_base.enabled().size());
  for(size_t idx = 0; idx < m_float_base.enabled().size(); ++idx)
  {
    qep(idx) = twist_base_world_in_world(m_float_base.enabled().at(idx)) + qepp.tail(m_float_base.enabled().size())(idx) * period.seconds();
    m_fb_command_interfaces.at(0).at(idx).get().set_value(qep(idx));
  }

  // *************
  // ** PUBLISH **
  // *************
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
  m_pub_friction_in_base->publish(msg_friction_in_world);

  geometry_msgs::msg::WrenchStamped msg_wrench_in_world;
  msg_wrench_in_world.header.frame_id = m_parameters.frames.base;
  msg_wrench_in_world.header.stamp = this->get_node()->get_clock()->now();
  msg_wrench_in_world.wrench.force.x =  wrench_tool_in_world[0];
  msg_wrench_in_world.wrench.force.y =  wrench_tool_in_world[1];
  msg_wrench_in_world.wrench.force.z =  wrench_tool_in_world[2];
  msg_wrench_in_world.wrench.torque.x = wrench_tool_in_world[3];
  msg_wrench_in_world.wrench.torque.y = wrench_tool_in_world[4];
  msg_wrench_in_world.wrench.torque.z = wrench_tool_in_world[5];
  m_pub_friction_in_base->publish(msg_wrench_in_world);

  return controller_interface::return_type::OK;
}

} // namespace elastoplastic

PLUGINLIB_EXPORT_CLASS(elastoplastic::ElastoplasticController, controller_interface::ChainableControllerInterface);
