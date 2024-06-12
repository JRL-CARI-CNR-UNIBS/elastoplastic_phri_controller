#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"

#include <urdfdom_headers/urdf_model/model.h>

#include <ranges>
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

  m_sub_robot_description = this->get_node()->create_subscription<std_msgs::msg::String>("robot_description", 10, [](const std_msgs::msg::String::SharedPtr){});
  // Ultra-complicazione
  rclcpp::WaitSet wait_sub;
  wait_sub.add_subscription(m_sub_robot_description);
  auto wait_result = wait_sub.wait(std::chrono::seconds(10));
  if(wait_result.kind() != rclcpp::WaitResultKind::Ready)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "No robot description received! Interrupting configuration");
    return controller_interface::CallbackReturn::FAILURE;
  }
  std_msgs::msg::String robot_description_msg;
  rclcpp::MessageInfo msg_info;
  m_sub_robot_description->take(robot_description_msg,msg_info);

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(robot_description_msg.data);

  Eigen::Vector3d gravity({m_parameters.gravity.at(0), m_parameters.gravity.at(1), m_parameters.gravity.at(2)});
  m_chain_base_tool   = rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.tool, gravity);
  m_chain_base_sensor = rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.sensor, gravity);

  // TODO: Limits velocity, acceleration, force

  std::string what;
  m_nax = m_parameters.joints.size();
  m_chain_base_tool  ->setInputJointsName(m_parameters.joints,what);
  m_chain_base_sensor->setInputJointsName(m_parameters.joints,what);

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

  using namespace std::placeholders;
  m_sub_target_joint_trajectory = this->get_node()->create_subscription<sensor_msgs::msg::JointState>(m_parameters.target_joint_trajectory_topic, 10, std::bind(&ElastoplasticController::get_target_callback, this, _1));

  m_ft_sensor = std::make_unique<semantic_components::ForceTorqueSensor>(m_parameters.ft_sensor_name);

  m_pub_z =                this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray> (fmt::format("{}/{}", this->get_node()->get_namespace(), "z"), 10);
  m_pub_w =                this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray> (fmt::format("{}/{}", this->get_node()->get_namespace(), "w"), 10);
  m_pub_friction_in_base = this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(fmt::format("{}/{}", this->get_node()->get_namespace(), "friciton_in_base"), 10);
  m_pub_wrench_in_base =   this->get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(fmt::format("{}/{}", this->get_node()->get_namespace(), "wrench_in_base"), 10);
  m_pub_pose_in_base =     this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>  (fmt::format("{}/{}", this->get_node()->get_namespace(), "pose_in_base"), 10);
  m_pub_target_in_base =   this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>  (fmt::format("{}/{}", this->get_node()->get_namespace(), "target_in_base"), 10);

  m_has_interfaces.state.position() = std::ranges::find(m_parameters.state_interfaces, hardware_interface::HW_IF_POSITION) == m_parameters.state_interfaces.end();
  m_has_interfaces.state.velocity() = std::ranges::find(m_parameters.state_interfaces, hardware_interface::HW_IF_VELOCITY) == m_parameters.state_interfaces.end();
  m_has_interfaces.state.effort  () = std::ranges::find(m_parameters.state_interfaces, hardware_interface::HW_IF_EFFORT)   == m_parameters.state_interfaces.end();

  m_has_interfaces.command.position() = std::ranges::find(m_parameters.command_interfaces, hardware_interface::HW_IF_POSITION) == m_parameters.command_interfaces.end();
  m_has_interfaces.command.velocity() = std::ranges::find(m_parameters.command_interfaces, hardware_interface::HW_IF_VELOCITY) == m_parameters.command_interfaces.end();
  m_has_interfaces.state.effort    () = std::ranges::find(m_parameters.command_interfaces, hardware_interface::HW_IF_EFFORT)   == m_parameters.command_interfaces.end();

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

  state_interface_configuration.names.reserve(m_parameters.joints.size() * std::ranges::count(m_has_interfaces.state, true) + 6);

  for(const auto& jnt : m_parameters.joints)
  {
    if(m_has_interfaces.state.position()) state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION));
    if(m_has_interfaces.state.velocity()) state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
    if(m_has_interfaces.state.effort  ()) state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_EFFORT))  ;
  }
  std::vector<std::string> ft_interfaces = m_ft_sensor->get_state_interface_names();
  state_interface_configuration.names.insert(state_interface_configuration.names.end(), ft_interfaces.begin(), ft_interfaces.end());

  return state_interface_configuration;
}

controller_interface::InterfaceConfiguration ElastoplasticController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interface_configuration;
  command_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interface_configuration.names.reserve(m_parameters.joints.size() * std::ranges::count(m_has_interfaces.command, true) + 6);
  for(const auto& jnt : m_parameters.joints)
  {
    if(m_has_interfaces.command.position()) command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION));
    if(m_has_interfaces.command.velocity()) command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
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
  m_cart_tool_in_base.clear();

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
  auto command_interface = m_joint_state_interfaces.begin();
  for(const auto& interface : m_allowed_interface_types)
  {
    if(not controller_interface::get_ordered_interfaces(state_interfaces_, m_parameters.joints, interface, *command_interface))
    {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Missing joints command interfaces are required");
      return controller_interface::CallbackReturn::FAILURE;
    }
    command_interface = std::next(state_interface);
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
  m_cart_tool_in_base.clear();

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> ElastoplasticController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  reference_interfaces.reserve(m_parameters.joints.size() * std::ranges::count(m_has_interfaces.command, true) + 6);
  size_t idxs = 0;
  for(const auto& jnt : m_parameters.joints)
  {
    if(m_has_interfaces.command.position())
    {
      reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION), &reference_interfaces_[idxs]));
      ++idxs;
    }
  }
  for(const auto& jnt : m_parameters.joints)
  {
    if(m_has_interfaces.command.velocity())
    {
      reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY), &reference_interfaces_[idxs]));
      ++idxs;
    }
  }

  return reference_interfaces;
}

controller_interface::return_type ElastoplasticController::update_reference_from_subscribers()
{
  sensor_msgs::msg::JointState::SharedPtr msg = *(m_joint_trajectory_buffer.readFromRT());
  for(auto iter = reference_interfaces_.begin(); std::next(iter,m_parameters.joints.size()) != reference_interfaces_.end(); ++iter)
  {
    if(m_has_interfaces.command.position())
    {
      *iter = msg->position.at(std::distance(iter, reference_interfaces_.begin()));
    }
    if(m_has_interfaces.command.velocity())
    {
      *std::next(iter,m_parameters.joints.size()) = msg->velocity.at(std::distance(iter, reference_interfaces_.begin()));
    }
  }

  return controller_interface::return_type::OK;
}

void ElastoplasticController::get_target_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  m_joint_trajectory_buffer.writeFromNonRT(msg);
}

controller_interface::return_type ElastoplasticController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // **********
  // ** Read **
  // **********
  Eigen::VectorXd target   = Eigen::Map<Eigen::VectorXd>(reference_interfaces_.data(), m_parameters.joints.size());
  Eigen::VectorXd target_p = Eigen::Map<Eigen::VectorXd>(reference_interfaces_.data()+m_parameters.joints.size(), m_parameters.joints.size());

  std::array<double, 3> input_force_in_sensor = m_ft_sensor->get_forces();
  Eigen::Vector6d wrench_sensor_in_sensor= Eigen::Map<Eigen::Vector6d>(input_force_in_sensor.data(), input_force_in_sensor.size());

  // ************
  // ** Update **
  // ************
  Eigen::Affine3d  T_base_target = m_chain_base_tool->getTransformation(target);
  Eigen::Matrix6Xd J_base_target = m_chain_base_tool->getJacobian(target);
  Eigen::Vector6d  cart_vel_target_in_base = J_base_target * target_p;

  Eigen::Affine3d  T_base_tool = m_chain_base_tool->getTransformation(m_q);
  Eigen::Matrix6Xd J_base_tool = m_chain_base_tool->getJacobian(m_q);
  Eigen::Vector6d  cart_vel_tool_in_base = J_base_tool * m_qp;

  Eigen::Vector6d cart_vel_error_in_base = cart_vel_tool_in_base - cart_vel_target_in_base;
  Eigen::Vector6d cart_acc_tool_in_base = Eigen::Vector6d::Zero();

  Eigen::Vector6d cart_error_target_in_base;
  rdyn::getFrameDistance(T_base_tool, T_base_target, cart_error_target_in_base);

  Eigen::Affine3d T_base_sensor = m_chain_base_sensor->getTransformation(m_q);
  Eigen::Affine3d T_tool_sensor = T_base_tool.inverse() * T_base_sensor;
  Eigen::Vector6d wrench_tool_in_tool = rdyn::spatialDualTranformation(wrench_sensor_in_sensor, T_tool_sensor);
  Eigen::Vector6d wrench_tool_in_base = rdyn::spatialRotation(wrench_tool_in_tool, T_base_tool.linear());
  Eigen::Vector6d wrench_tool_in_base_filtered;
  wrench_tool_in_base_filtered.head<3>() = wrench_tool_in_base.head<3>().unaryExpr([this](double w){return std::abs(w) > m_parameters.wrench_deadband.at(0)? w : 0.0;});
  wrench_tool_in_base_filtered.tail<3>() = wrench_tool_in_base.tail<3>().unaryExpr([this](double w){return std::abs(w) > m_parameters.wrench_deadband.at(1)? w : 0.0;});

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
  double dr = compute_dalpha(m_state.z.norm());
  Eigen::Vector3d dz = cart_error_target_in_base.head<3>() - alpha.cwiseProduct(cart_vel_error_in_base.head<3>().cwiseProduct(m_state.z)) / impedance.lugre.z_ss;
  Eigen::Vector3d dw = alpha.cwiseProduct(m_state.z - m_state.w) / impedance.tau_w;

  Eigen::Vector3d friction_force = impedance.lugre.sigma_0 * (m_state.z - m_state.w) +
                                   impedance.lugre.sigma_1 * dz +
                                   impedance.lugre.sigma_2 * cart_vel_error_in_base.head<3>();

  cart_acc_tool_in_base.head(3) = m_inertia_inv.head(3).cwiseProduct(wrench_tool_in_base_filtered.head(3) - friction_force);
  cart_acc_tool_in_base.tail(3) = Eigen::Vector3d::Zero();

  m_state.z += dz * period.seconds();
  m_state.w += dw * period.seconds();
  m_state.r += dr * period.seconds();

  // Reset condition
  if(alpha.maxCoeff() > 0)
  {
    const size_t window_reset_size = (size_t) std::ceil(impedance.reset_condition.reset_window_size/(period.seconds()*1e3));
    m_reset_window.emplace_back(wrench_tool_in_base_filtered.head<3>().transpose() * cart_vel_error_in_base.head<3>());
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


  m_cart_tool_in_base.position += m_cart_tool_in_base.velocity * period.seconds() + 0.5 * cart_acc_tool_in_base * std::pow(period.seconds(), 2.0);
  m_cart_tool_in_base.velocity += cart_acc_tool_in_base * period.seconds();

  Eigen::Affine3d T_base_tool_next     = Eigen::Translation3d(m_cart_tool_in_base.position.head<3>()) * T_base_target;
  Eigen::Vector6d twist_base_tool_next = m_cart_tool_in_base.velocity + cart_vel_target_in_base;

  m_chain_base_tool->computeLocalIk(m_q, T_base_tool_next, m_q);
  Eigen::Matrix6Xd J_base_tool_next = m_chain_base_tool->getJacobian(m_q);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_base_tool_next, Eigen::ComputeThinU | Eigen::ComputeThinV);

  if (svd.singularValues()(svd.cols()-1)==0)
    RCLCPP_ERROR(this->get_node()->get_logger(), "SINGULARITY POINT");
  else if (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2)
    RCLCPP_ERROR(this->get_node()->get_logger(), "SINGULARITY POINT");

  m_qp = svd.solve(twist_base_tool_next);

  // Scaling due to joint velocity limits
  double scaling_vel = 1.0;
  for(size_t idx = 0; idx < m_nax; idx++)
  {
    scaling_vel = std::max(scaling_vel,std::abs(m_qp(idx))/m_limits.vel(idx));
  }
  if(scaling_vel > 1)
  {
    m_qp = svd.solve(twist_base_tool_next/scaling_vel);
  }

  // Saturation
  for(size_t idx = 0; idx < m_nax; ++idx)
  {
    m_q(idx) =  std::max(m_limits.pos_lower(idx), std::min(m_limits.pos_upper(idx), m_q(idx)));
    m_qp(idx) = std::max(-m_limits.vel(idx),      std::min(m_limits.vel(idx), m_qp(idx)));
  }

  // ***********
  // ** Write **
  // ***********
  for(size_t ax = 0; ax < m_nax; ++ax)
  {
    if(m_has_interfaces.command.at(0)) m_joint_command_interfaces.at(0).at(ax).get().set_value(m_q(ax));
    if(m_has_interfaces.command.at(1)) m_joint_command_interfaces.at(1).at(ax).get().set_value(m_qp(ax));
  }

  return controller_interface::return_type::OK;
}

} // namespace elastoplastic
