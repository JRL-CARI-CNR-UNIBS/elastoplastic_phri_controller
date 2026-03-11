#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_world/types.h>

#include <algorithm>
#include <cmath>
#include <control_toolbox/filters.hpp>
#include <elastoplastic_msgs/msg/admittance_hqp_controller_state.hpp>
#include <elastoplastic_msgs/msg/detail/elastoplastic_controller_state__struct.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/multibody/joint/joint-generic.hpp>
#include <pinocchio/multibody/joint/joint-planar.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <rclcpp/logging.hpp>

#include "control_toolbox/filters.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"
#include "urdfdom_headers/urdf_model/model.h"  // IWYU pragma: export

#define GET_VALUE_FROM_INTERFACE(interface) interface.get_value()

using namespace std::chrono_literals;

namespace elastoplastic
{
using namespace elastoplastic;
namespace utils
{


Eigen::Affine3d integratePoseLocalWorldAligned(
    const pinocchio::SE3 & pose,
    const pinocchio::Motion & twist_lwa,
    const double dt)
{
  // LOCAL_WORLD_ALIGNED:
  // - origin: moving body/frame origin
  // - axes: aligned with WORLD
  //
  // So:
  //   angular() is a world-aligned angular velocity
  //   linear()  is the world-aligned velocity of the body origin

  const Eigen::Vector3d omega_world = twist_lwa.angular();
  const Eigen::Vector3d v_world     = twist_lwa.linear();

  // Integrate orientation with a left-multiplied SO(3) increment
  const Eigen::Matrix3d R_next =
      pinocchio::exp3(omega_world * dt) * pose.rotation();

  // Integrate translation directly in world coordinates
  const Eigen::Vector3d p_next =
      pose.translation() + v_world * dt;

  return Eigen::Affine3d(pinocchio::SE3(R_next, p_next).toHomogeneousMatrix());
}

void toWrenchMsg(const Eigen::Vector6d & v, geometry_msgs::msg::Wrench & msg)
{
  // msg.force = tf2::toMsg2(v.head<3>());
  // msg.torque = tf2::toMsg2(v.tail<3>());
  msg.force.x = v(0);
  msg.force.y = v(1);
  msg.force.z = v(2);
  msg.torque.x = v(3);
  msg.torque.y = v(4);
  msg.torque.z = v(5);
}

geometry_msgs::msg::Wrench toWrenchMsg(const Eigen::Vector6d & v)
{
  geometry_msgs::msg::Wrench msg;
  toWrenchMsg(v, msg);
  return msg;
}

geometry_msgs::msg::WrenchStamped toWrenchStampedMsg(const Eigen::Vector6d & v)
{
  geometry_msgs::msg::WrenchStamped msg;
  toWrenchMsg(v, msg.wrench);
  return msg;
}

void deadband(const double low, const double high, Eigen::Vector6d & wr)
{
  for (int idx = 0; idx < 3; idx++) {
    if (std::abs(wr(idx)) < low) {
      wr(idx) = 0;
    } else {
      wr(idx) = elastoplastic::utils::sgn(wr(idx)) * (std::abs(wr(idx)) - low);
    }
  }
  for (int idx = 3; idx < 6; idx++) {
    if (std::abs(wr(idx)) < high) {
      wr(idx) = 0;
    } else {
      wr(idx) = elastoplastic::utils::sgn(wr(idx)) * (std::abs(wr(idx)) - high);
    }
  }
}
}  // namespace utils

void ElastoplasticController::build_pinocchio_model(
  urdf::ModelInterfaceSharedPtr & urdf_model, bool attach_mobile_base)
{
  // == Pinocchio models ==
  RCLCPP_INFO(get_node()->get_logger(), "Start pinocchio chain creation");
  // === Full model
  pinocchio::Model chain_world_tool_full;
  pinocchio::urdf::buildModel(urdf_model, pinocchio::JointModelPlanar(), chain_world_tool_full);

  RCLCPP_INFO(get_node()->get_logger(), "== Pinocchio full model built");
  m_planar_joint_name = chain_world_tool_full.names[1];

  // Get unused joint names
  RCLCPP_INFO(get_node()->get_logger(), "== Get unused joint names");
  std::vector<std::string> joint_names_to_lock = chain_world_tool_full.names;
  unsigned int root_jnts_to_lock =
    attach_mobile_base ? 2 : 1;  // In theory, I could simply avoid adding the planar joint
  joint_names_to_lock.erase(
    joint_names_to_lock.begin(),
    joint_names_to_lock.begin() +
      root_jnts_to_lock);  // Remove universe (and maybe planar) from list
  joint_names_to_lock.erase(
    std::remove_if(
      joint_names_to_lock.begin(), joint_names_to_lock.end(),
      [this](const std::string & s) {
        return std::find(m_parameters.joints.begin(), m_parameters.joints.end(), s) !=
               m_parameters.joints.end();
      }),
    joint_names_to_lock.end());

  // Get unused joint ids
  RCLCPP_INFO(get_node()->get_logger(), "== Get unused joint ids");
  std::vector<pinocchio::JointIndex> joint_id_to_lock(joint_names_to_lock.size());
  std::transform(
    joint_names_to_lock.begin(), joint_names_to_lock.end(), joint_id_to_lock.begin(),
    [&chain_world_tool_full](const auto & s) { return chain_world_tool_full.getJointId(s); });

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Discarding joints: ");
  for (size_t idx = 0; idx < joint_names_to_lock.size(); idx++) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "== name: " << joint_names_to_lock[idx]);
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "== id: " << joint_id_to_lock[idx]);
  }

  RCLCPP_INFO(get_node()->get_logger(), "== Build reduced model");
  m_model = pinocchio::buildReducedModel(
    chain_world_tool_full, joint_id_to_lock, pinocchio::neutral(chain_world_tool_full));

  RCLCPP_INFO(get_node()->get_logger(), "== Gravity");
  Eigen::Vector3d gravity(
    {m_parameters.gravity.at(0), m_parameters.gravity.at(1), m_parameters.gravity.at(2)});
  m_model.gravity.linear() = gravity;  //pinocchio::Model::gravity981;

  RCLCPP_INFO(get_node()->get_logger(), "== Get data");
  m_model_data = pinocchio::Data(m_model);

  RCLCPP_INFO(get_node()->get_logger(), "Pinocchio joints:");
  std::for_each(m_model.names.begin(), m_model.names.end(), [this](const std::string & s) {
    RCLCPP_INFO(get_node()->get_logger(), "joint: %s", s.c_str());
  });

  m_tool_id = m_model.getFrameId(m_parameters.frames.tool);
  if (static_cast<unsigned int>(m_model.nframes) == m_tool_id) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Frame %s does not exist in the model",
      m_parameters.frames.tool.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  m_sensor_id = m_model.getFrameId(m_parameters.frames.sensor);
  if (static_cast<unsigned int>(m_model.nframes) == m_sensor_id) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Frame %s does not exist in the model",
      m_parameters.frames.sensor.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  m_base_id = m_model.getFrameId(m_parameters.frames.base);
  if (static_cast<unsigned int>(m_model.nframes) == m_base_id) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Frame %s does not exist in the model",
      m_parameters.frames.base.c_str());
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  m_jnt_id.reserve(m_parameters.joints.size());
  for (const auto & jnt : m_parameters.joints) {
    m_jnt_id.push_back(m_model.idx_qs[m_model.getJointId(jnt)]);
  }
  m_jnt_vs_id.reserve(m_parameters.joints.size());
  for (const auto & jnt : m_parameters.joints) {
    m_jnt_vs_id.push_back(m_model.idx_vs[m_model.getJointId(jnt)]);
  }
  // == End pinocchio models ==
}

Eigen::Vector6d ElastoplasticController::get_wrench_in_world(pinocchio::Data & data)
{
  Eigen::Vector6d wrench_sensor_in_sensor = get_wrench_from_sensor();

  if (wrench_sensor_in_sensor.hasNaN()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *this->get_node()->get_clock(), 1000,
      "Force sensor contains NaN values. Full measure "
      "discarded and replaced with zero");
    wrench_sensor_in_sensor.setZero();
  } else if (wrench_sensor_in_sensor.cwiseAbs().maxCoeff() > 1e20) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *this->get_node()->get_clock(), 1000,
      "Force sensor contains overflowed values. Full "
      "measure discarded and replaced with zero");
    wrench_sensor_in_sensor.setZero();
  }

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "wrench_sensor_in_sensor -> " << wrench_sensor_in_sensor.transpose());

  if (m_parameters.wrench.lpf.enable) {
    std::transform(
      wrench_sensor_in_sensor.begin(), wrench_sensor_in_sensor.end(), m_low_pass_filters.begin(),
      wrench_sensor_in_sensor.begin(),
      [](const double w, eigen_control_toolbox::FilteredScalar & f) {
        f.update(w);
        return f.getUpdatedValue();
      });
  }

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "wrench_sensor_in_sensor -> " << wrench_sensor_in_sensor.transpose());

  // Exponential filter
  std::transform(
    wrench_sensor_in_sensor.begin(), wrench_sensor_in_sensor.end(), m_wrench_in_sensor_prec.begin(),
    wrench_sensor_in_sensor.begin(), [this](const double w, const double w_prec) {
      return filters::exponentialSmoothing(w, w_prec, m_parameters.wrench.filter_alfa);
    });
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "wrench_sensor_in_sensor (after filter) -> " << wrench_sensor_in_sensor.transpose());
  m_wrench_in_sensor_prec = wrench_sensor_in_sensor;
  utils::deadband(
    m_parameters.wrench.deadband[0], m_parameters.wrench.deadband[1], wrench_sensor_in_sensor);

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "wrench_sensor_in_sensor (after filter and deadband) -> " << wrench_sensor_in_sensor.transpose());

  const pinocchio::SE3 & world_M_tool = data.oMf[m_tool_id];
  const pinocchio::SE3 & world_M_sensor = data.oMf[m_sensor_id];
  const pinocchio::SE3 tool_M_sensor = world_M_tool.inverse() * world_M_sensor;
  pinocchio::Force wrench_tool_in_tool =
    tool_M_sensor.act(pinocchio::Force(wrench_sensor_in_sensor));
  const pinocchio::SE3 world_R_tool(world_M_tool.rotation(), Eigen::Vector3d::Zero());

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "wrench_tool_in_tool -> " << wrench_tool_in_tool.toVector());

  Eigen::Vector6d wrench_tool_in_world = world_R_tool.act(wrench_tool_in_tool).toVector();

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "wrench_tool_in_tool -> " << wrench_tool_in_world);

  if (m_offset_future.wait_for(0s) == std::future_status::ready)
    wrench_tool_in_world -= m_offset_wrench_tool_in_world;

  return wrench_tool_in_world;
}

bool ElastoplasticController::write_cmd_vel(const Eigen::Ref<Eigen::Vector3d> & v)
{
  bool b = true;
  if (m_base_use_cmd_ifaces) {
    for (int idx = 0; idx < 3; ++idx) {
      m_mobile_base_command_interfaces.at(idx).get().set_value(v(idx));
    }
  } else {
    geometry_msgs::msg::Twist msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
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

bool ElastoplasticController::write_cmd_vel_zero()
{
  bool b = true;
  if (m_base_use_cmd_ifaces) {
    for (int idx = 0; idx < 3; ++idx) {
      m_mobile_base_command_interfaces.at(idx).get().set_value(0.0);
    }
  } else {
    geometry_msgs::msg::Twist msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
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

controller_interface::CallbackReturn ElastoplasticController::on_init()
{
  m_param_listener = std::make_shared<elastoplastic_controller::ParamListener>(this->get_node());
  RCLCPP_DEBUG(get_node()->get_logger(), "Admittance HQP Controller correctly loaded");
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

  RCLCPP_INFO(get_node()->get_logger(), "[[DEBUG]] creation URDF");

  // RCLCPP_INFO(get_node()->get_logger(), "robot_description:\n%s",
  // robot_description.c_str());

  urdf::ModelInterfaceSharedPtr urdf_model;
  try {
    urdf_model = urdf::parseURDF(robot_description);
  } catch (std::exception & ex) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "ex: " << ex.what());
  }

  RCLCPP_INFO(get_node()->get_logger(), "URDF model created 1");
  if (not urdf_model) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Cannot create URDF model from robot_description provided by "
      "controller_manager");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  RCLCPP_INFO(get_node()->get_logger(), "URDF model created");

  // Check motor and transmission ratios array lengths
  if (m_parameters.motor_torque_constants.size() != m_parameters.joints.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Motor torque constants vector has not the same size of the "
      "joint vector: %ld != %ld",
      m_parameters.motor_torque_constants.size(), m_parameters.joints.size());
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
  if (m_parameters.joints_static_friction.size() != m_parameters.joints.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Joint static friction constants vector has not the same size of the "
      "joint vector: %ld != %ld",
      m_parameters.joints_static_friction.size(), m_parameters.joints.size());
    m_robot_description_configuration = RDStatus::ERROR;
  }
  if (m_parameters.joints_damping.size() != m_parameters.joints.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Joint damping constants vector has not the same size of the "
      "joint vector: %ld != %ld",
      m_parameters.joints_damping.size(), m_parameters.joints.size());
    m_robot_description_configuration = RDStatus::ERROR;
  }
  if (
    std::count_if(
      m_parameters.mobile_base.virtual_inertia.begin(),
      m_parameters.mobile_base.virtual_inertia.end(),
      [](const double & v) { return v > 0; }) != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Virtual inertia constants vector have non-positive values");
    m_robot_description_configuration = RDStatus::ERROR;
  }
  if (
    std::count_if(
      m_parameters.mobile_base.virtual_damping.begin(),
      m_parameters.mobile_base.virtual_damping.end(),
      [](const double & v) { return v > 0; }) != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Virtual damping constants vector have non-positive values");
    m_robot_description_configuration = RDStatus::ERROR;
  }

  build_pinocchio_model(urdf_model, m_mobile_base->enabled);

  m_limits.pos_upper.resize(m_arm_nax);
  m_limits.pos_lower.resize(m_arm_nax);
  m_limits.vel.resize(m_arm_nax);
  m_limits.acc.resize(m_arm_nax);

  if (m_parameters.soft_limits.size() != m_arm_nax) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "soft_limits parameters size != " << m_arm_nax);
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  for (size_t ax = 0; ax < m_arm_nax; ++ax) {
    m_limits.pos_upper(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->upper -
                             m_parameters.soft_limits.at(ax);
    m_limits.pos_lower(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->lower +
                             m_parameters.soft_limits.at(ax);

    if (
      elastoplastic::utils::almost_zero(m_limits.pos_upper(ax)) &&
      elastoplastic::utils::almost_zero(m_limits.pos_lower(ax))) {
      m_limits.pos_upper(ax) = std::numeric_limits<double>::infinity();
      m_limits.pos_lower(ax) = -std::numeric_limits<double>::infinity();
      RCLCPP_WARN(
        get_node()->get_logger(),
        "joint %ld upper and lower limits are both equal to 0, set +/- infinity", ax);
    }

    m_limits.vel(ax) = urdf_model->getJoint(m_parameters.joints.at(ax))->limits->velocity;
    m_limits.acc(ax) = m_parameters.acceleration_limits_coeff * m_limits.vel(ax);
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "Limits joint %ld: upper = %5.2f, lower = %5.2f, vel = %5.2f, "
      "acc = %5.2f",
      ax, m_limits.pos_upper(ax), m_limits.pos_lower(ax), m_limits.vel(ax), m_limits.acc(ax));
  }
  RCLCPP_INFO(get_node()->get_logger(), "Kinematics limits: OK");

  std::string what;
  m_joint_names.resize(m_parameters.joints.size() + m_mobile_base->nax());
  std::ranges::copy(m_mobile_base->base_joint_names, m_joint_names.begin());
  std::ranges::copy(m_parameters.joints, std::next(m_joint_names.begin(), m_mobile_base->nax()));

  RCLCPP_INFO(
    this->get_node()->get_logger(), "Nq: %d, Nv: %d, Manipulator NAx: %ld", m_model.nq, m_model.nv,
    m_arm_nax);
  m_q.resize(m_model.nq);
  m_qp.resize(m_model.nv);
  m_qpp.resize(m_model.nv);

  m_q_in.resize(m_model.nq);
  m_qp_in.resize(m_model.nv);
  m_tau_in.resize(m_model.nv);

  m_W.setIdentity(m_model.nv, m_model.nv);

  // Set here so it is relaunched in on_activate in case of old parameters
  for (size_t idx = 0; idx < 6; idx++) {
    m_impedance.invM.diagonal()(idx) = 1.0 / m_parameters.impedance.inertia[idx];
    m_impedance.K.diagonal()(idx) = m_parameters.impedance.k[idx];
    m_impedance.D.diagonal()(idx) = m_parameters.impedance.d[idx];
    m_impedance.enabled_axis(idx) = m_parameters.impedance.enable_axis[idx];
  }

  RCLCPP_INFO(get_node()->get_logger(), "Kinematics: COMPLETED");
  if (m_robot_description_configuration == RDStatus::EMPTY)
    m_robot_description_configuration = RDStatus::OK;
}

controller_interface::CallbackReturn ElastoplasticController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  m_parameters = m_param_listener->get_params();

  // The parameter update_rate, if not defined, is provided by the
  // controller_manager
  auto update_rate = this->get_node()->get_parameter("update_rate").as_int();
  m_dt = 1.0 / double(update_rate);
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), "dt: " << m_dt);
  if (m_dt < M_MINIMUM_SAMPLING_TIME) {
    RCLCPP_FATAL(
      this->get_node()->get_logger(), "dt: %.6f, too low. Minimum sampling time: %.6f", m_dt,
      M_MINIMUM_SAMPLING_TIME);
    return controller_interface::CallbackReturn::ERROR;
  }

  m_mobile_base = std::make_unique<FloatBaseData>(m_parameters.mobile_base.enabled);

  m_low_pass_filters.reserve(6);
  for (int idx = 0; idx < 6; idx++) {
    m_low_pass_filters.emplace_back();
  }
  if (m_parameters.wrench.lpf.enable) {
    for (int idx = 0; idx < m_low_pass_filters.size(); idx++) {
      m_low_pass_filters.at(idx).activateFilter(
        m_parameters.wrench.deadband.at(0), 50.0, m_parameters.wrench.lpf.fc, m_dt, 0.0);
    }
    RCLCPP_INFO(get_node()->get_logger(), "Configured LPF Filter");
  }

  m_arm_nax = m_parameters.joints.size();

  if (std::ranges::min(m_parameters.impedance.inertia) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Inertia has negative values!");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (m_mobile_base->enabled) {
    m_pub_cmd_vel = this->get_node()->create_publisher<geometry_msgs::msg::Twist>(
      m_parameters.cmd_vel_topic, rclcpp::SystemDefaultsQoS());
    m_rt_pub_cmd_vel =
      std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(m_pub_cmd_vel);
  }

  m_state_controller_publisher =
    get_node()->create_publisher<std_msgs::msg::Int16>("~/hqp_state", rclcpp::SystemDefaultsQoS());
  m_rt_state_controller_publisher =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Int16>>(
      m_state_controller_publisher);

  m_ft_sensor =
    std::make_unique<semantic_components::ForceTorqueSensor>(m_parameters.ft_sensor_name);

  m_pub_full_state =
    this->get_node()->create_publisher<elastoplastic_msgs::msg::ElastoplasticControllerState>(
      "~/full_state", rclcpp::SensorDataQoS());

  m_state_interfaces_names.reserve(m_required_interface_types.size());
  m_command_interfaces_names.reserve(m_required_interface_types.size());

  for (const auto & interface : m_required_interface_types) {
    auto it = std::ranges::find(m_parameters.state_interfaces, interface);
    if (it == m_parameters.state_interfaces.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing State interfaces from parameters");
      return controller_interface::CallbackReturn::FAILURE;
    } else {
      m_state_interfaces_names.push_back(*it);
      RCLCPP_INFO(get_node()->get_logger(), "State interface name: %s", (*it).c_str());
    }
  }

  for (const auto & interface : m_required_interface_types) {
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

  RCLCPP_INFO(get_node()->get_logger(), "Pre Robot description");

  // Robot description-related operations
  if (get_node()->has_parameter("robot_description")) {
    RCLCPP_INFO(get_node()->get_logger(), "Robot description from parameter");
    std_msgs::msg::String::SharedPtr rd = std::make_shared<std_msgs::msg::String>();
    rd->data = get_node()->get_parameter("robot_description").as_string();
    configure_after_robot_description_callback(rd);
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Robot description from topic");
    rclcpp::QoS qos(1);
    qos.transient_local();
    m_robot_description_configuration = RDStatus::EMPTY;
    m_sub_robot_description = get_node()->create_subscription<std_msgs::msg::String>(
      m_parameters.robot_description_topic, qos,
      std::bind(
        &ElastoplasticController::configure_after_robot_description_callback, this,
        std::placeholders::_1));
  }

  RCLCPP_INFO(get_node()->get_logger(), "Pre Init vari");

  std::ranges::fill(m_used_command_interfaces, false);
  if (
    std::ranges::find(m_command_interfaces_names, m_required_interface_types[0]) !=
    m_command_interfaces_names.end()) {
    m_used_command_interfaces.at(0) = true;
  }
  if (
    std::ranges::find(m_command_interfaces_names, m_required_interface_types[1]) !=
    m_command_interfaces_names.end()) {
    m_used_command_interfaces.at(1) = true;
  }
  if (
    std::ranges::find(m_command_interfaces_names, m_required_interface_types[2]) !=
    m_command_interfaces_names.end()) {
    m_used_command_interfaces.at(2) = true;
  }

  m_mobile_base->vel_limits = {
    m_parameters.mobile_base.max_vel.linear[0], m_parameters.mobile_base.max_vel.linear[1],
    m_parameters.mobile_base.max_vel.angular};
  m_mobile_base->acc_limits = {
    m_parameters.mobile_base.max_acc_x, m_parameters.mobile_base.max_acc_y,
    m_parameters.mobile_base.max_acc_yaw};

  m_carteisan_trj_sub = get_node()->create_subscription<moveit_msgs::msg::CartesianTrajectory>(
    m_parameters.cartesian_trajectory_topic, 1,
    [this](const moveit_msgs::msg::CartesianTrajectory & msg) {
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "got trajectory");
      if (msg.header.frame_id != m_parameters.frames.map) {
        RCLCPP_WARN_STREAM(
          get_node()->get_logger(),
          "Trajectory received but in wrong reference "
          "frame. Should be in {"
            << m_parameters.frames.map << "} but instead is in {" << msg.header.frame_id
            << "}. Continuing anyways...");
      }
      m_interpolator = elastoplastic::utils::interpolation::Interpolator::from_msg(msg);
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "-> " << m_interpolator.is_empty());
    });

  m_world_M_base.setIdentity();

  m_base_use_cmd_ifaces = m_parameters.mobile_base.use_command_interfaces;

  RCLCPP_INFO(get_node()->get_logger(), "on_configure() completed");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ElastoplasticController::state_interface_configuration() const
{
  RCLCPP_INFO(get_node()->get_logger(), "Starting state interface export");
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interface_configuration.names.reserve(
    m_parameters.joints.size() * m_required_interface_types.size() + 6);

  for (const auto & jnt : m_parameters.joints) {
    state_interface_configuration.names.emplace_back(
      fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION));
    RCLCPP_INFO(
      get_node()->get_logger(), "State Interface (position): %s",
      state_interface_configuration.names.back().c_str());
  }
  for (const auto & jnt : m_parameters.joints) {
    state_interface_configuration.names.emplace_back(
      fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
    RCLCPP_INFO(
      get_node()->get_logger(), "State Interface (velocity): %s",
      state_interface_configuration.names.back().c_str());
  }
  for (const auto & jnt : m_parameters.joints) {
    state_interface_configuration.names.emplace_back(
      fmt::format("{}/{}", jnt, hardware_interface::HW_IF_EFFORT));
    RCLCPP_INFO(
      get_node()->get_logger(), "State Interface (effort): %s",
      state_interface_configuration.names.back().c_str());
  }
  std::vector<std::string> ft_interfaces = m_ft_sensor->get_state_interface_names();
  state_interface_configuration.names.insert(
    state_interface_configuration.names.end(), ft_interfaces.begin(), ft_interfaces.end());
  RCLCPP_INFO(
    get_node()->get_logger(), "State Interface: %s",
    state_interface_configuration.names.back().c_str());

  return state_interface_configuration;
}

controller_interface::InterfaceConfiguration
ElastoplasticController::command_interface_configuration() const
{
  RCLCPP_INFO(get_node()->get_logger(), "Starting command interface export");
  controller_interface::InterfaceConfiguration command_interface_configuration;
  command_interface_configuration.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interface_configuration.names.reserve(
    m_parameters.joints.size() * m_command_interfaces_names.size());
  if (
    std::ranges::find(m_command_interfaces_names, m_required_interface_types[0]) !=
    m_command_interfaces_names.end()) {
    for (const auto & jnt : m_parameters.joints) {
      command_interface_configuration.names.emplace_back(
        fmt::format("{}/{}", jnt, m_required_interface_types[0]));
      RCLCPP_INFO(
        get_node()->get_logger(), "Command Interface: %s",
        command_interface_configuration.names.back().c_str());
    }
  }
  if (
    std::ranges::find(m_command_interfaces_names, m_required_interface_types[1]) !=
    m_command_interfaces_names.end()) {
    for (const auto & jnt : m_parameters.joints) {
      command_interface_configuration.names.emplace_back(
        fmt::format("{}/{}", jnt, m_required_interface_types[1]));
      RCLCPP_INFO(
        get_node()->get_logger(), "Command Interface: %s",
        command_interface_configuration.names.back().c_str());
    }
  }
  if (
    std::ranges::find(m_command_interfaces_names, m_required_interface_types[2]) !=
    m_command_interfaces_names.end()) {
    for (const auto & jnt : m_parameters.joints) {
      command_interface_configuration.names.emplace_back(
        fmt::format("{}/{}", jnt, m_required_interface_types[2]));
      RCLCPP_INFO(
        get_node()->get_logger(), "Command Interface: %s",
        command_interface_configuration.names.back().c_str());
    }
  }

  if (m_mobile_base->enabled && m_base_use_cmd_ifaces) {
    for (const auto & iface : m_parameters.mobile_base.command_interfaces) {
      // command_interface_configuration.names.emplace_back(fmt::format("{}/{}",
      // iface, hardware_interface::HW_IF_VELOCITY));
      command_interface_configuration.names.emplace_back(iface);
      RCLCPP_INFO(
        get_node()->get_logger(), "Command Interface: %s",
        command_interface_configuration.names.back().c_str());
    }
  }

  return command_interface_configuration;
}

controller_interface::CallbackReturn ElastoplasticController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto t_start = get_node()->get_clock()->now();
  while (!ready_for_activation() &&
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
    std_msgs::msg::String::SharedPtr rd = std::make_shared<std_msgs::msg::String>();
    // rd->data = this->get_robot_description();
    rd->data = get_node()->get_parameter("robot_description").as_string();
    configure_after_robot_description_callback(rd);

    if (m_parameters.wrench.lpf.enable) {
      for (int idx = 0; idx < m_low_pass_filters.size(); idx++) {
        m_low_pass_filters.at(idx).activateFilter(
          m_parameters.wrench.deadband.at(0), 50.0, m_parameters.wrench.lpf.fc, m_dt, 0.0);
      }
      RCLCPP_INFO(get_node()->get_logger(), "Reconfigured LPF Filter");
    }
  }

  m_rt_pub_full_state = std::make_unique<
    realtime_tools::RealtimePublisher<elastoplastic_msgs::msg::ElastoplasticControllerState>>(
    m_pub_full_state);

  m_joint_state_interfaces.resize(3);
  m_joint_command_interfaces.resize(3);

  for (const auto & interface : m_required_interface_types) {
    auto it = std::ranges::find(m_required_interface_types, interface);
    auto idx = std::distance(m_required_interface_types.begin(), it);
    if (not controller_interface::get_ordered_interfaces(
          state_interfaces_, m_parameters.joints, interface, m_joint_state_interfaces.at(idx))) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing joints state interfaces: %ld names vs %ld interfaces",
        m_parameters.joints.size(), m_joint_state_interfaces.at(idx).size());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  assert(command_interfaces_.size() == m_parameters.joints.size());
  auto at_least_one_command_interface{false};
  for (const auto & interface : m_required_interface_types) {
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
    RCLCPP_ERROR(get_node()->get_logger(), "Missing at least one joints command interface");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!m_ft_sensor->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot assing state interface to ft_sensor");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (m_mobile_base->enabled && m_base_use_cmd_ifaces) {
    if (not controller_interface::get_ordered_interfaces(
          command_interfaces_, m_parameters.mobile_base.command_interfaces, "",
          m_mobile_base_command_interfaces)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing base controller command interfaces");
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "Command interfaces:");
  std::for_each(command_interfaces_.begin(), command_interfaces_.end(), [&](const auto & rif) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Interface: " << rif.get_name());
  });
  RCLCPP_INFO(get_node()->get_logger(), "End Command interfaces");

  // Joint initialization
  std::transform(
    m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(),
    m_q.tail(m_arm_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return GET_VALUE_FROM_INTERFACE(lsi);
    });
  RCLCPP_INFO(get_node()->get_logger(), "Read position");
  std::transform(
    m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(),
    m_qp.tail(m_arm_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return GET_VALUE_FROM_INTERFACE(lsi);
    });
  RCLCPP_INFO(get_node()->get_logger(), "Read velocity");
  m_qpp.setZero();

  RCLCPP_INFO(get_node()->get_logger(), "Pre-init mobile base pos");
  if (m_mobile_base->enabled) {
    m_q.head<2>() = m_world_M_base.translation().head<2>();

    Eigen::Affine3d T_world_base(m_world_M_base.toHomogeneousMatrix());
    m_q(2) = std::cos(elastoplastic::utils::vector_from_affine(T_world_base)(5));
    m_q(3) = std::sin(elastoplastic::utils::vector_from_affine(T_world_base)(5));
    m_qp.head<4>().setZero();
  }

  m_initial_q = m_q;

  m_wrench_in_sensor_prec.setZero();
  m_admittance_value.setZero();

  m_offset_wrench_tool_in_world.setZero();
  RCLCPP_INFO(get_node()->get_logger(), "Starting async");
  m_offset_future = std::async(std::launch::async, [this](void) -> bool {
    // Compensate force offset
    if (elastoplastic::utils::almost_zero(m_parameters.offset_force_window)) {
      return true;
    }

    pinocchio::Data model_data_local(m_model);

    const double offset_force_window =
      std::round(m_parameters.offset_force_window * get_update_rate());
    Eigen::Vector6d exp_filter_prec_state = Eigen::Vector6d::Zero();

    // Wrench is in sensor frame
    Eigen::Vector6d result_in_sensor = Eigen::Vector6d::Zero();
    for (int idx = 0; idx < offset_force_window; ++idx) {
      Eigen::Vector6d wr = get_wrench_from_sensor();

      if (m_parameters.wrench.lpf.enable) {
        std::transform(
          wr.begin(), wr.end(), m_low_pass_filters.begin(), wr.begin(),
          [](const double w, eigen_control_toolbox::FilteredScalar & f) {
            f.update(w);
            return f.getUpdatedValue();
          });
      }

      // Exponential filter
      std::transform(
        wr.begin(), wr.end(), exp_filter_prec_state.begin(), wr.begin(),
        [this](const double w, const double w_prec) {
          return filters::exponentialSmoothing(w, w_prec, m_parameters.wrench.filter_alfa);
        });

      utils::deadband(m_parameters.wrench.deadband[0], m_parameters.wrench.deadband[1], wr);

      // accumulate
      std::transform(
        wr.begin(), wr.end(), result_in_sensor.begin(), result_in_sensor.begin(),
        std::plus<double>{});

      std::this_thread::sleep_for(rclcpp::Rate(get_update_rate()).period());
    }
    result_in_sensor /= offset_force_window;
    pinocchio::Force offset_wrench_sensor_in_sensor(result_in_sensor);

    pinocchio::framesForwardKinematics(m_model, model_data_local, m_q);

    const pinocchio::SE3 & world_M_tool = model_data_local.oMf[m_tool_id];
    const pinocchio::SE3 & world_M_sensor = model_data_local.oMf[m_sensor_id];
    const pinocchio::SE3 tool_M_sensor = world_M_tool.inverse() * world_M_sensor;
    pinocchio::Force offset_wrench_tool_in_tool = tool_M_sensor.act(offset_wrench_sensor_in_sensor);
    const pinocchio::SE3 world_R_tool(world_M_tool.rotation(), Eigen::Vector3d::Zero());
    m_offset_wrench_tool_in_world = world_R_tool.act(offset_wrench_tool_in_tool).toVector();

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Wrench Offset computed: " << m_offset_wrench_tool_in_world.transpose());

    m_elastoplastic_model = std::make_unique<ElastoplasticModel>(
      m_impedance.K, m_parameters.impedance.z_max, m_parameters.impedance.z_kmax,
      m_parameters.impedance.z_start,
      static_cast<size_t>(m_parameters.impedance.reset.time * update_rate);
      , m_parameters.impedance.reset.threshold);
    return true;
  });

  pinocchio::framesForwardKinematics(m_model, m_model_data, m_q);
  const pinocchio::SE3 & world_M_tool = m_model_data.oMf[m_tool_id];
  const pinocchio::SE3 & world_M_sensor = m_model_data.oMf[m_sensor_id];
  m_T_tool_sensor = (world_M_tool.inverse() * world_M_sensor).toHomogeneousMatrix();

  m_reference_target_T_world_tool = world_M_tool.toHomogeneousMatrix();
  m_reference_target_acc_tool_world_in_world.setZero();
  m_reference_target_twist_tool_world_in_world.setZero();

  if (m_parameters.use_threshold == 1) {
    m_what_sot_is_active = SoTEnabled::ADMITTANCE;
  } else {
    m_what_sot_is_active = SoTEnabled::TRACKING;
  }

  m_full_position_references = m_initial_q;
  m_full_velocity_references = Eigen::VectorXd::Zero(m_model.nv);

  RCLCPP_INFO(get_node()->get_logger(), "Activated...");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElastoplasticController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  m_interpolator.end_plan();
  for (int idx = 0; idx < m_low_pass_filters.size(); idx++) {
    m_low_pass_filters.at(idx).deactivateFilter();
  }

  std::transform(
    m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(),
    m_q.tail(m_arm_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return GET_VALUE_FROM_INTERFACE(lsi);
    });
  std::transform(
    m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(),
    m_qp.tail(m_arm_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return GET_VALUE_FROM_INTERFACE(lsi);
    });
  m_qpp.setZero();

  pinocchio::framesForwardKinematics(m_model, m_model_data, m_q);
  m_reference_target_T_world_tool = m_model_data.oMf[m_tool_id].toHomogeneousMatrix();
  m_reference_target_acc_tool_world_in_world.setZero();
  m_reference_target_twist_tool_world_in_world.setZero();

  if (m_mobile_base->enabled) {
    Eigen::Vector6d empty = Eigen::Vector6d::Zero();
    geometry_msgs::msg::Twist cmd_vel = tf2::toMsg(empty);
    m_pub_cmd_vel->publish(cmd_vel);
    write_cmd_vel_zero();
  }

  m_rt_pub_full_state->stop();

  m_joint_state_interfaces.clear();
  m_joint_command_interfaces.clear();
  m_mobile_base_command_interfaces.clear();

  m_ft_sensor->release_interfaces();

  m_wrench_in_sensor_prec.setZero();
  m_elastoplastic_model->clear();

  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElastoplasticController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElastoplasticController::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  return ElastoplasticController::on_deactivate(previous_state);
}

std::vector<hardware_interface::CommandInterface>
ElastoplasticController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(m_parameters.joints.size() * m_required_interface_types.size());
  std::vector<hardware_interface::CommandInterface> reference_interfaces{};
  for (const auto & hwi : m_required_interface_types) {
    for (const auto & jnt : m_parameters.joints) {
      reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), fmt::format("{}/{}", jnt, hwi),
        &reference_interfaces_[reference_interfaces.size()]));
    }
  }

  return reference_interfaces;
}

controller_interface::return_type ElastoplasticController::update_reference_from_subscribers()
{
  /* "Joint trajectory available only in chainable mode with
   * joint_trajectory_controller" */

  std::copy(
    m_q.tail(m_arm_nax).begin(), m_q.tail(m_arm_nax).end(),
    reference_interfaces_.begin());  // position
  std::fill(
    std::next(reference_interfaces_.begin(), m_arm_nax), reference_interfaces_.end(),
    0.0);  // velocity

  return controller_interface::return_type::OK;
}

controller_interface::return_type ElastoplasticController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  rclcpp::Time t_start = get_node()->get_clock()->now();

  // **********
  // ** Read **
  // **********

  if (m_mobile_base->enabled) {
    m_q_in.head<4>() = m_q.head<4>();
    m_qp_in.head<3>() = m_qp.head<3>();
    m_tau_in.head<3>().setZero();
  }
  std::transform(
    m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(),
    m_q_in.tail(m_arm_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return GET_VALUE_FROM_INTERFACE(lsi);
    });
  std::transform(
    m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(),
    m_qp_in.tail(m_arm_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return GET_VALUE_FROM_INTERFACE(lsi);
    });
  std::transform(
    m_joint_state_interfaces.at(2).begin(), m_joint_state_interfaces.at(2).end(),
    m_tau_in.tail(m_arm_nax).begin(), [](const hardware_interface::LoanedStateInterface & lsi) {
      return GET_VALUE_FROM_INTERFACE(lsi);
    });

  m_q = m_q_in;
  m_qp = m_qp_in;

  // Update pinocchio data
  pinocchio::forwardKinematics(
    m_model, m_model_data, m_q_in, m_qp_in, Eigen::VectorXd::Zero(m_model.nv));
  pinocchio::computeJointJacobians(m_model, m_model_data, m_q_in);
  pinocchio::updateFramePlacements(m_model, m_model_data);

  m_world_M_base = m_model_data.oMf[m_base_id];

  // m_full_position_references.tail(m_arm_nax) = m_initial_q.tail(m_arm_nax);
  // m_full_velocity_references.setZero();

  /* Cartesian reference */
  if (!m_interpolator.is_plan_started() && m_interpolator.is_ready()) {
    RCLCPP_INFO(get_node()->get_logger(), "Start interpolation of cartesian plan");
    m_interpolator.start_plan(get_node()->get_clock()->now());
  }
  // TODO: end plan

  if (m_interpolator.is_plan_started()) {
    [[maybe_unused]] auto status = m_interpolator.interpolate(
      get_node()->get_clock()->now(), m_reference_target_acc_tool_world_in_world,
      m_reference_target_twist_tool_world_in_world, m_reference_target_T_world_tool);
  }

  /* FT state */
  Eigen::Vector6d wrench_tool_in_world = get_wrench_in_world(m_model_data);

  // ************
  // ** Update **
  // ************

  if (wrench_tool_in_world.hasNaN()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Wrench has NaNs... Skipping update");
    return controller_interface::return_type::ERROR;
  }

  Eigen::Vector6d cart_vel_error_shared_target_in_world =
    (twist_shared_world_in_world - m_computed_target_twist_shared_world_in_world)
      .cwiseProduct(m_elastoplastic_model->get_enabled_axis());

  m_zp = m_elastoplastic_model->update_z(cart_vel_error_shared_target_in_world, m_dt);
  bool reset = m_elastoplastic_model->reset(
    wrench_shared_in_world.cwiseProduct(m_elastoplastic_model->get_enabled_axis()),
    cart_vel_error_shared_target_in_world);

  m_computed_target_T_world_shared = reset ? m_T_world_shared : m_computed_target_T_world_shared;
  m_computed_target_twist_shared_world_in_world =
    reset ? twist_shared_world_in_world : m_computed_target_twist_shared_world_in_world;

  if (reset) {
    RCLCPP_WARN_STREAM(get_node()->get_logger(), "Reset to Elastic Mode");
  }

  std::optional<Eigen::VectorXd> solution_qp = optimize(wrench_tool_in_world);
  Eigen::VectorXd tau_cmd(m_model.nv);

  // Calcolo offset
  if (m_offset_future.wait_for(0s) != std::future_status::ready) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000, "[Waiting] Computing Offset Force");
    // tau_cmd = pinocchio::computeGeneralizedGravity(m_model, m_model_data, m_q_in);
    // tau_cmd.head<3>().setZero();
    tau_cmd.setZero();
    bool result{true};
    for (size_t idx = 0; idx < m_arm_nax; ++idx) {
      if (m_used_command_interfaces.at(0)) {
        m_joint_command_interfaces.at(0).at(idx).get().set_value(
          GET_VALUE_FROM_INTERFACE(m_joint_state_interfaces.at(0).at(idx).get()));
      }
      if (m_used_command_interfaces.at(1)) {
        m_joint_command_interfaces.at(1).at(idx).get().set_value(0.0);
      } else if (m_used_command_interfaces.at(2)) {
        m_joint_command_interfaces.at(2).at(idx).get().set_value(tau_cmd(3 + idx));
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
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Cannot find a solution for the CLIK QP problem. "
      "Keeping actual position");
    assert(false);  // Handled inside clik();
  } else {
    m_admittance_value = solution_qp.value().tail<6>();
    m_qpp = solution_qp.value().head(m_model.nv);
    tau_cmd = solution_qp.value().segment(m_model.nv, m_model.nv);
  }

  m_qp += m_qpp * m_dt;
  m_q = pinocchio::integrate(m_model, m_q, m_qp * m_dt);

  m_computed_target_twist_shared_world_in_world += m_computed_target_acc_shared_world_in_world * m_dt;
  Eigen::Affine3d comp_target_pose = integratePoseLocalWorldAligned(pin::SE3(m_computed_target_T_world_tool), pin::Motion(m_computed_target_twist_tool_world_in_world), m_dt);
  m_computed_target_T_world_tool = comp_target_pose;

  // BEGIN - Saturation Manipulator
  // for (size_t idx = 0; idx < m_arm_nax; ++idx) {
  //   double q = m_q(idx + (m_model.nq - m_arm_nax));
  //   double dq = m_qp(idx + (m_model.nv - m_arm_nax));
  //   m_q(idx + (m_model.nq - m_arm_nax)) =
  //     std::max(
  //       m_limits.pos_lower(idx) - m_parameters.soft_limits.at(idx),
  //       std::min(m_limits.pos_upper(idx) + m_parameters.soft_limits.at(idx),
  //                m_q(idx + (m_model.nq - m_arm_nax))));
  //   m_qp(idx + (m_model.nv - m_arm_nax)) = std::max(
  //       -m_limits.vel(idx),
  //       std::min(m_limits.vel(idx), m_qp(idx + (m_model.nv - m_arm_nax))));
  //   if (!elastoplastic::utils::almost_equal(
  //           q, m_q(idx + (m_model.nq - m_arm_nax)))) {
  //     RCLCPP_WARN(get_node()->get_logger(),
  //                 "Saturation at %f of POSITION (cmd: %f) on manipulator joint "
  //                 "with index %ld",
  //                 q, m_q(idx + (m_model.nq - m_arm_nax)), idx);
  //   }
  //   if (!elastoplastic::utils::almost_equal(
  //           dq, m_qp(idx + (m_model.nv - m_arm_nax)))) {
  //     RCLCPP_WARN(get_node()->get_logger(),
  //                 "Saturation of VELOCITY on manipulator joint with index %ld",
  //                 idx);
  //   }
  // }
  // END - Saturation Manipulator

  int dim_cmd = m_used_command_interfaces.at(0) ? m_model.nq : m_model.nv;
  Eigen::VectorXd cmd(dim_cmd);

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "q -> " << m_q.transpose());
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "qp -> " << m_qp.transpose());

  if (m_used_command_interfaces.at(0)) {
    cmd = m_q;
    // cmd = m_q_in + m_parameters.clik.joint_task.kp * (m_q - m_q_in);
  } else if (m_used_command_interfaces.at(1)) {
    // cmd = m_qp_in + m_parameters.clik.joint_task.kv * (m_qp - m_qp_in);
    cmd = m_qp;
  } else if (m_used_command_interfaces.at(2)) {
    cmd = tau_cmd + m_parameters.gains.k1 * (m_q - m_q_in) +
          m_parameters.gains.k2 * (m_qp - m_qp_in) + m_parameters.gains.k3 * m_qpp;
    if (m_parameters.remove_gravity_from_torque_cmd) {
      cmd -= pinocchio::computeGeneralizedGravity(m_model, m_model_data, m_q_in);
    }
  }

  // ***********
  // ** Write **
  // ***********
  bool is_value_set{true};
  if (m_used_command_interfaces.at(0)) {
    for (size_t ax = 0; ax < m_arm_nax; ++ax) {
      m_joint_command_interfaces.at(0).at(ax).get().set_value(cmd(ax + (m_model.nq - m_arm_nax)));
    }
  }
  if (m_used_command_interfaces.at(1)) {
    for (size_t ax = 0; ax < m_arm_nax; ++ax) {
      m_joint_command_interfaces.at(1).at(ax).get().set_value(cmd(ax + (m_model.nv - m_arm_nax)));
    }
  }
  if (m_used_command_interfaces.at(2)) {
    for (size_t ax = 0; ax < m_arm_nax; ++ax) {
      m_joint_command_interfaces.at(2).at(ax).get().set_value(
        cmd(ax + (m_model.nv - m_arm_nax)) /
        (m_parameters.reduction_ratios[ax] * m_parameters.motor_torque_constants[ax]));
    }
  }
  if (!is_value_set) {
    RCLCPP_FATAL_STREAM(
      get_node()->get_logger(), "Could not write on the hardware interface! Halting!");
    this->on_deactivate(rclcpp_lifecycle::State());
    throw std::runtime_error("Controller crashed");
  }

  if (m_mobile_base->enabled) {
    [[maybe_unused]] bool is_mobile_base_write_ok = write_cmd_vel(m_qp.head<3>());
  }

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

  msg.cart_actual_pose = tf2::toMsg(m_chain_world_tool->getTransformation(q_in));
  msg.cart_actual_twist = tf2::toMsg(m_chain_world_tool->getTwistTool(q_in, qp_in));

  std::tie(msg.reset_buffer_state, msg.reset_buffer_fill) =
    m_elastoplastic_model->get_reset_buffer_status();

  // Admittance state msg
  msg.admittance_state.wrench_base = utils::toWrenchStampedMsg(wrench_tool_in_world);
  msg.admittance_state.wrench_base.header.frame_id = m_parameters.frames.map;
  msg.admittance_state.wrench_base.header.stamp = time_now;

  Eigen::Affine3d fk = m_computed_target_T_world_tool;
  msg.admittance_state.admittance_position = tf2::eigenToTransform(fk);
  msg.admittance_state.admittance_position.header.stamp = time_now;
  msg.admittance_state.admittance_position.header.frame_id = m_parameters.frames.map;
  msg.admittance_state.admittance_position.child_frame_id = m_parameters.frames.tool;

  msg.admittance_state.admittance_velocity.twist =
    tf2::toMsg(m_computed_target_twist_tool_world_in_world);
  msg.admittance_state.admittance_velocity.header.stamp = time_now;
  msg.admittance_state.admittance_velocity.header.frame_id = m_parameters.frames.map;

  msg.admittance_state.admittance_acceleration.twist =
    tf2::toMsg(m_computed_target_acc_tool_world_in_world);
  msg.admittance_state.admittance_acceleration.header.stamp = time_now;
  msg.admittance_state.admittance_acceleration.header.frame_id = m_parameters.frames.map;

  msg.admittance_state.joint_state.header.stamp = time_now;
  msg.admittance_state.joint_state.name.reserve(m_joint_names.size());
  msg.admittance_state.joint_state.position.reserve(m_joint_names.size());
  msg.admittance_state.joint_state.velocity.reserve(m_joint_names.size());
  msg.admittance_state.joint_state.effort.reserve(m_joint_names.size());
  std::copy(
    m_joint_names.begin(), m_joint_names.end(),
    std::back_inserter(msg.admittance_state.joint_state.name));
  std::copy(m_q.begin(), m_q.end(), std::back_inserter(msg.admittance_state.joint_state.position));
  std::copy(
    m_qp.begin(), m_qp.end(), std::back_inserter(msg.admittance_state.joint_state.velocity));
  std::copy(
    m_qpp.begin(), m_qpp.end(), std::back_inserter(msg.admittance_state.joint_state.effort));

  msg.joint_reference.name.reserve(m_joint_names.size());
  msg.joint_reference.position.reserve(m_joint_names.size());
  msg.joint_reference.velocity.reserve(m_joint_names.size());
  std::ranges::copy(full_position_references, std::back_inserter(msg.joint_reference.position));
  std::ranges::copy(full_velocity_references, std::back_inserter(msg.joint_reference.velocity));

  msg.admittance_state.selected_axes.data.reserve(6);
  std::copy(
    m_elastoplastic_model->get_enabled_axis().begin(),
    m_elastoplastic_model->get_enabled_axis().end(),
    std::back_inserter(msg.admittance_state.selected_axes.data));
  msg.admittance_state.ft_sensor_frame.data = m_parameters.frames.sensor;
  msg.admittance_state.ref_trans_base_ft =
    tf2::eigenToTransform(m_chain_base_sensor->getTransformation(m_q.tail(m_nax)));
  msg.admittance_state.rot_base_control = tf2::toMsg(Eigen::Quaterniond(m_T_tool_sensor.linear()));
  msg.admittance_state.stiffness.data.reserve(6);
  msg.admittance_state.damping.data.reserve(6);

  auto [K, D] = m_elastoplastic_model->compute_variable_matrices(T_world_tool);
  Eigen::Vector6d K_diag = K.diagonal();
  std::copy(K_diag.begin(), K_diag.end(), std::back_inserter(msg.admittance_state.stiffness.data));
  std::copy(
    D.diagonal().begin(), D.diagonal().end(),
    std::back_inserter(msg.admittance_state.damping.data));

  utils::toWrenchMsg(m_admittance_value, msg.virtual_force);

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

  // std::for_each(command_interfaces_.begin(), command_interfaces_.end(),
  // [&](const auto& rif) {
  //   RCLCPP_INFO_STREAM(get_node()->get_logger(), "Interface: " <<
  //   rif.get_name() << " - " << GET_VALUE_FROM_INTERFACE(rif));
  // });

  return controller_interface::return_type::OK;
}

}  // namespace elastoplastic

PLUGINLIB_EXPORT_CLASS(
  elastoplastic::ElastoplasticController, controller_interface::ChainableControllerInterface);
