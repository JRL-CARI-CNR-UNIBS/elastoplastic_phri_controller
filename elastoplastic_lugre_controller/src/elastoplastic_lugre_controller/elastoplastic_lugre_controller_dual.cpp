#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller_dual.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"

#include "control_toolbox/filters.hpp"
#include "pal_statistics/pal_statistics_macros.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "urdfdom_headers/urdf_model/model.h" // IWYU pragma: export

#include "rclcpp/qos.hpp"

#include <algorithm>
#include <chrono>
#include <numeric>

#define ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MOBILE_BASE
// #define ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MANIPULATOR
#define USE_CARTESIAN_REFERENCE


namespace elastoplastic {

namespace utils {
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

inline ElastoplasticModelData get_model_data(const elastoplastic_controller_dual::Params& params, const double update_rate) {
  ElastoplasticModelData data;
  std::copy(params.impedance.inertia.begin(), params.impedance.inertia.end(), data.inertia_inv.diagonal().begin());
  std::copy(params.impedance.k.begin(), params.impedance.k.end(), data.k.diagonal().begin());
  std::copy(params.impedance.d.begin(), params.impedance.d.end(), data.d.diagonal().begin());
  data.z_max = params.impedance.z_max;
  data.z_start = params.impedance.z_start;
  data.z_kmax = params.impedance.z_kmax;
  data.enable_axis = params.impedance.enable_axis;
  data.buffer_size = static_cast<size_t>(params.impedance.reset.time * update_rate);
  data.reset_threshold = params.impedance.reset.threshold;
  return data;
}

} // namespace utils

using namespace std::chrono_literals;

bool ElastoplasticControllerDual::write_cmd_vel(const Eigen::Vector3d& v) {
  bool b = true;
  if (m_base_use_cmd_ifaces) {
    for (int idx = 0; idx < 3; ++idx) {
      b = b && m_mobile_base_command_interfaces.at(idx).get().set_value(v(idx));
    }
  } else {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = v(0);
    msg.linear.y = v(1);
    msg.angular.z = v(2);
    if (!m_rt_pub_cmd_vel->tryPublish(msg)) {
      LOG_ERROR_THROTTLE_COUNT(get_node()->get_logger(), get_node()->get_clock(), 1,
                               "Could not publish on " << m_parameters.cmd_vel_topic);
      b = false;
    }
  }
  return b;
}

void ElastoplasticControllerDual::update_grasp_matrices(const Eigen::Matrix3d& R_world_shared) {
  m_grasp_matrix_wrench.block<3, 3>(Eigen::fix<3>, Eigen::fix<0>) = rdyn::skew(R_world_shared * m_p_left_shared_in_shared);
  m_grasp_matrix_wrench.block<3, 3>(Eigen::fix<3>, Eigen::fix<6>) = rdyn::skew(R_world_shared * m_p_right_shared_in_shared);
  m_grasp_matrix_twist.block<3, 3>(Eigen::fix<0>, Eigen::fix<3>) = 0.5 * rdyn::skew(R_world_shared * -m_p_left_shared_in_shared);
  m_grasp_matrix_twist.block<3, 3>(Eigen::fix<0>, Eigen::fix<9>) = 0.5 * rdyn::skew(R_world_shared * -m_p_right_shared_in_shared);
}

controller_interface::CallbackReturn ElastoplasticControllerDual::on_init() {
  m_param_listener = std::make_shared<elastoplastic_controller_dual::ParamListener>(this->get_node());
  RCLCPP_DEBUG(get_node()->get_logger(), "Elastoplastic controller correctly loaded");
  return controller_interface::CallbackReturn::SUCCESS;
}


void ElastoplasticControllerDual::configure_after_robot_description_callback(const std_msgs::msg::String::SharedPtr msg) {
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
  // rdyn::ChainPtr chain_base_fork = rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.fork, gravity);

  // rdyn::ChainPtr chain_fork_tool =
  //   rdyn::createChain(*urdf_model, m_parameters.frames.fork, m_parameters.frames.tools[Side::LEFT], gravity);
  // m_chain_base_tools[Side::LEFT] = rdyn::joinChains(chain_base_fork, chain_fork_tool);
  // chain_fork_tool = rdyn::createChain(*urdf_model, m_parameters.frames.fork, m_parameters.frames.tools[Side::RIGHT], gravity);
  // m_chain_base_tools[Side::RIGHT] = rdyn::joinChains(chain_base_fork, chain_fork_tool);

  m_chain_base_tools[Side::LEFT] =
    rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.tools[Side::LEFT], gravity);
  m_chain_base_tools[Side::RIGHT] =
    rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.tools[Side::RIGHT], gravity);

  // rdyn::ChainPtr chain_fork_sensor =
  //   rdyn::createChain(*urdf_model, m_parameters.frames.fork, m_parameters.frames.sensors[Side::LEFT], gravity);
  // m_chain_base_sensors[Side::LEFT] = rdyn::joinChains(chain_base_fork, chain_fork_sensor);
  // chain_fork_sensor = rdyn::createChain(*urdf_model, m_parameters.frames.fork, m_parameters.frames.sensors[Side::RIGHT],
  // gravity); m_chain_base_sensors[Side::RIGHT] = rdyn::joinChains(chain_base_fork, chain_fork_sensor);

  m_chain_base_sensors[Side::LEFT] =
    rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.sensors[Side::LEFT], gravity);
  m_chain_base_sensors[Side::RIGHT] =
    rdyn::createChain(*urdf_model, m_parameters.frames.base, m_parameters.frames.sensors[Side::RIGHT], gravity);


  if (std::ranges::find(m_chain_base_tools, nullptr) != m_chain_base_tools.end()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot create rdyn chain from base to tool");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  if (std::ranges::find(m_chain_base_sensors, nullptr) != m_chain_base_sensors.end()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot create rdyn chain from base to sensor");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }

  if (m_mobile_base->enabled) {
    urdf::ModelInterfaceSharedPtr mobile_base_model = urdf::parseURDF(utils::MOBILE_BASE_URDF);
    rdyn::ChainPtr chain_world_base = rdyn::createChain(*mobile_base_model, "x_base", "mount_link", {0, 0, -9.806});
    for (const auto& side : Side::arms()) {
      m_chain_world_tools[side] = rdyn::joinChains(chain_world_base, m_chain_base_tools[side]);
    }
  } else {
    m_chain_world_tools[Side::LEFT] =
      rdyn::createChain(*urdf_model, m_parameters.frames.map, m_parameters.frames.tools[Side::LEFT], gravity);
    m_chain_world_tools[Side::RIGHT] =
      rdyn::createChain(*urdf_model, m_parameters.frames.map, m_parameters.frames.tools[Side::RIGHT], gravity);
  }

  if (std::ranges::find(m_chain_world_tools, nullptr) != m_chain_world_tools.end()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot create rdyn chain from base to tool");
    m_robot_description_configuration = RDStatus::ERROR;
    return;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "RDyn chains created");

  m_joint_names.resize(m_parameters.joints.left.size() + m_parameters.joints.right.size() + m_parameters.joints.common.size());
  auto it = std::ranges::copy(m_parameters.joints.common, m_joint_names.begin());
  it = std::ranges::copy(m_parameters.joints.left, it.out);
  it = std::ranges::copy(m_parameters.joints.right, it.out);

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
    m_limits.pos_upper(ax) = urdf_model->getJoint(m_joint_names.at(ax))->limits->upper;
    m_limits.pos_lower(ax) = urdf_model->getJoint(m_joint_names.at(ax))->limits->lower;

    if (utils::almost_zero(m_limits.pos_upper(ax)) && utils::almost_zero(m_limits.pos_lower(ax))) {
      m_limits.pos_upper(ax) = std::numeric_limits<double>::infinity();
      m_limits.pos_lower(ax) = -std::numeric_limits<double>::infinity();
      RCLCPP_WARN(get_node()->get_logger(), "Upper and Lower limits are both equal to 0, set +/- infinity");
    }

    m_limits.vel(ax) = urdf_model->getJoint(m_joint_names.at(ax))->limits->velocity;
    m_limits.acc(ax) = m_parameters.acceleration_limits_coeff * m_limits.vel(ax);
    RCLCPP_DEBUG(get_node()->get_logger(), "Limits joint %ld: upper = %5.2f, lower = %5.2f, vel = %5.2f, acc = %5.2f", ax,
                 m_limits.pos_upper(ax), m_limits.pos_lower(ax), m_limits.vel(ax), m_limits.acc(ax));
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Kinematics limits: OK");

  std::string what;
  std::vector<std::string> v(m_mobile_base->nax() + m_parameters.joints.left.size() + m_parameters.joints.common.size());
  std::vector<std::string> vb(v.size() - m_mobile_base->nax());

  auto it_jnt_names = std::ranges::copy(m_mobile_base->base_joint_names, v.begin());
  it_jnt_names = std::ranges::copy(m_parameters.joints.common, it_jnt_names.out);
  it_jnt_names = std::ranges::copy(m_parameters.joints.left, it_jnt_names.out);
  std::copy(std::next(v.begin(), m_mobile_base->nax()), v.end(), vb.begin());
  m_chain_base_tools[Side::LEFT]->setInputJointsName(vb, what);
  m_chain_base_sensors[Side::LEFT]->setInputJointsName(vb, what);
  m_chain_world_tools[Side::LEFT]->setInputJointsName(v, what);

  v.clear();
  vb.clear();

  v.resize(m_mobile_base->nax() + m_parameters.joints.right.size() + m_parameters.joints.common.size());
  vb.resize(v.size() - m_mobile_base->nax());
  std::copy(std::next(v.begin(), m_mobile_base->nax()), v.end(), vb.begin());

  it_jnt_names = std::ranges::copy(m_mobile_base->base_joint_names, v.begin());
  it_jnt_names = std::ranges::copy(m_parameters.joints.common, it_jnt_names.out);
  it_jnt_names = std::ranges::copy(m_parameters.joints.right, it_jnt_names.out);
  m_chain_base_tools[Side::RIGHT]->setInputJointsName(vb, what);
  m_chain_base_sensors[Side::RIGHT]->setInputJointsName(vb, what);
  m_chain_world_tools[Side::RIGHT]->setInputJointsName(v, what);

  RCLCPP_DEBUG(get_node()->get_logger(), "Kinematics limits: OK");

  m_robot_description_configuration = RDStatus::OK;
}


controller_interface::CallbackReturn
ElastoplasticControllerDual::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  m_parameters = m_param_listener->get_params();

  m_enable_shared_frame_bcast = false;

  // The parameter update_rate, if not defined, is provided by the controller_manager
  auto update_rate = this->get_node()->get_parameter("update_rate").as_int();
  m_dt = 1.0 / double(update_rate);
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), "dt: " << m_dt);
  if (m_dt < M_MINIMUM_SAMPLING_TIME) {
    RCLCPP_FATAL(this->get_node()->get_logger(), "dt: %.6f, too low. Minimum sampling time: %.6f", m_dt, M_MINIMUM_SAMPLING_TIME);
    return controller_interface::CallbackReturn::ERROR;
  }
  m_elastoplastic_model = std::make_unique<ElastoplasticModel>(utils::get_model_data(m_parameters, update_rate));

  m_mobile_base = std::make_unique<FloatBaseData>(m_parameters.mobile_base.enabled);

  m_split_nax[Side::LEFT] = m_parameters.joints.left.size();
  m_split_nax[Side::RIGHT] = m_parameters.joints.right.size();
  m_split_nax[Side::COMMON] = m_parameters.joints.common.size();
  m_split_nax[Side::BASE] = m_mobile_base->nax();
  m_nax_s[Side::LEFT] = m_parameters.joints.left.size() + m_split_nax[Side::COMMON];
  m_nax_s[Side::RIGHT] = m_parameters.joints.right.size() + m_split_nax[Side::COMMON];
  m_idx_st[Side::COMMON] = m_mobile_base->nax();
  m_idx_st[Side::LEFT] = m_idx_st[Side::COMMON] + m_split_nax[Side::COMMON];
  m_idx_st[Side::RIGHT] = m_idx_st[Side::LEFT] + m_split_nax[Side::LEFT];
  m_nax = m_split_nax[Side::LEFT] + m_split_nax[Side::RIGHT] + m_split_nax[Side::COMMON];
  m_full_nax = std::accumulate(m_split_nax.begin(), m_split_nax.end(), 0);
  RCLCPP_DEBUG(this->get_node()->get_logger(), "Full NAx: %ld, Manipulator NAx: %ld", m_full_nax, m_nax);
  m_q.resize(m_full_nax);
  m_qp.resize(m_full_nax);
  m_qpp.resize(m_full_nax);

  // Select appropriate elements for computations on individual chains: m_q(m_selector[Side::LEFT]) --> q[base, commmon, left]
  m_sel[Side::LEFT].resize(m_split_nax[Side::COMMON] + m_split_nax[Side::LEFT] + m_split_nax[Side::BASE]);
  std::iota(m_sel[Side::LEFT].begin(), std::next(m_sel[Side::LEFT].begin(), m_split_nax[Side::BASE]), 0);
  std::iota(std::next(m_sel[Side::LEFT].begin(), m_idx_st[Side::COMMON]),
            std::next(m_sel[Side::LEFT].begin(), m_idx_st[Side::COMMON] + m_split_nax[Side::COMMON]), m_idx_st[Side::COMMON]);
  std::iota(std::next(m_sel[Side::LEFT].begin(), m_split_nax[Side::BASE] + m_split_nax[Side::COMMON]),
            std::next(m_sel[Side::LEFT].begin(), m_split_nax[Side::BASE] + m_split_nax[Side::COMMON] + m_split_nax[Side::LEFT]),
            m_idx_st[Side::LEFT]);

  m_sel[Side::RIGHT].resize(m_split_nax[Side::COMMON] + m_split_nax[Side::RIGHT] + m_split_nax[Side::BASE]);
  std::iota(m_sel[Side::RIGHT].begin(), std::next(m_sel[Side::RIGHT].begin(), m_split_nax[Side::BASE]), 0);
  std::iota(std::next(m_sel[Side::RIGHT].begin(), m_split_nax[Side::BASE]),
            std::next(m_sel[Side::RIGHT].begin(), m_split_nax[Side::BASE] + m_split_nax[Side::COMMON]), m_idx_st[Side::COMMON]);
  std::iota(std::next(m_sel[Side::RIGHT].begin(), m_split_nax[Side::BASE] + m_split_nax[Side::COMMON]),
            std::next(m_sel[Side::RIGHT].begin(), m_split_nax[Side::BASE] + m_split_nax[Side::COMMON] + m_split_nax[Side::RIGHT]),
            m_idx_st[Side::RIGHT]);

  if (std::ranges::min(m_parameters.impedance.inertia) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Inertia has negative values!");
    return controller_interface::CallbackReturn::FAILURE;
  }

  using namespace std::placeholders;
  m_mobile_base_pose_updated = false;
  m_sub_mobile_base_odometry = this->get_node()->create_subscription<nav_msgs::msg::Odometry>(
    m_parameters.mobile_base.odom, 1, std::bind(&ElastoplasticControllerDual::get_odometry_callback, this, _1));
  if (m_mobile_base->enabled) {
    m_mobile_base_pose_updated = true;
    m_pub_cmd_vel =
      this->get_node()->create_publisher<geometry_msgs::msg::Twist>(m_parameters.cmd_vel_topic, rclcpp::SystemDefaultsQoS());
    m_rt_pub_cmd_vel = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(m_pub_cmd_vel);
  }

  m_pub_full_state = this->get_node()->create_publisher<elastoplastic_msgs::msg::ElastoplasticDualControllerState>(
    "~/full_state", rclcpp::SensorDataQoS());
  m_rt_pub_full_state =
    std::make_unique<realtime_tools::RealtimePublisher<elastoplastic_msgs::msg::ElastoplasticDualControllerState>>(
      m_pub_full_state);

  m_ft_sensors[Side::LEFT] = std::make_unique<semantic_components::ForceTorqueSensor>(m_parameters.ft_sensor_names.at(0));
  m_ft_sensors[Side::RIGHT] = std::make_unique<semantic_components::ForceTorqueSensor>(m_parameters.ft_sensor_names.at(1));


  m_state_interfaces_names.reserve(m_allowed_interface_types.size());
  m_command_interfaces_names.reserve(m_allowed_interface_types.size());

  for (const auto& interface : m_allowed_interface_types) {
    auto it = std::ranges::find(m_parameters.state_interfaces, interface);
    if (it == m_parameters.state_interfaces.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing State interfaces from parameters");
      return controller_interface::CallbackReturn::FAILURE;
    } else {
      m_state_interfaces_names.push_back(*it);
      RCLCPP_INFO(get_node()->get_logger(), "State interface name: %s", (*it).c_str());
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
      std::bind(&ElastoplasticControllerDual::configure_after_robot_description_callback, this, std::placeholders::_1));
    m_robot_description_configuration = RDStatus::EMPTY;
#endif
  }

  std::ranges::fill(m_used_command_interfaces, false);
  if (std::ranges::find(m_command_interfaces_names, m_allowed_interface_types[0]) != m_command_interfaces_names.end()) {
    m_used_command_interfaces.at(0) = true;
  }
  if (std::ranges::find(m_command_interfaces_names, m_allowed_interface_types[1]) != m_command_interfaces_names.end()) {
    m_used_command_interfaces.at(1) = true;
  }

  m_W.setIdentity(m_full_nax, m_full_nax);

  m_kp_joint_task = m_parameters.clik.joint_task.kp;
  m_kv_joint_task = m_parameters.clik.joint_task.kv;

  m_mobile_base->vel_limits = {m_parameters.mobile_base.max_vel.linear[0], m_parameters.mobile_base.max_vel.linear[1],
                               m_parameters.mobile_base.max_vel.angular};
  m_mobile_base->acc_limits = {m_parameters.mobile_base.max_acc_x, m_parameters.mobile_base.max_acc_y,
                               m_parameters.mobile_base.max_acc_yaw};
  m_cartesian_pos_limits = Eigen::Map<Eigen::Vector6d>(m_parameters.cartesian_limits.position.data(), 6);

  m_logistic = {.max = m_parameters.impedance.logistic.max,
                .slope = m_parameters.impedance.logistic.slope,
                .inflection = m_parameters.impedance.logistic.inflection * m_mobile_base->vel_limits};

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
  m_tf_base_pose_recovery_thread = std::make_unique<std::thread>(&ElastoplasticControllerDual::update_base_pose_from_tf, this);
  bool can_transform{false};
  do {
    can_transform = m_tf_buffer->canTransform(m_parameters.frames.map, m_parameters.frames.base, tf2::TimePointZero);
  } while (!can_transform);

  m_tf_bcast = std::make_shared<tf2_ros::TransformBroadcaster>(get_node()->shared_from_this());

  std::fill_n(m_deadbands.begin(), 3, m_parameters.wrench.deadband[0]);
  std::fill_n(std::next(m_deadbands.begin(), 3), 3, m_parameters.wrench.deadband[1]);
  std::fill_n(std::next(m_deadbands.begin(), 6), 3, m_parameters.wrench.deadband[2]);
  std::fill_n(std::next(m_deadbands.begin(), 9), 3, m_parameters.wrench.deadband[3]);

  m_base_use_cmd_ifaces = m_parameters.mobile_base.use_command_interfaces;

  return controller_interface::CallbackReturn::SUCCESS;
}

void ElastoplasticControllerDual::update_base_pose_from_tf() {
  m_node_support = rclcpp::Node::make_shared(
    "__support_node__", fmt::format("{}{}", this->get_node()->get_namespace(),
                                    this->get_node()->get_name())); // lifecycle nodes cannot create sub_nodes
  m_node_support->set_parameter(get_node()->get_parameter("use_sim_time"));
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer, m_node_support, false);
  auto timer = rclcpp::create_timer(m_node_support, m_node_support->get_clock(),
                                    std::chrono::duration<double>(1e1 / (double)get_update_rate()), [this]() {
                                      if (m_enable_shared_frame_bcast) {

                                        geometry_msgs::msg::TransformStamped t;
                                        {
                                          std::lock_guard<std::mutex> lock(m_mutex);
                                          t = tf2::eigenToTransform(m_T_world_shared);
                                        }
                                        t.header.frame_id = m_parameters.frames.map;
                                        t.child_frame_id = SHARED_FRAME_NAME;
                                        t.header.stamp = m_node_support->get_clock()->now();
                                        m_tf_bcast->sendTransform(t);
                                      }
                                    });
  m_support_node_exec = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  m_support_node_exec->add_node(m_node_support);
  m_support_node_exec->spin();
}

controller_interface::InterfaceConfiguration ElastoplasticControllerDual::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interface_configuration.names.reserve(
    (m_parameters.joints.left.size() + m_parameters.joints.right.size()) * m_allowed_interface_types.size() + 12);

  for (const auto& jnt : m_joint_names) {
    state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_POSITION));
  }
  for (const auto& jnt : m_joint_names) {
    state_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, hardware_interface::HW_IF_VELOCITY));
  }
  for (const auto& side : Side::arms()) {
    std::vector<std::string> ft_interfaces = m_ft_sensors[side]->get_state_interface_names();
    state_interface_configuration.names.insert(state_interface_configuration.names.end(), ft_interfaces.begin(),
                                               ft_interfaces.end());
  }

  return state_interface_configuration;
}


controller_interface::InterfaceConfiguration ElastoplasticControllerDual::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interface_configuration;
  command_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interface_configuration.names.reserve(m_nax * m_command_interfaces_names.size());
  if (std::ranges::find(m_command_interfaces_names, m_allowed_interface_types[0]) != m_command_interfaces_names.end()) {
    for (const auto& jnt : m_joint_names) {
      command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, m_allowed_interface_types[0]));
    }
  }
  if (std::ranges::find(m_command_interfaces_names, m_allowed_interface_types[1]) != m_command_interfaces_names.end()) {
    for (const auto& jnt : m_joint_names) {
      command_interface_configuration.names.emplace_back(fmt::format("{}/{}", jnt, m_allowed_interface_types[1]));
    }
  }

  if (m_mobile_base->enabled && m_base_use_cmd_ifaces) {
    for (const auto& iface : m_parameters.mobile_base.command_interfaces) {
      // command_interface_configuration.names.emplace_back(fmt::format("{}/{}", iface, hardware_interface::HW_IF_VELOCITY));
      command_interface_configuration.names.emplace_back(iface);
    }
  }

  return command_interface_configuration;
}


controller_interface::CallbackReturn ElastoplasticControllerDual::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
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
    std::make_unique<realtime_tools::RealtimePublisher<elastoplastic_msgs::msg::ElastoplasticDualControllerState>>(
      m_pub_full_state);

  m_elastoplastic_model->clear();

  m_joint_state_interfaces.resize(m_allowed_interface_types.size());
  m_joint_command_interfaces.resize(m_allowed_interface_types.size());
  for (const auto& interface : m_allowed_interface_types) {
    auto it = std::ranges::find(m_allowed_interface_types, interface);
    auto idx = std::distance(m_allowed_interface_types.begin(), it);
    if (not controller_interface::get_ordered_interfaces(state_interfaces_, m_joint_names, interface,
                                                         m_joint_state_interfaces.at(idx))) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing joints state interfaces: %ld names vs %ld interfaces", m_nax,
                   m_joint_state_interfaces.at(idx).size());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  auto at_least_one_command_interface{false};
  for (const auto& interface : m_allowed_interface_types) {
    auto it = std::ranges::find(m_allowed_interface_types, interface);
    auto idx = std::distance(m_allowed_interface_types.begin(), it);
    if (not controller_interface::get_ordered_interfaces(command_interfaces_, m_joint_names, interface,
                                                         m_joint_command_interfaces.at(idx))) {
      continue;
    }
    at_least_one_command_interface = true;
  }
  if (!at_least_one_command_interface) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing at least one joints command interface");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!m_ft_sensors[Side::LEFT]->assign_loaned_state_interfaces(state_interfaces_) ||
      !m_ft_sensors[Side::RIGHT]->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot assing state interface to an ft_sensor");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (m_mobile_base->enabled && m_base_use_cmd_ifaces) {
    if (not controller_interface::get_ordered_interfaces(command_interfaces_, m_parameters.mobile_base.command_interfaces, "",
                                                         m_mobile_base_command_interfaces)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing base controller command interfaces");
      return controller_interface::CallbackReturn::FAILURE;
    }
    // else {
    // for (const auto& iface : m_mobile_base_command_interfaces) {
    // RCLCPP_INFO_STREAM(get_node()->get_logger(), "mobile base iface: " << iface.get().get_name());
    // }
    // }
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

  m_last_odom_msg_time = this->get_node()->get_clock()->now();

  try {
    m_T_world_base =
      tf2::transformToEigen(m_tf_buffer->lookupTransform(m_parameters.frames.map, m_parameters.frames.base, tf2::TimePointZero));
  } catch (tf2::LookupException ex) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Cannot get transformation from " << m_parameters.frames.base << " to "
                                                                                    << m_parameters.frames.map
                                                                                    << ". Cannot activate controller.");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (m_mobile_base->enabled) {
    m_q.head<2>() = m_T_world_base.translation().head<2>();
    m_q(2) = utils::vector_from_affine(m_T_world_base)(5);
    // write_cmd_vel(Eigen::Vector3d::Zero());
    m_qp.head<3>().setZero();
  }

  m_velocity_base_in_base.setZero();

  m_initial_q = m_q;

  m_wrench_in_sensor_prec.setZero();
  m_admittance_value.setZero();


  Eigen::Affine3d T_world_left = m_chain_world_tools[Side::LEFT]->getTransformation(m_q(m_sel[Side::LEFT]));
  Eigen::Affine3d T_world_right = m_chain_world_tools[Side::RIGHT]->getTransformation(m_q(m_sel[Side::RIGHT]));
  Eigen::Affine3d T_world_shared;
  Eigen::Vector6d left_right_distance; // vector from left to right
  utils::get_frame_distance(T_world_left, T_world_right, left_right_distance);
  T_world_shared = get_shared_frame(T_world_left, T_world_right);
  m_T_left_shared = T_world_left.inverse() * T_world_shared;
  m_T_right_shared_ideal = T_world_right.inverse() * T_world_shared;

  m_enable_shared_frame_bcast = m_parameters.enable_shared_frame_broadcast;

  Eigen::Vector3d p_left_shared_in_world = -utils::get_frame_distance(T_world_shared, T_world_left).head<3>();
  Eigen::Vector3d p_right_shared_in_world = -utils::get_frame_distance(T_world_shared, T_world_right).head<3>();

  m_p_left_shared_in_shared = T_world_shared.linear().transpose() * (T_world_left.translation() - T_world_shared.translation());
  m_p_right_shared_in_shared = T_world_shared.linear().transpose() * (T_world_right.translation() - T_world_shared.translation());

  m_grasp_matrix_wrench << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity(),
    Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity(), rdyn::skew(p_left_shared_in_world), Eigen::Matrix3d::Identity(),
    rdyn::skew(p_right_shared_in_world);

  m_grasp_matrix_twist << Eigen::Matrix3d::Identity(), -rdyn::skew(p_left_shared_in_world), Eigen::Matrix3d::Identity(),
    -rdyn::skew(p_right_shared_in_world), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
    Eigen::Matrix3d::Identity();
  m_grasp_matrix_twist *= 0.5;

  // m_grasp_matrix_wrench << Eigen::Matrix6d::Identity(), Eigen::Matrix6d::Identity();
  // m_grasp_matrix_twist << 0.5 * Eigen::Matrix6d::Identity(), 0.5 * Eigen::Matrix6d::Identity();

  m_computed_target_T_world_shared =
    get_shared_frame(m_chain_world_tools[Side::LEFT]->getTransformation(m_q(m_sel[Side::LEFT])),
                     m_chain_world_tools[Side::RIGHT]->getTransformation(m_q(m_sel[Side::RIGHT])));
  m_computed_target_acc_shared_world_in_world.setZero();
  m_computed_target_twist_shared_world_in_world.setZero();

  m_logis_prec = 0;

  m_offset_wrench_tool_in_world.setZero();
  m_offset_future = std::async(std::launch::async, [this](void) -> bool {
    // Compensate force offset
    if (utils::almost_zero(m_parameters.offset_force_window)) {
      return true;
    }
    m_offset_wrench_sensor_in_sensor.setZero();
    const double offset_force_window = std::round(m_parameters.offset_force_window * get_update_rate());
    for (int idx = 0; idx < offset_force_window; ++idx) {
      Eigen::Vector12d wr = get_wrenches();
      std::transform(wr.begin(), wr.end(), m_deadbands.begin(), wr.begin(), [](const double w, const double deadband) {
        return std::abs(w) > deadband ? utils::sgn(w) * (std::abs(w) - deadband) : 0.0;
      });
      std::transform(wr.begin(), wr.end(), m_offset_wrench_sensor_in_sensor.begin(), m_offset_wrench_sensor_in_sensor.begin(),
                     std::plus<double>{});
      // std::this_thread::sleep_for(rclcpp::Rate(get_update_rate()).period());
      rclcpp::Rate(get_update_rate()).sleep();
    }
    m_offset_wrench_sensor_in_sensor /= offset_force_window;

    // Transform wrench offset in world T_tool_sensor
    for (const auto& side : Side::arms()) {
      Eigen::Vector6d offset_wrench_tool_in_tool = rdyn::spatialDualTranformation(
        m_offset_wrench_sensor_in_sensor.segment<6>(side * 6),
        m_chain_base_tools[side]->getTransformation(m_q(m_sel[side]).tail(m_nax_s[side])).inverse() *
          m_chain_base_sensors[side]->getTransformation(m_q(m_sel[side]).tail(m_nax_s[side])));
      m_offset_wrench_tool_in_world.segment<6>(side * 6) = rdyn::spatialRotation(
        offset_wrench_tool_in_tool, m_chain_world_tools[side]->getTransformation(m_q(m_sel[side])).linear());
    }

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Wrench Offset computed:\n" << m_offset_wrench_tool_in_world);
    return true;
  });

  Eigen::Matrix6d kfA, kfC, kfQ, kfR, kfP0;
  Eigen::Matrix<double, 6, 3> kfB;
  kfA << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity() * m_dt, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();
  kfB << Eigen::Matrix3d::Identity() * 0.5 * std::pow(m_dt, 2), Eigen::Matrix3d::Identity() * m_dt;
  kfC.setIdentity();
  kfQ.setIdentity();
  kfR.setIdentity() * 1e2;
  kfP0.setIdentity() * 1e6;
  m_base_position_filter =
    state_observer::KalmanFilter(kfA, kfB, kfC, (Eigen::VectorXd(6) << m_q.head<3>(), m_qp.head<3>()).finished(), kfQ, kfR, kfP0);
  m_joint_filter.initialize(
    (Eigen::VectorXd(3 * m_nax) << m_q.tail(m_nax), m_qp.tail(m_nax), Eigen::VectorXd::Zero(m_nax)).finished());

  m_pos_task_slider.init(1, 1e1, 1e-4);

  RCLCPP_DEBUG(m_node_support->get_logger(), "Activated...");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ElastoplasticControllerDual::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {

  std::transform(m_joint_state_interfaces.at(0).begin(), m_joint_state_interfaces.at(0).end(), m_q.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
  std::transform(m_joint_state_interfaces.at(1).begin(), m_joint_state_interfaces.at(1).end(), m_qp.tail(m_nax).begin(),
                 [](const hardware_interface::LoanedStateInterface& lsi) { return lsi.get_optional().value(); });
  m_qpp.setZero();

  m_computed_target_T_world_shared =
    get_shared_frame(m_chain_world_tools[Side::LEFT]->getTransformation(m_q(m_sel[Side::LEFT])),
                     m_chain_world_tools[Side::RIGHT]->getTransformation(m_q(m_sel[Side::RIGHT])));
  m_computed_target_acc_shared_world_in_world.setZero();
  m_computed_target_twist_shared_world_in_world.setZero();
  m_joint_state_interfaces.clear();
  m_joint_command_interfaces.clear();
  m_ft_sensors[Side::LEFT]->release_interfaces();
  m_ft_sensors[Side::RIGHT]->release_interfaces();
  m_wrench_in_sensor_prec.setZero();

  if (m_mobile_base->enabled) {
    Eigen::Vector6d empty = Eigen::Vector6d::Zero();
    geometry_msgs::msg::Twist cmd_vel = tf2::toMsg(empty);
    m_pub_cmd_vel->publish(cmd_vel);
    write_cmd_vel(Eigen::Vector3d::Zero());
  }

  m_enable_shared_frame_bcast = false;

  m_rt_pub_full_state->stop();

  m_elastoplastic_model->clear();

  m_joint_state_interfaces.clear();
  m_joint_command_interfaces.clear();
  m_mobile_base_command_interfaces.clear();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElastoplasticControllerDual::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (m_support_node_exec->is_spinning()) {
    m_support_node_exec->cancel();
  }
  if (m_tf_base_pose_recovery_thread->joinable())
    m_tf_base_pose_recovery_thread->join();
  m_support_node_exec->remove_node(m_node_support);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElastoplasticControllerDual::on_error(const rclcpp_lifecycle::State& previous_state) {
  return ElastoplasticControllerDual::on_deactivate(previous_state);
}


std::vector<hardware_interface::CommandInterface> ElastoplasticControllerDual::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  m_joint_reference_interfaces_size =
    m_joint_names.size() * m_allowed_interface_types.size(); // There must be both position and velocity reference interfaces!

  reference_interfaces_.resize(m_joint_reference_interfaces_size);
  reference_interfaces.reserve(m_joint_reference_interfaces_size);

  size_t idx = 0;
  for (const auto& hwi : m_allowed_interface_types) {
    for (const auto& jnt : m_joint_names) {
      reference_interfaces.emplace_back(hardware_interface::CommandInterface(
        std::string(get_node()->get_name()), fmt::format("{}/{}", jnt, hwi), &reference_interfaces_[idx]));
      ++idx;
    }
  }

  return reference_interfaces;
}


controller_interface::return_type
ElastoplasticControllerDual::update_reference_from_subscribers(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  /* "Joint trajectory available only in chainable mode with joint_trajectory_controller" */

  std::copy(m_q.tail(m_nax).begin(), m_q.tail(m_nax).end(), reference_interfaces_.begin());     // position
  std::fill(std::next(reference_interfaces_.begin(), m_nax), reference_interfaces_.end(), 0.0); // velocity

  return controller_interface::return_type::OK;
}

void ElastoplasticControllerDual::get_odometry_callback(const nav_msgs::msg::Odometry& msg) {
  m_rt_buffer_base_odom.writeFromNonRT(msg);
}


controller_interface::return_type ElastoplasticControllerDual::update_and_write_commands(const rclcpp::Time& /*time*/,
                                                                                         const rclcpp::Duration& /*period*/) {
  rclcpp::Time t_start = get_node()->get_clock()->now();

  if (m_offset_future.wait_for(0s) != std::future_status::ready) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "[Waiting] Computing Offset Force");
    bool result{true};
    for (size_t idx = 0; idx < m_nax; ++idx) {
      result &=
        m_joint_command_interfaces.at(0).at(idx).get().set_value(m_joint_state_interfaces.at(0).at(idx).get().get_optional().value());
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

#ifdef ELASTOPLASTIC__READ_STATES_FROM_INTERFACES__MOBILE_BASE
  // Base state
  bool got_new_odom = false;
  if (m_mobile_base->enabled) {
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
    m_qp.head<M_SE2>() = estim_base.tail<M_SE2>();
    m_q.head<M_SE2>() = estim_base.head<M_SE2>();
    // m_qp.head<M_SE2>() = utils::base_velocity_from_twist(twist_base_world_in_world);
    // m_q.head<2>() = m_T_world_base.translation().head<2>();
    // m_q(2) = Eigen::AngleAxisd(m_T_world_base.linear()).angle();
  }
#else
  bool got_new_odom = true;
#endif

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

  Eigen::Vector12d twist_tool_world_in_world;
  std::array<Eigen::Affine3d, 2> T_world_tool;
  T_world_tool[Side::LEFT] = m_chain_world_tools[Side::LEFT]->getTransformation(m_q(m_sel[Side::LEFT]));
  T_world_tool[Side::RIGHT] = m_chain_world_tools[Side::RIGHT]->getTransformation(m_q(m_sel[Side::RIGHT]));
  twist_tool_world_in_world.head<6>() =
    m_chain_world_tools[Side::LEFT]->getTwistTool(m_q(m_sel[Side::LEFT]), m_qp(m_sel[Side::LEFT]));
  twist_tool_world_in_world.tail<6>() =
    m_chain_world_tools[Side::RIGHT]->getTwistTool(m_q(m_sel[Side::RIGHT]), m_qp(m_sel[Side::RIGHT]));
  Eigen::VectorXd full_position_references(m_full_nax), full_velocity_references(m_full_nax);

  {
    std::lock_guard<std::mutex> lock(m_mutex); // Thread to publish tf
    m_T_world_shared = get_shared_frame(m_chain_world_tools[Side::LEFT]->getTransformation(m_q(m_sel[Side::LEFT])),
                                        m_chain_world_tools[Side::RIGHT]->getTransformation(m_q(m_sel[Side::RIGHT])));
  }

#ifndef USE_CARTESIAN_REFERENCE
  /* Joint Reference */
  // Target base
  Eigen::Vector6d target_twist_base_world_in_world;
  Eigen::Vector3d mobile_base_pose_in_world;
  if (m_mobile_base->enabled) {
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
  if (m_mobile_base->enabled) {
    full_position_references.head(m_mobile_base->nax()) << mobile_base_pose_in_world;
    full_velocity_references.head(m_mobile_base->nax()) << utils::base_velocity_from_twist(target_twist_base_world_in_world);
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

  Eigen::Vector6d reference_target_acc_shared_world_in_world;
  Eigen::Vector6d reference_target_twist_shared_world_in_world;
  Eigen::Affine3d reference_target_T_world_shared;
  if (m_interpolator.is_plan_started()) {
    auto status = m_interpolator.interpolate(get_node()->get_clock()->now(), reference_target_acc_shared_world_in_world,
                                             reference_target_twist_shared_world_in_world, reference_target_T_world_shared);
    if (status != utils::interpolation::Interpolator::InterpolationResult::OK) {
      reference_target_acc_shared_world_in_world.setZero();
      reference_target_twist_shared_world_in_world.setZero();
      reference_target_T_world_shared =
        get_shared_frame(m_chain_world_tools[Side::LEFT]->getTransformation(m_initial_q(m_sel[Side::LEFT])),
                         m_chain_world_tools[Side::RIGHT]->getTransformation(m_initial_q(m_sel[Side::RIGHT])));
    }
  } else {
    reference_target_acc_shared_world_in_world.setZero();
    reference_target_twist_shared_world_in_world.setZero();
    reference_target_T_world_shared =
      get_shared_frame(m_chain_world_tools[Side::LEFT]->getTransformation(m_initial_q(m_sel[Side::LEFT])),
                       m_chain_world_tools[Side::RIGHT]->getTransformation(m_initial_q(m_sel[Side::RIGHT])));
  }
  if (m_mobile_base->enabled) {
    full_velocity_references.head<M_SE2>() = utils::base_velocity_from_twist(reference_target_twist_shared_world_in_world);
    Eigen::Affine3d T_world_base_ref =
      m_chain_world_tools[Side::LEFT]->getTransformationLink(m_initial_q(m_sel[Side::LEFT]), m_parameters.frames.base);
    Eigen::Affine3d T_world_shared_ref =
      get_shared_frame(m_chain_world_tools[Side::LEFT]->getTransformation(m_initial_q(m_sel[Side::LEFT])),
                       m_chain_world_tools[Side::RIGHT]->getTransformation(m_initial_q(m_sel[Side::RIGHT])));
    full_position_references.head<M_SE2>() = utils::base_velocity_from_twist(
      utils::vector_from_affine(reference_target_T_world_shared * T_world_shared_ref.inverse() * T_world_base_ref));
  }

#endif

  update_grasp_matrices(m_T_world_shared.linear());

  /* FT state */
  Eigen::Vector12d wrench_sensor_in_sensor = get_wrenches();

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

  // ************
  // ** Update **
  // ************

  // Wrench deadband
  std::transform(wrench_sensor_in_sensor.begin(), wrench_sensor_in_sensor.end(), m_deadbands.begin(),
                 wrench_sensor_in_sensor.begin(), [](const double w, const double deadband) {
                   return std::abs(w) > deadband ? utils::sgn(w) * (std::abs(w) - deadband) : 0.0;
                 });

  // Exponential filter
  std::transform(wrench_sensor_in_sensor.begin(), wrench_sensor_in_sensor.end(), m_wrench_in_sensor_prec.begin(),
                 wrench_sensor_in_sensor.begin(), [this](const double w, const double w_prec) {
                   return filters::exponentialSmoothing(w, w_prec, m_parameters.wrench.filter_alfa);
                 });

  Eigen::Vector12d wrench_tool_in_tool, wrench_tool_in_world;
  for (const auto& side : Side::arms()) {
    Eigen::Affine3d T_base_tool = m_chain_base_tools[side]->getTransformation(m_q(m_sel[side]).tail(m_nax_s[side]));
    Eigen::Affine3d T_base_sensor = m_chain_base_sensors[side]->getTransformation(m_q(m_sel[side]).tail(m_nax_s[side]));
    Eigen::Affine3d T_tool_sensor = T_base_tool.inverse() * T_base_sensor;

    // Eigen::Vector6d wrench_tool_in_tool;
    wrench_tool_in_tool.segment<6>(side * 6) =
      rdyn::spatialDualTranformation(wrench_sensor_in_sensor.segment<6>(side * 6), T_tool_sensor);

    wrench_tool_in_world.segment<6>(side * 6) =
      rdyn::spatialRotation(wrench_tool_in_tool.segment<6>(side * 6), T_world_tool[side].linear());
    // - m_offset_wrench_tool_in_world.segment<6>(side * 6);
  }
  m_wrench_in_sensor_prec = wrench_sensor_in_sensor;
  Eigen::Vector6d wrench_shared_in_world = m_grasp_matrix_wrench * wrench_tool_in_world;
  // Eigen::Vector6d wrench_shared_offset = m_grasp_matrix_wrench * m_offset_wrench_tool_in_world;

  Eigen::Matrix6Xd Jtmp1, Jtmp2;
  Jtmp1 = m_chain_world_tools[Side::LEFT]->getJacobian(m_q(m_sel[Side::LEFT]));
  Jtmp2 = m_chain_world_tools[Side::RIGHT]->getJacobian(m_q(m_sel[Side::RIGHT]));
  // Reorder: [J_base, J_common, J_left, J_right]
  Eigen::Matrix12Xd J_world_tool_in_world(12, m_full_nax);
  J_world_tool_in_world << Jtmp1.leftCols(m_split_nax[Side::BASE]),
    Jtmp1.middleCols(m_idx_st[Side::COMMON], m_split_nax[Side::COMMON]),
    Jtmp1.middleCols(m_idx_st[Side::LEFT], m_split_nax[Side::LEFT]), Eigen::Matrix6Xd::Zero(6, m_split_nax[Side::RIGHT]), //
    Jtmp2.leftCols(m_split_nax[Side::BASE]), Jtmp2.middleCols(m_idx_st[Side::COMMON], m_split_nax[Side::COMMON]),
    Eigen::Matrix6Xd::Zero(6, m_split_nax[Side::LEFT]), //
    Jtmp2.rightCols(m_split_nax[Side::RIGHT]);

  Eigen::Vector6d twist_shared_world_in_world = m_grasp_matrix_twist * twist_tool_world_in_world;

  Eigen::Vector6d cart_vel_error_shared_target_in_world =
    (twist_shared_world_in_world - m_computed_target_twist_shared_world_in_world)
      .cwiseProduct(m_elastoplastic_model->get_enabled_axis());
  // Eigen::Vector6d pose_error;
  // utils::get_frame_distance(T_world_shared, m_computed_target_T_world_shared, pose_error);
  // pose_error.normalize();
  m_zp = m_elastoplastic_model->update_z(cart_vel_error_shared_target_in_world, m_dt);
  bool reset = m_elastoplastic_model->reset(wrench_shared_in_world.cwiseProduct(m_elastoplastic_model->get_enabled_axis()),
                                            cart_vel_error_shared_target_in_world);
  if (reset) {
    RCLCPP_WARN_STREAM(get_node()->get_logger(), "Reset to Elastic Mode");
  }
  m_computed_target_T_world_shared = reset ? m_T_world_shared : m_computed_target_T_world_shared;

  m_computed_target_twist_shared_world_in_world += m_computed_target_acc_shared_world_in_world * m_dt;
  m_computed_target_T_world_shared =
    rdyn::spatialIntegration(m_computed_target_T_world_shared, m_computed_target_twist_shared_world_in_world, m_dt);

  // Eigen::Vector6d cmp_t_wt = utils::vector_from_affine(m_computed_target_T_world_shared);
  // std::tie(cmp_t_wt, m_computed_target_twist_shared_world_in_world) =
  // utils::rk4_double([&](const auto&, const auto&, const auto& u) { return u; }, cmp_t_wt,
  // m_computed_target_twist_shared_world_in_world, m_computed_target_acc_shared_world_in_world, m_dt);
  // m_computed_target_T_world_shared = utils::affine_from_vector(cmp_t_wt);


  Eigen::Vector6d dist;
  utils::get_frame_distance(m_T_world_shared, reference_target_T_world_shared, dist);
  if (m_elastoplastic_model->to_restore() && !m_elastoplastic_model->is_plastic() && dist.head<3>().norm() < 1e-2 &&
      dist.tail<3>().norm() < 1.0 && m_parameters.impedance.plastic_restoration) {
    m_elastoplastic_model->restore();
    RCLCPP_INFO(get_node()->get_logger(), "Restore elastic state");
  }

  ClikData clik_data{.position_references = full_position_references,
                     .velocity_references = full_velocity_references,
                     .twist_tool_world_in_world = twist_tool_world_in_world,
                     .twist_shared_world_in_world = twist_shared_world_in_world,
                     .T_world_tool = T_world_tool,
                     .T_world_shared = m_T_world_shared,
                     .target_acc_tool_target_in_world = reference_target_acc_shared_world_in_world,
                     .J_world_tools_in_world = J_world_tool_in_world,
                     // .J_base_tools_in_world = J_base_tool_in_world,
                     .target_T_world_tool = reference_target_T_world_shared,
                     .target_twist_shared_world_in_world = reference_target_twist_shared_world_in_world,
                     .wrench_tool_in_world = wrench_tool_in_world,
                     .wrench_shared_in_world = wrench_shared_in_world,
                     .got_new_odom = got_new_odom};

  std::optional<Eigen::VectorXd> solution_qp = clik(clik_data);
  if (!solution_qp.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot find a solution for the CLIK QP problem");
    // this->on_deactivate(rclcpp_lifecycle::State());
    // throw std::runtime_error("Controller crashed");
    // m_qp.setZero();
    m_qpp.setZero();
  } else {
    Eigen::VectorXd qepp = solution_qp.value().head(m_full_nax);
    m_computed_target_acc_shared_world_in_world = solution_qp.value().tail<M_SE3>();
    m_qpp = qepp;
    std::tie(m_q, m_qp) = utils::rk4_double([](const auto&, const auto&, const auto& u) { return u; }, m_q, m_qp, qepp, m_dt);
  }

  if (m_mobile_base->enabled) {
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
      if (std::abs(m_velocity_base_in_base(idx)) > m_mobile_base->vel_limits(idx)) {
        RCLCPP_WARN_STREAM(this->get_node()->get_logger(),
                           "Saturation of Velocity on base linear direction "
                             << idx << ": " << m_velocity_base_in_base(idx) << " should be "
                             << utils::sgn(m_velocity_base_in_base(idx)) * m_mobile_base->vel_limits(idx));
        m_velocity_base_in_base(idx) = utils::sgn(m_velocity_base_in_base(idx)) * m_mobile_base->vel_limits(idx);
      }
    }
    m_qp.head<M_SE2>() = utils::base_velocity_from_twist(
      rdyn::spatialRotation(utils::twist_from_base_velocity(m_velocity_base_in_base), m_T_world_base.linear()));
    // END - Check Saturation Base
  }

  // BEGIN - Saturation Manipulator
  for (size_t idx = 0; idx < m_nax; ++idx) {
    // double q = m_q(idx + (m_full_nax - m_nax));
    // double dq = m_qp(idx + (m_full_nax - m_nax));
    m_q(idx + (m_full_nax - m_nax)) =
      std::max(m_limits.pos_lower(idx), std::min(m_limits.pos_upper(idx), m_q(idx + (m_full_nax - m_nax))));
    m_qp(idx + (m_full_nax - m_nax)) =
      std::max(-m_limits.vel(idx), std::min(m_limits.vel(idx), m_qp(idx + (m_full_nax - m_nax))));
    // if (!utils::almost_equal(q, m_q(idx + (m_full_nax - m_nax)))) {
    //   // RCLCPP_WARN(get_node()->get_logger(), "Saturation of POSITION on manipulator joint with index %ld", idx);
    // }
    // if (!utils::almost_equal(dq, m_qp(idx + (m_full_nax - m_nax)))) {
    //   // RCLCPP_WARN(get_node()->get_logger(), "Saturation of VELOCITY on manipulator joint with index %ld", idx);
    // }
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

  if (m_mobile_base->enabled) {
    Eigen::Vector6d base_twist_in_base = utils::twist_from_base_velocity(m_velocity_base_in_base);

    bool is_mobile_base_write_ok = write_cmd_vel(m_velocity_base_in_base);
    if (!is_mobile_base_write_ok) {
      write_cmd_vel(Eigen::Vector3d::Zero());
      LOG_ERROR_THROTTLE_COUNT(get_node()->get_logger(), get_node()->get_clock(), 1,
                               "Problem occurred while writing on mobile base interfaces! Stopping the movement");
    }

    m_T_world_base = rdyn::spatialIntegration(m_T_world_base, base_twist_in_base, m_dt);
  }

  // *************
  // ** PUBLISH **
  // *************
  using namespace elastoplastic_msgs::msg;

  auto time_now = this->get_node()->get_clock()->now();
  elastoplastic_msgs::msg::ElastoplasticDualControllerState msg;

  msg.header.stamp = time_now;
  msg.header.frame_id = m_parameters.frames.map;

  msg.z.reserve(6);
  msg.zp.reserve(6);
  Eigen::Vector6d tmpz = m_elastoplastic_model->z();
  std::copy(tmpz.begin(), tmpz.end(), std::back_inserter(msg.z));
  std::copy(m_zp.begin(), m_zp.end(), std::back_inserter(msg.zp));

  msg.cart_ref_pose = tf2::toMsg(reference_target_T_world_shared);
  msg.cart_ref_twist = tf2::toMsg(reference_target_twist_shared_world_in_world);

  // cmd
  msg.cart_actual_cmd_pose[ElastoplasticDualControllerState::SIDE_LEFT] =
    tf2::toMsg(m_chain_world_tools[Side::LEFT]->getTransformation(m_q(m_sel[Side::LEFT])));
  msg.cart_actual_cmd_pose[ElastoplasticDualControllerState::SIDE_RIGHT] =
    tf2::toMsg(m_chain_world_tools[Side::RIGHT]->getTransformation(m_q(m_sel[Side::RIGHT])));

  msg.cart_actual_cmd_twist[ElastoplasticDualControllerState::SIDE_LEFT] =
    tf2::toMsg(m_chain_world_tools[Side::LEFT]->getTwistTool(m_q(m_sel[Side::LEFT]), m_qp(m_sel[Side::LEFT])));
  msg.cart_actual_cmd_twist[ElastoplasticDualControllerState::SIDE_RIGHT] =
    tf2::toMsg(m_chain_world_tools[Side::RIGHT]->getTwistTool(m_q(m_sel[Side::RIGHT]), m_qp(m_sel[Side::RIGHT])));

  msg.cart_actual_cmd_acc[ElastoplasticDualControllerState::SIDE_LEFT] = tf2::toMsg(
    m_chain_world_tools[Side::LEFT]->getDTwistTool(m_q(m_sel[Side::LEFT]), m_qp(m_sel[Side::LEFT]), m_qpp(m_sel[Side::LEFT])));
  msg.cart_actual_cmd_acc[ElastoplasticDualControllerState::SIDE_RIGHT] =
    tf2::toMsg(m_chain_world_tools[Side::RIGHT]->getDTwistTool(m_q(m_sel[Side::RIGHT]), m_qp(m_sel[Side::RIGHT]),
                                                               m_qpp(m_sel[Side::RIGHT])));

  msg.cart_actual_shared_pose = tf2::toMsg(m_T_world_shared);
  msg.cart_actual_shared_twist = tf2::toMsg(twist_shared_world_in_world);

  [[maybe_unused]] double unused_double;
  std::tie(msg.reset_buffer_state, unused_double) = m_elastoplastic_model->get_reset_buffer_status();

  msg.wrenches[ElastoplasticDualControllerState::SIDE_LEFT] = utils::toWrenchMsg(wrench_tool_in_world.head<6>());
  msg.wrenches[ElastoplasticDualControllerState::SIDE_RIGHT] = utils::toWrenchMsg(wrench_tool_in_world.tail<6>());

  msg.admittance_state.wrench_base = utils::toWrenchStampedMsg(wrench_shared_in_world);

  msg.admittance_state.admittance_position = tf2::eigenToTransform(m_computed_target_T_world_shared);
  msg.admittance_state.admittance_position.header.stamp = time_now;
  msg.admittance_state.admittance_position.header.frame_id = m_parameters.frames.map;

  msg.admittance_state.admittance_velocity.twist = tf2::toMsg(m_computed_target_twist_shared_world_in_world);
  msg.admittance_state.admittance_velocity.header.stamp = time_now;
  msg.admittance_state.admittance_velocity.header.frame_id = m_parameters.frames.map;

  msg.admittance_state.admittance_acceleration.twist = tf2::toMsg(m_computed_target_acc_shared_world_in_world);
  msg.admittance_state.admittance_acceleration.header.stamp = time_now;
  msg.admittance_state.admittance_acceleration.header.frame_id = m_parameters.frames.map;

  msg.admittance_state.joint_state.header.stamp = time_now;
  msg.admittance_state.joint_state.name.reserve(m_joint_names.size() + m_mobile_base->nax());
  msg.admittance_state.joint_state.position.reserve(m_joint_names.size() + m_mobile_base->nax());
  msg.admittance_state.joint_state.velocity.reserve(m_joint_names.size() + m_mobile_base->nax());
  msg.admittance_state.joint_state.effort.reserve(m_joint_names.size() + m_mobile_base->nax());
  std::copy(m_mobile_base->base_joint_names.begin(), m_mobile_base->base_joint_names.end(),
            std::back_inserter(msg.admittance_state.joint_state.name));
  std::copy(m_joint_names.begin(), m_joint_names.end(), std::back_inserter(msg.admittance_state.joint_state.name));
  std::copy(m_q.begin(), m_q.end(), std::back_inserter(msg.admittance_state.joint_state.position));
  std::copy(m_qp.begin(), m_qp.end(), std::back_inserter(msg.admittance_state.joint_state.velocity));
  std::copy(m_qpp.begin(), m_qpp.end(), std::back_inserter(msg.admittance_state.joint_state.effort));

  msg.joint_reference.name.reserve(m_joint_names.size() + m_mobile_base->nax());
  msg.joint_reference.position.reserve(m_joint_names.size() + m_mobile_base->nax());
  msg.joint_reference.velocity.reserve(m_joint_names.size() + m_mobile_base->nax());
  std::copy(m_mobile_base->base_joint_names.begin(), m_mobile_base->base_joint_names.end(),
            std::back_inserter(msg.joint_reference.name));
  std::copy(m_joint_names.begin(), m_joint_names.end(), std::back_inserter(msg.joint_reference.name));
  std::copy(full_position_references.begin(), full_position_references.end(), std::back_inserter(msg.joint_reference.position));
  std::copy(full_velocity_references.begin(), full_velocity_references.end(), std::back_inserter(msg.joint_reference.velocity));

  msg.admittance_state.selected_axes.data.reserve(6);
  std::copy(m_elastoplastic_model->get_enabled_axis().begin(), m_elastoplastic_model->get_enabled_axis().end(),
            std::back_inserter(msg.admittance_state.selected_axes.data));

  msg.admittance_state.stiffness.data.reserve(6);
  msg.admittance_state.damping.data.reserve(6);
  auto [K, D] = m_elastoplastic_model->compute_variable_matrices(m_T_world_shared);
  Eigen::Vector6d K_diag = K.diagonal();
  std::copy(K_diag.begin(), K_diag.end(), std::back_inserter(msg.admittance_state.stiffness.data));
  std::copy(D.diagonal().begin(), D.diagonal().end(), std::back_inserter(msg.admittance_state.damping.data));

  utils::toWrenchMsg(m_admittance_value, msg.virtual_force);

  if (m_elastoplastic_model->is_plastic()) {
    msg.mode = ElastoplasticDualControllerState::MODE_PLASTIC;
  } else if (m_elastoplastic_model->to_restore()) {
    msg.mode = ElastoplasticDualControllerState::MODE_RESTORE;
  } else {
    msg.mode = ElastoplasticDualControllerState::MODE_ELASTIC;
  }

  if (m_rt_pub_full_state->trylock()) {
    m_rt_pub_full_state->msg_ = msg;
    m_rt_pub_full_state->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}


} // namespace elastoplastic

PLUGINLIB_EXPORT_CLASS(elastoplastic::ElastoplasticControllerDual, controller_interface::ChainableControllerInterface);
