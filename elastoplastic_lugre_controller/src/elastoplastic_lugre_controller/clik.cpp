#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"
#include "elastoplastic_lugre_controller/sot.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"
#include <algorithm>

namespace elastoplastic {

void normalize(elastoplastic::Task& t) {
  auto [H, F] = t.update_task();
  t.W() *= 1 / std::sqrt(H.trace());
}

void whitening(elastoplastic::Task& t) {
  Eigen::MatrixXd M = t.A().colPivHouseholderQr().matrixQ();
  t.W() *= M * M.transpose();
};

std::optional<Eigen::VectorXd> ElastoplasticController::clik(const ClikData& data) {

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
  // elastoplastic::Task task_admittance(prb_dim, M_SE3);

  /**********************
   ** Task Definitions **
   **********************/

  // Task Cartesian : Minimize cartesian distance from reference twist
  task_cart_vel.A().middleCols<M_SE3>(m_full_nax) = Eigen::Matrix6d::Identity() * m_dt;
  task_cart_vel.b() = (m_computed_target_twist_tool_world_in_world - data.target_twist_tool_world_in_world);
  normalize(task_cart_vel);
  // whitening(task_cart_vel);

  // Task Cartesian : Minimize difference between the real target and the computed one
  Eigen::Vector6d ref_p_err;
  rdyn::getFrameDistanceQuat(m_computed_target_T_world_tool, data.target_T_world_tool, ref_p_err);
  task_cart_pos.A().middleCols<M_SE3>(m_full_nax) = Eigen::Matrix6d::Identity() * 0.5 * std::pow(m_dt, 2);
  task_cart_pos.b() << ref_p_err + m_computed_target_twist_tool_world_in_world * m_dt;
  normalize(task_cart_pos);
  // whitening(task_cart_pos);

  // Task Cartesian:
  elastoplastic::Task task_minimize_cart_vel(prb_dim, M_SE3);
  task_minimize_cart_vel.A().middleCols<M_SE3>(m_full_nax) = Eigen::Matrix6d::Identity() * m_dt;
  task_minimize_cart_vel.b() << data.twist_tool_world_in_world;
  normalize(task_minimize_cart_vel);
  // whitening(task_minimize_cart_vel);

  task_minimize_cart_acc.A().middleCols<M_SE3>(m_full_nax).setIdentity();
  task_minimize_cart_acc.b().setZero();
  normalize(task_minimize_cart_acc);
  // whitening(task_minimize_cart_acc);

  elastoplastic::Task task_minimize_jerk(prb_dim, M_SE3);
  task_minimize_jerk.A().middleCols<M_SE3>(m_full_nax) = Eigen::Matrix6d::Identity() / m_dt;
  task_minimize_jerk.b() = -m_computed_target_acc_tool_world_in_world / m_dt;
  normalize(task_minimize_jerk);
  // whitening(task_minimize_jerk);

  // Task: Admittance
  auto [K, D] = m_elastoplastic_model->compute_variable_matrices(data.T_world_tool);
  auto invM = m_elastoplastic_model->get_inertia_inv();
  Eigen::Vector6d twist_error_tool_world_in_world = data.twist_tool_world_in_world - m_computed_target_twist_tool_world_in_world;
  Eigen::Vector6d pose_error_tool_world_in_world;
  rdyn::getFrameDistance(data.T_world_tool, m_computed_target_T_world_tool, pose_error_tool_world_in_world);

  Eigen::Matrix6d adm = Eigen::Matrix6d::Identity() + invM * D * m_dt + 0.5 * invM * K * std::pow(m_dt, 2);
  elastoplastic::Task task_admittance(prb_dim, M_SE3);
  task_admittance.A() << adm * data.J_world_tool_in_world, -adm;
  task_admittance.b() << adm * acc_non_linear_in_world + invM * D * twist_error_tool_world_in_world +
                           invM * K *
                             (twist_error_tool_world_in_world * m_dt + m_elastoplastic_model->z() +
                              pose_error_tool_world_in_world.cwiseProduct(Eigen::Vector6d::Ones() - enabled_axis)) -
                           invM * (data.wrench_tool_in_world);
  normalize(task_admittance);
  // whitening(task_admittance);

  // Weighting matrix
  m_W.setIdentity();
  if (m_mobile_base.enabled) {
    auto logis = 1;
    // auto logis = m_logistic.get(m_velocity_base_in_base);
    // m_logis_prec = logis;
    m_W.diagonal()(2) *= 1e6;
    m_W.diagonal().head<2>() *= 1e3;
    double weight_coeff = (1.0 + m_parameters.clik.alpha_gain * m_elastoplastic_model->alpha() * logis);
    m_W.diagonal().head<2>() /= weight_coeff;
  }

  // Task: Minimize joint acceleration and weighting
  task_minimize_joint_acc.A().leftCols(m_full_nax).setIdentity();
  task_minimize_joint_acc.b().setZero();
  normalize(task_minimize_joint_acc);
  task_minimize_joint_acc.W() = m_W.transpose() * m_W;
  // whitening(task_minimize_joint_acc);


  // Task: Joint Velocity
  task_joint_vel.A().leftCols(m_full_nax) += -Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  task_joint_vel.b() += (data.velocity_references - m_qp);
  normalize(task_joint_vel);
  task_joint_vel.W() *= m_W.transpose() * m_W;
  // whitening(task_joint_vel);

  // Task: Joint Position
  task_joint_pos.A().leftCols(m_full_nax) += -0.5 * Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * std::pow(m_dt, 2);
  task_joint_pos.b() += (data.position_references - (m_q + m_qp * m_dt));
  normalize(task_joint_pos);
  task_joint_pos.W() *= m_W.transpose() * m_W;
  // whitening(task_joint_pos);

  // Task: Joint reference
  elastoplastic::Task task_joint_reference(prb_dim, m_full_nax);
  task_joint_reference.A().leftCols(m_full_nax) =
    Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * (1 + m_kv_joint_task * m_dt + m_kp_joint_task * 0.5 * m_dt * m_dt);
  task_joint_reference.b() = m_kv_joint_task * m_qp + m_kp_joint_task * (m_q + m_qp * m_dt - data.position_references);
  normalize(task_joint_reference);
  task_joint_reference.W() *= m_W.transpose() * m_W;
  // whitening(task_joint_reference);

  elastoplastic::Task task_minimize_joint_vel(prb_dim, m_full_nax);
  task_minimize_joint_vel.A().leftCols(m_full_nax) << Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  task_minimize_joint_vel.b() << m_qp;
  normalize(task_minimize_joint_vel);
  task_minimize_joint_vel.W() *= m_W.transpose() * m_W;
  // whitening(task_minimize_joint_vel);

  /*
    elastoplastic::Task task_keep_T_base_tool(prb_dim, 3);
    Eigen::Affine3d T_base_tool = m_chain_base_tool->getTransformation(m_q.tail(m_nax));
    task_keep_T_base_tool.A().middleCols<3>(m_full_nax) = Eigen::Matrix3d::Identity() * 0.5 * m_dt * m_dt;
    task_keep_T_base_tool.b() = (T_base_tool.translation().head<3>() - m_t_base_tool) +
                                m_chain_base_tool->getTwistTool(m_q.tail(m_nax), m_qp.tail(m_nax)).head<3>();

    elastoplastic::Task task_base_desired(prb_dim, 2);
    Eigen::Vector6d base_to_ideal_base;
    rdyn::getFrameDistance(m_T_world_base, (m_chain_world_tool->getTransformation(m_initial_q) * T_base_tool.inverse()),
                           base_to_ideal_base);
    task_base_desired.A().leftCols<2>() = Eigen::Matrix2d::Identity() * 0.5 * m_dt * m_dt;
    task_base_desired.A().middleCols<2>(m_full_nax) = Eigen::Matrix2d::Identity() * 0.5 * m_dt * m_dt;
    task_base_desired.b() = base_to_ideal_base.head<2>() + m_qp.head<2>() * m_dt;
  */

  /****************
   ** Task Stack **
   ****************/
  constexpr double STACK_LEVEL_STEP = 1e-3;
  constexpr int STACK_LEVEL_ZERO = 0;
  elastoplastic::Stack sot(prb_dim, STACK_LEVEL_STEP, STACK_LEVEL_ZERO);

  /* Variable stack */
  constexpr int CART_POS_LEVEL_OFFSET = 3;
  int cart_pos_level = STACK_LEVEL_ZERO + CART_POS_LEVEL_OFFSET;
  if (!m_elastoplastic_model->is_plastic() && m_elastoplastic_model->to_restore() && m_parameters.impedance.plastic_restoration) {
    RCLCPP_DEBUG_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1, "Is restoring");
    cart_pos_level = STACK_LEVEL_ZERO;
    m_W.setIdentity();
    // m_W.diagonal().head<M_SE2>() *= 1 / .head<2>().norm();
    task_joint_pos.W().setIdentity();
    task_joint_vel.W() = m_W.transpose() * m_W;
    task_minimize_joint_vel.W() = m_W.transpose() * m_W;
    task_minimize_joint_acc.W() = m_W.transpose() * m_W;
    // sot.push_task(task_base_desired);
    sot.insert_task(task_cart_pos, cart_pos_level, 1);
  }

  /* Constant stack */
  sot.push_task(task_cart_vel, 1);
  sot.new_level();
  sot.push_task(task_minimize_jerk);
  sot.push_task(task_minimize_cart_acc);
  sot.new_level();
  sot.push_task(task_admittance);
  sot.new_level();
  sot.push_task(task_joint_vel, m_kv_joint_task);
  sot.push_task(task_joint_pos, m_kp_joint_task);
  // sot.push_task(task_joint_reference);
  sot.new_level();
  sot.push_task(task_minimize_joint_vel);
  sot.push_task(task_minimize_joint_acc, 1e-1);

  /********************
   ** EQ Constraints **
   ********************/
  elastoplastic::EqualitySet eq_set(prb_dim);
  // eq_set.push_constraint(task_admittance);

  //  elastoplastic::Task task_can_update_base(prb_dim, M_SE2);
  //  if (m_mobile_base.enabled) {
  //    // Eq Constraint
  //    // NOTE: Not the best, but I do not have other ideas on how to sync this with the mobile base controller
  //    if (!a_data.got_new_odom) {
  //      task_can_update_base.A().leftCols<M_SE2>().setIdentity();
  //      task_can_update_base.b().setZero();
  //    }
  //    eq_set.push_constraint(task_can_update_base);
  //  }

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
  elastoplastic::InequalityConstraint ineq_xpp_max(prb_dim, M_SE3, "Cartesian Acceleration Max");
  elastoplastic::InequalityConstraint ineq_xpp_min(prb_dim, M_SE3, "Cartesian Acceleration Min");

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

  // Acceleration
  ineq_xpp_min.CI().middleCols<M_SE3>(m_full_nax) = Eigen::Matrix6d::Identity();
  ineq_xpp_min.ci() = Eigen::VectorXd::Constant(6, 10);

  ineq_xpp_max.CI().middleCols<M_SE3>(m_full_nax) = -Eigen::Matrix6d::Identity();
  ineq_xpp_max.ci() = Eigen::VectorXd::Constant(6, 10);

  // Move base limits to world
  if (m_mobile_base.enabled) {
    // BUG: vel limits comes in base from parameters
    // Velocity
    Eigen::Vector6d max_vel_base_in_world = utils::twist_from_base_velocity(m_mobile_base.vel_limits);
    Eigen::Vector6d max_vel_base_in_base = rdyn::spatialRotation(max_vel_base_in_world, m_T_world_base.linear().transpose());
    Eigen::Vector3d max_vel_base = utils::base_velocity_from_twist(max_vel_base_in_base);
    ineq_qp_min.ci().head<M_SE2>() << m_qp.head<M_SE2>() + max_vel_base;
    ineq_qp_max.ci().head<M_SE2>() << max_vel_base - m_qp.head<M_SE2>();

    // BUG: acc limits comes in base from parameters
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
  ineq_set.push_constraint(ineq_xpp_min);
  ineq_set.push_constraint(ineq_xpp_max);
  ineq_set.compute_set();

  /***********
   ** Solve **
   ***********/
  sot.symmetrize();
  sot.regularize();
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
  constexpr double VIOLATION_TOLL = 1e-3;
  if (ineq_set.violations(solutionQP, VIOLATION_TOLL) != 0) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Constraint violated:");
    auto ineq_violated = ineq_set.which_violations(solutionQP, VIOLATION_TOLL);
    std::for_each(ineq_violated.begin(), ineq_violated.end(), [this, &solutionQP](const InequalityConstraint& ineq) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          " - " << ineq.description() << " | values: " << ineq.value(solutionQP).transpose());
    });
    return std::nullopt;
  }

  m_admittance_value = task_admittance.value(solutionQP) + invM * (data.wrench_tool_in_world);
  return solutionQP;
}
} // namespace elastoplastic
