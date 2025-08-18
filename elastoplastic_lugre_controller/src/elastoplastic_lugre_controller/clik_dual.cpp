#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller_dual.hpp"
#include "elastoplastic_lugre_controller/sot.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"

namespace elastoplastic {

void normalize(elastoplastic::Task& t) {
  auto [H, F] = t.update_task();
  t.W() *= 1 / std::sqrt(H.trace());
}

void whitening(elastoplastic::Task& t) {
  Eigen::MatrixXd M = t.A().colPivHouseholderQr().matrixQ();
  t.W() *= M * M.transpose();
};

std::optional<Eigen::VectorXd> ElastoplasticControllerDual::clik(const ClikData& data) {

  // RCLCPP_WARN_STREAM(get_node()->get_logger(), "**********" << data.twist_tool_world_in_world << "\n---\n"
  // << data.twist_shared_world_in_world << "\n---\n"
  // << data.T_world_tool[Side::LEFT].matrix() << "\n---\n"
  // << data.T_world_tool[Side::RIGHT].matrix() << "\n---\n"
  // << data.T_world_shared.matrix() << "\n---\n"
  // << data.target_acc_tool_target_in_world << "\n---\n"
  // << data.J_world_tool_in_world << "\n---\n"
  // << data.target_twist_tool_world_in_world << "\n---\n"
  // << data.wrench_tool_in_world << "\n---\n"
  // << data.wrench_shared_in_world << "\n---\n"
  // << data.target_T_world_tool.matrix());

  Eigen::Vector12d acc_non_linear_in_world;
  acc_non_linear_in_world.head<6>() =
    m_chain_world_tools[Side::LEFT]->getDTwistNonLinearPartTool(m_q(m_sel[Side::LEFT]), m_qp(m_sel[Side::LEFT]));
  acc_non_linear_in_world.tail<6>() =
    m_chain_world_tools[Side::RIGHT]->getDTwistNonLinearPartTool(m_q(m_sel[Side::RIGHT]), m_qp(m_sel[Side::RIGHT]));

  auto t_start_qp = get_node()->get_clock()->now();
  const unsigned int prb_dim = m_full_nax + M_SE3;
  Eigen::Vector6d enabled_axis = m_elastoplastic_model->get_enabled_axis();

  elastoplastic::Task task_cart_vel(prb_dim, M_SE3);
  elastoplastic::Task task_cart_pos(prb_dim, M_SE3);
  elastoplastic::Task task_minimize_cart_acc(prb_dim, M_SE3);
  elastoplastic::Task task_minimize_cart_vel(prb_dim, M_SE3);
  elastoplastic::Task task_joint_pos(prb_dim, m_full_nax);
  elastoplastic::Task task_joint_vel(prb_dim, m_full_nax);
  elastoplastic::Task task_minimize_joint_acc(prb_dim, m_full_nax);
  elastoplastic::Task task_admittance(prb_dim, M_SE3);


  /**********************
   ** Task Definitions **
   **********************/

  Eigen::Matrix612d idn612;
  idn612 << Eigen::Matrix6d::Identity(), -Eigen::Matrix6d::Identity();

  // elastoplastic::Task task_ref_relative(prb_dim, M_SE3);
  // task_ref_relative.A().middleCols<M_SE3>(m_full_nax) << m_dt;
  // task_ref_relative.b() << idn612 * data.twist_tool_world_in_world;

  // elastopastoplastic::Task task_cart_relative(prb_dim, m_full_nax);
  // task_cart_relative.A().leftCols(m_full_nax) << data.J_world_tool_in_world

  // Task Cartesian : Minimize cartesian distance from reference twist
  task_cart_vel.A().rightCols(M_SE3) = Eigen::Matrix6d::Identity() * m_dt;
  task_cart_vel.b() = (m_computed_target_twist_shared_world_in_world - data.target_twist_shared_world_in_world);
  normalize(task_cart_vel);
  // whitening(task_cart_vel);

  // Task cartesian: relative distances between end effectors
  elastoplastic::Task task_keep_relative_tf(prb_dim, M_SE3);
  Eigen::Vector6d dist_right_left_ideal_in_shared, dist_right_left_ideal_in_world;
  utils::get_frame_distance(m_T_left_shared.inverse(), m_T_right_shared_ideal.inverse(), dist_right_left_ideal_in_shared);
  dist_right_left_ideal_in_world = rdyn::spatialRotation(dist_right_left_ideal_in_shared, data.T_world_shared.linear());
  Eigen::Vector6d dist_right_left_in_world;
  utils::get_frame_distance(data.T_world_tool[Side::LEFT], data.T_world_tool[Side::RIGHT], dist_right_left_in_world);
  task_keep_relative_tf.A().leftCols(m_full_nax) = idn612 * data.J_world_tools_in_world * m_dt * m_dt * 0.5;
  task_keep_relative_tf.b() = idn612 * acc_non_linear_in_world * m_dt * m_dt * 0.5 +
                              idn612 * data.twist_tool_world_in_world * m_dt + dist_right_left_in_world -
                              dist_right_left_ideal_in_world;
  normalize(task_keep_relative_tf);
  // whitening(task_keep_relative_tf);

  elastoplastic::Task task_keep_relative_vel(prb_dim, M_SE3);
  task_keep_relative_vel.A().leftCols(m_full_nax) = idn612 * data.J_world_tools_in_world * m_dt;
  task_keep_relative_vel.b() = idn612 * acc_non_linear_in_world * m_dt + idn612 * data.twist_tool_world_in_world;
  normalize(task_keep_relative_vel);
  // whitening(task_keep_relative_vel);

  // Task Cartesian : Minimize difference between the real target and the computed one
  Eigen::Vector6d ref_p_err;
  utils::get_frame_distance(m_computed_target_T_world_shared, data.target_T_world_tool, ref_p_err);
  task_cart_pos.A().rightCols<6>() = Eigen::Matrix6d::Identity() * 0.5 * std::pow(m_dt, 2);
  task_cart_pos.b() << ref_p_err + m_computed_target_twist_shared_world_in_world * m_dt;
  normalize(task_cart_pos);
  // whitening(task_cart_pos);

  // Task Cartesian:
  // elastoplastic::Task task_minimize_cart_vel(prb_dim, M_SE3);
  // task_minimize_cart_vel.A().rightCols(M_SE3) = Eigen::Matrix6d::Identity() * m_dt;
  // task_minimize_cart_vel.b() << data.twist_shared_world_in_world;

  task_minimize_cart_acc.A().rightCols(M_SE3).setIdentity();
  task_minimize_cart_acc.b().setZero();
  normalize(task_minimize_cart_acc);
  // whitening(task_minimize_cart_acc);

  elastoplastic::Task task_minimize_jerk(prb_dim, M_SE3);
  task_minimize_jerk.A().middleCols<M_SE3>(m_full_nax) = Eigen::Matrix6d::Identity() / m_dt;
  task_minimize_jerk.b() = -m_computed_target_acc_shared_world_in_world / m_dt;
  normalize(task_minimize_jerk);
  // whitening(task_minimize_jerk);

  // Task: Admittance
  auto [K, D] = m_elastoplastic_model->compute_variable_matrices(data.T_world_shared);
  auto invM = m_elastoplastic_model->get_inertia_inv();
  Eigen::Vector6d twist_error_shared_world_in_world =
    data.twist_shared_world_in_world - m_computed_target_twist_shared_world_in_world;
  Eigen::Vector6d pose_error_shared_world_in_world;
  utils::get_frame_distance(data.T_world_shared, m_computed_target_T_world_shared, pose_error_shared_world_in_world);

  Eigen::Matrix6d adm = Eigen::Matrix6d::Identity() + invM * D * m_dt + 0.5 * invM * K * std::pow(m_dt, 2);
  task_admittance.A() << adm * m_grasp_matrix_twist * data.J_world_tools_in_world, -adm;
  task_admittance.b() << adm * m_grasp_matrix_twist * acc_non_linear_in_world + invM * D * twist_error_shared_world_in_world +
                           invM * K *
                             (twist_error_shared_world_in_world * m_dt + m_elastoplastic_model->z() +
                              pose_error_shared_world_in_world.cwiseProduct(Eigen::Vector6d::Ones() - enabled_axis)) -
                           invM * (data.wrench_shared_in_world);
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
    // m_W.diagonal().tail(m_nax) *= weight_coeff;
  }

  // Task: Minimize joint acceleration and weighting
  task_minimize_joint_acc.A().leftCols(m_full_nax).setIdentity();
  task_minimize_joint_acc.b().setZero();
  task_minimize_joint_acc.W() = m_W.transpose() * m_W;
  normalize(task_minimize_joint_acc);
  // whitening(task_minimize_joint_acc);


  // Task: Joint Velocity
  task_joint_vel.A().leftCols(m_full_nax) += -Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  task_joint_vel.b() += (data.velocity_references - m_qp);
  normalize(task_joint_vel);
  // whitening(task_joint_vel);
  task_joint_vel.W() *= m_W.transpose() * m_W;

  // Task: Joint Position
  task_joint_pos.A().leftCols(m_full_nax) += -0.5 * Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * std::pow(m_dt, 2);
  task_joint_pos.b() += (data.position_references - (m_q + m_qp * m_dt));
  normalize(task_joint_pos);
  // whitening(task_joint_pos);
  task_joint_pos.W() *= m_W.transpose() * m_W;

  // Task: Joint reference
  elastoplastic::Task task_joint_reference(prb_dim, m_full_nax);
  task_joint_reference.A().leftCols(m_full_nax) =
    Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * (1 + m_kv_joint_task * m_dt + m_kp_joint_task * 0.5 * m_dt * m_dt);
  task_joint_reference.b() = m_kv_joint_task * m_qp + m_kp_joint_task * (m_q + m_qp * m_dt - data.position_references);
  normalize(task_joint_pos);
  // whitening(task_joint_pos);
  task_joint_reference.W() *= m_W.transpose() * m_W;

  elastoplastic::Task task_minimize_joint_vel(prb_dim, m_full_nax);
  task_minimize_joint_vel.A().leftCols(m_full_nax) << Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  task_minimize_joint_vel.b() << m_qp;
  normalize(task_minimize_joint_vel);
  // whitening(task_minimize_joint_vel);
  task_minimize_joint_vel.W() *= m_W.transpose() * m_W;

  /*
   * Test tasks
   */

  /****************
   ** Task Stack **
   ****************/
  constexpr double STACK_LEVEL_STEP = 1e-2;
  constexpr int STACK_LEVEL_ZERO = 0;
  elastoplastic::Stack sot(prb_dim, STACK_LEVEL_STEP, STACK_LEVEL_ZERO);

  /* Variable stack */
  constexpr int CART_POS_LEVEL_OFFSET = 2;
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
  }
  sot.insert_task(task_cart_pos, cart_pos_level, 1);

  /* Constant stack */
  sot.push_task(task_cart_vel, 1);
  sot.new_level();
  sot.push_task(task_minimize_jerk);
  sot.push_task(task_minimize_cart_acc);
  sot.push_task(task_keep_relative_vel);
  sot.new_level();
  sot.push_task(task_admittance);
  // sot.new_level();
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
  elastoplastic::Task task_fixed_torso(prb_dim, m_split_nax[Side::COMMON]);
  // task_fixed_torso.A().middleCols(m_idx_st[Side::COMMON], m_split_nax[Side::COMMON]) =
  // Eigen::MatrixXd::Identity(m_split_nax[Side::COMMON], m_split_nax[Side::COMMON]) * std::pow(m_dt, 2) * 0.5;
  // task_fixed_torso.b().segment(m_idx_st[Side::COMMON], m_split_nax[Side::COMMON]) =
  // m_qp(Eigen::seqN(m_idx_st[Side::COMMON], m_split_nax[Side::COMMON])) +
  // m_q(Eigen::seqN(m_idx_st[Side::COMMON], m_split_nax[Side::COMMON])) -
  // m_initial_q(Eigen::seqN(m_idx_st[Side::COMMON], m_split_nax[Side::COMMON]));

  task_fixed_torso.A().middleCols(m_idx_st[Side::COMMON], m_split_nax[Side::COMMON]) =
    Eigen::MatrixXd::Identity(m_split_nax[Side::COMMON], m_split_nax[Side::COMMON]) * m_dt;
  task_fixed_torso.b().segment(m_idx_st[Side::COMMON], m_split_nax[Side::COMMON]) =
    m_qp(Eigen::seqN(m_idx_st[Side::COMMON], m_split_nax[Side::COMMON]));

  elastoplastic::EqualitySet eq_set(prb_dim);
  // eq_set.push_constraint(task_admittance);
  eq_set.push_constraint(task_fixed_torso);
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
  elastoplastic::InequalityConstraint ineq_x_max(prb_dim, M_SE3, "Cartesian Position Max");
  elastoplastic::InequalityConstraint ineq_x_min(prb_dim, M_SE3, "Cartesian Position Min");
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

  // Cartesian Constraints
  // Position
  // ineq_x_min.CI().middleCols<M_SE3>(m_full_nax) = Eigen::Matrix6d::Identity() * 0.5 * m_dt * m_dt;
  // ineq_x_min.ci() = m_computed_target_twist_shared_world_in_world * m_dt +
  // utils::vector_from_affine(m_computed_target_T_world_shared) +
  // utils::vector_from_affine(m_T_world_base * utils::affine_from_vector(m_cartesian_pos_limits));

  // ineq_x_max.CI().middleCols<M_SE3>(m_full_nax) = -Eigen::Matrix6d::Identity() * 0.5 * m_dt * m_dt;
  // ineq_x_max.ci() = -m_computed_target_twist_shared_world_in_world * m_dt -
  // utils::vector_from_affine(m_computed_target_T_world_shared) +
  // utils::vector_from_affine(m_T_world_base * utils::affine_from_vector(m_cartesian_pos_limits));

  // Acceleration
  ineq_xpp_min.CI().middleCols<M_SE3>(m_full_nax) = Eigen::Matrix6d::Identity();
  ineq_xpp_min.ci() = Eigen::VectorXd::Constant(6, 10);

  ineq_xpp_max.CI().middleCols<M_SE3>(m_full_nax) = -Eigen::Matrix6d::Identity();
  ineq_xpp_max.ci() = Eigen::VectorXd::Constant(6, 10);

  // Move base limits to world
  if (m_mobile_base.enabled) {
    // Velocity
    Eigen::Vector6d max_vel_base_in_world = utils::twist_from_base_velocity(m_mobile_base.vel_limits);
    Eigen::Vector6d max_vel_base_in_base = rdyn::spatialRotation(max_vel_base_in_world, m_T_world_base.linear().transpose());
    Eigen::Vector3d max_vel_base = utils::base_velocity_from_twist(max_vel_base_in_base);
    ineq_qp_min.ci().head<M_SE2>() << m_qp.head<M_SE2>() + max_vel_base;
    ineq_qp_max.ci().head<M_SE2>() << max_vel_base - m_qp.head<M_SE2>();

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
  // ineq_set.push_constraint(ineq_x_min);
  // ineq_set.push_constraint(ineq_x_max);
  ineq_set.push_constraint(ineq_xpp_min);
  ineq_set.push_constraint(ineq_xpp_max);
  ineq_set.compute_set();

  /***********
   ** Solve **
   ***********/
  sot.symmetrize();
  sot.regularize();
  // LOG_ERROR_THROTTLE_COUNT(get_node()->get_logger(), get_node()->get_clock(), 2.0,
  //                          "There are " << ineq_set.redundancies() << " in the inquality constraints");
  elastoplastic::SolverQP solver(prb_dim, sot, eq_set, ineq_set);
  auto [solutionQP, status] = solver.solve();

  if (status != SolverStatus::EIQUADPROG_FAST_OPTIMAL) {
    // RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Problem unfeasible. Solver status: " << status);
    LOG_ERROR_THROTTLE_COUNT(get_node()->get_logger(), get_node()->get_clock(), 1.0,
                             "Problem unfeasible. Solver status: " << status);
    return std::nullopt;
  }

  if (solutionQP.hasNaN()) {
    RCLCPP_ERROR(get_node()->get_logger(), "NaN in the solution!");
    RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Dump: "
                                                    << "\n## first round sol [qpp(" << m_full_nax << "), slack("
                                                    << prb_dim - m_full_nax << ")]##\n"
                                                    << solutionQP.transpose() << "\n## first round ret ##\n"
                                                    << status << "## G ## " << sot.G() << "\n## F ##" << sot.F().transpose()
                                                    << "\n## eq_set.CE() ## " << eq_set.CE() << "\n ## eq_set.ce() ## "
                                                    << eq_set.ce().transpose() << "\n## CI ## " << ineq_set.CI() << "\n## ci ##"
                                                    << ineq_set.ci().transpose());
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

  m_admittance_value = task_admittance.value(solutionQP) + invM * (data.wrench_shared_in_world);
  RCLCPP_WARN_STREAM(get_node()->get_logger(), "task admittance residue: " << task_admittance.value(solutionQP) << " - norm: "
                                                                           << task_admittance.value(solutionQP).norm());
  return solutionQP;
}

} // namespace elastoplastic
