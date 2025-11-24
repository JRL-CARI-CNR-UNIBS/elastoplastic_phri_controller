#include "adaptive_hqp/adaptive_hqp.hpp"
#include "elastoplastic_lugre_controller/sot.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/src/Eigenvalues/EigenSolver.h>
#include <algorithm>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/fwd.hpp>

namespace elastoplastic {

void normalize(elastoplastic::Task &t) {
  Eigen::MatrixXd H = t.A().transpose() * t.A();
  t.W() *= std::max(utils::K_ABS_EPSILON,
                    std::sqrt(t.A().rows()) / std::sqrt(H.trace()));
}

double normalize2(const elastoplastic::Task &t) {
  Eigen::MatrixXd H = t.A().transpose() * t.A();
  return std::pow(std::max(utils::K_ABS_EPSILON,
                           std::sqrt(t.A().rows()) / std::sqrt(H.trace())),
                  2.0);
}

std::optional<Eigen::VectorXd> AdaptiveHQP::clik(const ClikData &data) {
  constexpr static double WRENCH_THRESH = 1.0;

  Eigen::VectorXd q(m_full_nax + 1), qp(m_full_nax), qpp(m_full_nax);
  Eigen::VectorXd q_in(m_full_nax + 1), qp_in(m_full_nax);
  q = to_pinocchio_config(m_q);
  qp = m_qp;
  qpp = m_qpp;

  q_in = to_pinocchio_config(m_q_in);
  qp_in = m_qp_in;

  pin::computeForwardKinematicsDerivatives(
      m_chain_world_tool_model, m_chain_world_tool_data, q_in, qp_in, qpp);
  pin::computeJointJacobians(m_chain_world_tool_model, m_chain_world_tool_data,
                             q_in);
  pin::computeJointJacobiansTimeVariation(m_chain_world_tool_model,
                                          m_chain_world_tool_data, q_in, qp_in);
  pin::updateFramePlacements(m_chain_world_tool_model, m_chain_world_tool_data);

  Eigen::Matrix6Xd fdJ(6, m_full_nax), dJ(6, m_full_nax);
  pin::getFrameJacobianTimeVariation(m_chain_world_tool_model,
                                     m_chain_world_tool_data, m_tool_id,
                                     pin::LOCAL_WORLD_ALIGNED, fdJ);
  dJ = fdJ;
  // dJ(Eigen::all) = fdJ(Eigen::all, m_jnt_id);
  // dJ.leftCols<3>() = fdJ(Eigen::all, );

  Eigen::Vector6d acc_non_linear_in_world = dJ * m_qp_in;

  Eigen::Matrix6Xd fJ(6, m_full_nax), J(6, m_full_nax);
  pin::computeFrameJacobian(m_chain_world_tool_model, m_chain_world_tool_data,
                            q_in, m_tool_id, pin::LOCAL_WORLD_ALIGNED, fJ);
  J = fJ;
  // J.rightCols(m_nax) = fJ(Eigen::all, m_jnt_id);
  // J.leftCols<3>() = fJ(Eigen::all, 3);

  Eigen::Vector6d enabled_axis = m_elastoplastic_model->get_enabled_axis();

  auto t_start_qp = get_node()->get_clock()->now();
  const unsigned int prb_dim = 2 * m_full_nax + M_SE3; // qpp, tau, forces

  //=== Dynamics
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(m_full_nax, m_full_nax);
  M.bottomRightCorner(m_nax, m_nax) =
      pin::crba(m_chain_world_tool_model, m_chain_world_tool_data, q_in)
          .bottomRightCorner(m_nax, m_nax);
  M.bottomRightCorner(m_nax, m_nax) =
      M.bottomRightCorner(m_nax, m_nax).selfadjointView<Eigen::Upper>();
  M.topLeftCorner<3, 3>() =
      Eigen::DiagonalMatrix<double, 3>(
          Eigen::Map<Eigen::Vector3d>(
              m_parameters.mobile_base.virtual_inertia.data(),
              m_parameters.mobile_base.virtual_inertia.size()))
          .toDenseMatrix();

  Eigen::MatrixXd C(m_full_nax, m_full_nax);
  C = pin::computeCoriolisMatrix(m_chain_world_tool_model,
                                 m_chain_world_tool_data, q_in, qp_in);
  C.topLeftCorner<3, 3>() =
      Eigen::DiagonalMatrix<double, 3>(
          Eigen::Map<Eigen::Vector3d>(
              m_parameters.mobile_base.virtual_damping.data(),
              m_parameters.mobile_base.virtual_damping.size()))
          .toDenseMatrix();

  Eigen::VectorXd grav = pin::computeGeneralizedGravity(
      m_chain_world_tool_model, m_chain_world_tool_data, q_in);
  //   grav.head<3>().setZero();

  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),
                      "M\n"
                          << M << "\nC\n"
                          << C << "\ng\n"
                          << grav << "\nJ^T f\n"
                          << J.transpose() * data.wrench_tool_in_world);

  //=== Tikhonov
  elastoplastic::Task task_tikhonov_regular(prb_dim, prb_dim);
  task_tikhonov_regular.A() = Eigen::MatrixXd::Identity(prb_dim, prb_dim);
  task_tikhonov_regular.W().diagonal().segment(m_full_nax, m_full_nax) *= 1e-1;
  task_tikhonov_regular.W().diagonal().tail<6>() *= 1e-2;
  //   task_tikhonov_regular.W().topLeftCorner(m_full_nax, m_full_nax) *= 1e0;
  //   task_tikhonov_regular.W().block(m_full_nax, m_full_nax, m_full_nax,
  //                                   m_full_nax) *= 1e-2;
  //   task_tikhonov_regular.W().bottomRightCorner<6, 6>() *= 1e-1;

  //=== Task motion tracking
  elastoplastic::Task task_motion_tracking(prb_dim, 6, "Motion tracking");

  Eigen::Matrix6d invMc = m_elastoplastic_model->get_inertia_inv();
  auto [K, D] = m_elastoplastic_model->compute_variable_matrices(
      Eigen::Affine3d::Identity());

  task_motion_tracking.A().leftCols(m_full_nax) = J;
  task_motion_tracking.A().rightCols<6>() = -invMc;

  Eigen::Vector6d dist =
      utils::get_frame_distance(data.target_T_world_tool, data.T_world_tool);
  task_motion_tracking.b() = acc_non_linear_in_world -
                             data.target_acc_tool_target_in_world -
                             invMc * D *
                                 (data.target_twist_tool_world_in_world -
                                  data.twist_tool_world_in_world) -
                             invMc * K * dist;
  normalize(task_motion_tracking);

  //=== Task admittance
  elastoplastic::Task task_pseudo_admittance(prb_dim, 6, "Admittance");
  task_pseudo_admittance.A().rightCols<6>().setIdentity();
  task_pseudo_admittance.b() = -data.wrench_tool_in_world;
  normalize(task_pseudo_admittance);

  //=== Task gravity - only on arm
  // elastoplastic::Task task_gravity(prb_dim, m_nax, "Gravity");
  elastoplastic::Task task_gravity(prb_dim, m_full_nax, "Gravity");
  task_gravity
      .A()
      // .middleCols(m_full_nax + 3, m_nax)
      .middleCols(m_full_nax, m_full_nax)
      .setIdentity();
  task_gravity.b() = -grav;
  // task_gravity.b() = -grav.tail(m_nax);
  normalize(task_gravity);

  //=== Task minimize velocity
  elastoplastic::Task task_joint_vel(prb_dim, m_full_nax,
                                     "Joint velocity minimization");
  task_joint_vel.A().leftCols(m_full_nax) +=
      Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  task_joint_vel.b() += qp_in;
  normalize(task_joint_vel);

  //=== Task fix position
  elastoplastic::Task task_joint_pos(prb_dim, m_full_nax, "Joint position");
  task_joint_pos.A().leftCols(m_full_nax) +=
      Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt * m_dt * 0.5;
  task_joint_pos.b() += qp * m_dt + m_q_in - data.position_references;

  /***********
   ** Stack **
   ***********/
  elastoplastic::Stack sot1(prb_dim);
  elastoplastic::Stack sot2(prb_dim);
  elastoplastic::Stack sot3(prb_dim);

  const bool is_force_active =
      true; // data.wrench_tool_in_world.norm() > WRENCH_THRESH;
  if (is_force_active) {
    sot1.push_task(task_pseudo_admittance);
    sot2.push_task(task_gravity);
    sot3.push_task(task_joint_vel);
  } else {
    sot1.push_task(task_motion_tracking);
    sot2.push_task(task_pseudo_admittance);
    sot3.push_task(task_joint_vel);
  }
  // sot3.push_task(task_joint_pos);

  // sot1.new_level();
  // sot1.new_level();
  // sot1.push_task(task_tikhonov_regular);
  // sot2.new_level();
  // sot2.new_level();
  // sot2.push_task(task_tikhonov_regular);
  // sot3.new_level();
  // sot3.new_level();
  // sot3.push_task(task_tikhonov_regular);

  /********************
   ** EQ Constraints **
   ********************/
  elastoplastic::EqualitySet eq_set1(prb_dim), eq_set2(prb_dim),
      eq_set3(prb_dim);
  elastoplastic::EqualityConstraint eq_model(prb_dim, m_full_nax, "Model");

  eq_model.A() << M, -Eigen::MatrixXd::Identity(m_full_nax, m_full_nax),
      -J.transpose();
  eq_model.b() = C * qp_in + grav;
  eq_model.b().tail(m_nax) +=
      (Eigen::Map<Eigen::VectorXd>(m_parameters.joints_static_friction.data(),
                                   m_parameters.joints_static_friction.size())
           .array() *
       qp_in.tail(m_nax).array().sign())
          .matrix(); // Friction
  eq_model.b().tail(m_nax) +=
      Eigen::Map<Eigen::VectorXd>(m_parameters.joints_damping.data(),
                                  m_parameters.joints_damping.size())
          .cwiseProduct(qp_in.tail(m_nax)); // Damping
  eq_set1.push_constraint(eq_model);
  eq_set1.compute_set();

  eq_set2.push_constraint(eq_model);
  eq_set3.push_constraint(eq_model);

  /***********************
   ** DISEQ Constraints **
   ***********************/
  elastoplastic::InequalityConstraint ineq_qpp_max(prb_dim, m_full_nax,
                                                   "Joint Acceleration Max");
  elastoplastic::InequalityConstraint ineq_qpp_min(prb_dim, m_full_nax,
                                                   "Joint Acceleration Min");
  elastoplastic::InequalityConstraint ineq_qp_max(prb_dim, m_full_nax,
                                                  "Joint Velocity Max");
  elastoplastic::InequalityConstraint ineq_qp_min(prb_dim, m_full_nax,
                                                  "Joint Velocity Min");
  elastoplastic::InequalityConstraint ineq_q_max(prb_dim, m_nax,
                                                 "Joint Position Max");
  elastoplastic::InequalityConstraint ineq_q_min(prb_dim, m_nax,
                                                 "Joint Position Min");

  // Velocity
  ineq_qp_min.CI().leftCols(m_full_nax)
      << Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  ineq_qp_min.ci().segment(m_mobile_base->nax(), m_nax) =
      (m_qp_in.tail(m_nax) + m_limits.vel);
  ineq_qp_min.ci().segment(m_mobile_base->nax(), m_nax).cwiseMax(1e-9);

  ineq_qp_max.CI().leftCols(m_full_nax)
      << -Eigen::MatrixXd::Identity(m_full_nax, m_full_nax) * m_dt;
  ineq_qp_max.ci().segment(m_mobile_base->nax(), m_nax) =
      (m_limits.vel - m_qp_in.tail(m_nax));
  ineq_qp_max.ci().segment(m_mobile_base->nax(), m_nax).cwiseMax(1e-9);

  // Acceleration
  ineq_qpp_min.CI().leftCols(m_full_nax)
      << Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
  ineq_qpp_min.ci().segment(m_mobile_base->nax(), m_nax) = m_limits.acc;

  ineq_qpp_max.CI().leftCols(m_full_nax)
      << -Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
  ineq_qpp_max.ci().segment(m_mobile_base->nax(), m_nax) = m_limits.acc;

  // Positions
  ineq_q_min.CI().block(0, m_mobile_base->nax(), m_nax, m_nax)
      << (Eigen::MatrixXd::Identity(m_nax, m_nax) * 0.5 * m_dt * m_dt);
  ineq_q_min.ci().head(m_nax) =
      ((m_q_in.tail(m_nax) + m_qp_in.tail(m_nax) * m_dt) - m_limits.pos_lower);
  ineq_q_min.ci() = ineq_q_min.ci().cwiseMax(1e-9);

  ineq_q_max.CI().block(0, m_mobile_base->nax(), m_nax, m_nax)
      << (-Eigen::MatrixXd::Identity(m_nax, m_nax) * 0.5 * m_dt * m_dt);
  ineq_q_max.ci().head(m_nax) =
      (m_limits.pos_upper - (m_q_in.tail(m_nax) + m_qp_in.tail(m_nax) * m_dt));
  ineq_q_max.ci() = ineq_q_max.ci().cwiseMax(1e-9);

  // Move base limits to world
  if (m_mobile_base->enabled) {
    // Eigen::Vector6d max_vel_base_in_base =
    //     utils::twist_from_base_velocity(m_mobile_base->vel_limits);
    // Eigen::Vector6d max_vel_base_in_world =
    //     rdyn::spatialRotation(max_vel_base_in_base, m_T_world_base.linear());
    // Eigen::Vector3d max_vel =
    //     utils::base_velocity_from_twist(max_vel_base_in_world);
    ineq_qp_min.ci().head<M_SE2>()
        << m_qp_in.head<M_SE2>() + m_mobile_base->vel_limits;
    ineq_qp_min.ci().head<M_SE2>().cwiseMax(1e-9);
    ineq_qp_max.ci().head<M_SE2>()
        << m_mobile_base->vel_limits - m_qp_in.head<M_SE2>();
    ineq_qp_max.ci().head<M_SE2>().cwiseMax(1e-9);

    Eigen::Vector6d max_acc_base_in_base =
        utils::twist_from_base_velocity(m_mobile_base->acc_limits);
    Eigen::Vector6d max_acc_base_in_world =
        rdyn::spatialRotation(max_acc_base_in_base, m_T_world_base.linear());
    Eigen::Vector3d max_acc =
        utils::base_velocity_from_twist(max_acc_base_in_world);
    ineq_qpp_min.ci().head<M_SE2>() << max_acc;
    ineq_qpp_max.ci().head<M_SE2>() << max_acc;
    ineq_qpp_min.ci().head<M_SE2>().cwiseMax(1e-9);
    ineq_qpp_max.ci().head<M_SE2>().cwiseMax(1e-9);
  }

  elastoplastic::InequalityConstraint ineq_tau_max(prb_dim, m_full_nax);
  elastoplastic::InequalityConstraint ineq_tau_min(prb_dim, m_full_nax);
  elastoplastic::InequalityConstraint ineq_f_max(prb_dim, 6);
  elastoplastic::InequalityConstraint ineq_f_min(prb_dim, 6);

  ineq_tau_min.CI().middleCols(m_full_nax, m_full_nax)
      << Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
  ineq_tau_min.ci().segment(0, m_full_nax) =
      Eigen::VectorXd::Ones(m_full_nax) * 50;

  ineq_tau_max.CI().middleCols(m_full_nax, m_full_nax)
      << -Eigen::MatrixXd::Identity(m_full_nax, m_full_nax);
  ineq_tau_max.ci().segment(0, m_full_nax) =
      Eigen::VectorXd::Ones(m_full_nax) * 50;

  ineq_f_min.CI().rightCols<6>() << Eigen::Matrix6d::Identity();
  ineq_f_min.ci() = Eigen::Vector6d::Ones() * 50;

  ineq_f_max.CI().rightCols<6>() << -Eigen::Matrix6d::Identity();
  ineq_f_max.ci() = Eigen::Vector6d::Ones() * 50;

  elastoplastic::InequalitySet ineq_set1(prb_dim), ineq_set2(prb_dim),
      ineq_set3(prb_dim);
  ineq_set1.push_constraint(ineq_q_min);
  ineq_set1.push_constraint(ineq_q_max);
  ineq_set1.push_constraint(ineq_qp_min);
  ineq_set1.push_constraint(ineq_qp_max);
  ineq_set1.push_constraint(ineq_qpp_min);
  ineq_set1.push_constraint(ineq_qpp_max);
  ineq_set1.push_constraint(ineq_tau_min);
  ineq_set1.push_constraint(ineq_tau_max);
  ineq_set1.push_constraint(ineq_f_min);
  ineq_set1.push_constraint(ineq_f_max);
  ineq_set1.compute_set();

  ineq_set2.push_constraint(ineq_q_min);
  ineq_set2.push_constraint(ineq_q_max);
  ineq_set2.push_constraint(ineq_qp_min);
  ineq_set2.push_constraint(ineq_qp_max);
  ineq_set2.push_constraint(ineq_qpp_min);
  ineq_set2.push_constraint(ineq_qpp_max);
  ineq_set2.push_constraint(ineq_tau_min);
  ineq_set2.push_constraint(ineq_tau_max);
  ineq_set2.push_constraint(ineq_f_min);
  ineq_set2.push_constraint(ineq_f_max);
  ineq_set2.compute_set();

  ineq_set3.push_constraint(ineq_q_min);
  ineq_set3.push_constraint(ineq_q_max);
  ineq_set3.push_constraint(ineq_qp_min);
  ineq_set3.push_constraint(ineq_qp_max);
  ineq_set3.push_constraint(ineq_qpp_min);
  ineq_set3.push_constraint(ineq_qpp_max);
  ineq_set3.push_constraint(ineq_tau_min);
  ineq_set3.push_constraint(ineq_tau_max);
  ineq_set3.push_constraint(ineq_f_min);
  ineq_set3.push_constraint(ineq_f_max);
  ineq_set3.compute_set();

  /***********
   ** Solve **
   ***********/
  sot1.symmetrize();
  sot1.regularize(1e-12);
  sot2.symmetrize();
  sot2.regularize(1e-9);
  sot3.symmetrize();
  sot3.regularize(1e-9);

  // RCLCPP_WARN_STREAM(get_node()->get_logger(),
  //                    "sot.G\n"
  //                        << sot1.G() << "\nsot.F\n"
  //                        << sot1.F().transpose() << "\neq_set1.CE\n"
  //                        << eq_set1.CE() << "\neq_set1.ce\n"
  //                        << eq_set1.ce().transpose() << "\nineq_set1.CI\n"
  //                        << ineq_set1.CI() << "\nineq_set1.ci\n"
  //                        << ineq_set1.ci());
  //   assert(false);

  // =============== First level
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "ci -> " << ineq_set1.ci().transpose());
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "qp -> " << qp.transpose());
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "q -> " <<
  // m_q_in.transpose()); RCLCPP_INFO_STREAM(get_node()->get_logger(),
  //                    "limits_sup_q -> " << m_limits.pos_upper.transpose());
  elastoplastic::SolverQP solver1(prb_dim, sot1, eq_set1, ineq_set1);
  auto [sol1, status] = solver1.solve();

  std::optional<Eigen::VectorXd> return_val;
  Eigen::VectorXd solutionQP = sol1;

  if (status != SolverStatus::EIQUADPROG_FAST_OPTIMAL) {
    RCLCPP_ERROR(get_node()->get_logger(), "Problem unfeasible: ");
    switch (status) {
    case SolverStatus::EIQUADPROG_FAST_INFEASIBLE:
      RCLCPP_ERROR(get_node()->get_logger(), "EIQUADPROG_FAST_INFEASIBLE");
      break;
    case SolverStatus::EIQUADPROG_FAST_UNBOUNDED:
      RCLCPP_ERROR(get_node()->get_logger(), "EIQUADPROG_FAST_UNBOUNDED");
      RCLCPP_WARN_STREAM(get_node()->get_logger(),
                         "sot.G\n"
                             << sot1.G() << "\nsot.F\n"
                             << sot1.F().transpose() << "\neq_set1.CE\n"
                             << eq_set1.CE() << "\neq_set1.ce\n"
                             << eq_set1.ce().transpose() << "\nineq_set1.CI\n"
                             << ineq_set1.CI() << "\nineq_set1.ci\n"
                             << ineq_set1.ci());
      // assert(false);
      break;
    case SolverStatus::EIQUADPROG_FAST_REDUNDANT_EQUALITIES:
      RCLCPP_ERROR(get_node()->get_logger(),
                   "EIQUADPROG_FAST_REDUNDANT_EQUALITIES");
      break;
    case SolverStatus::EIQUADPROG_FAST_MAX_ITER_REACHED:
      RCLCPP_ERROR(get_node()->get_logger(),
                   "EIQUADPROG_FAST_MAX_ITER_REACHED");
      break;
    default:
      break;
    }
    return_val = std::nullopt;
    // return std::nullopt;
  } else if (sol1.hasNaN()) {
    RCLCPP_ERROR(get_node()->get_logger(), "NaN in the solution at level 1!");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "sot.G\n"
                            << sot1.G() << "\nsot.F\n"
                            << sot1.F().transpose() << "\neq_set1.CE\n"
                            << eq_set1.CE() << "\neq_set1.ce\n"
                            << eq_set1.ce().transpose() << "\nineq_set1.CI\n"
                            << ineq_set1.CI() << "\nineq_set1.ci\n"
                            << ineq_set1.ci());
    assert(false);
    return_val = std::nullopt;
    // return std::nullopt;
  } else {
    // RCLCPP_INFO_STREAM(
    //     get_node()->get_logger(),
    //     "\ntask_admittance:\n"
    //         << task_pseudo_admittance.value(solutionQP).transpose()
    //         << "\ntask_gravity:\n"
    //         << task_gravity.value(solutionQP).transpose());

    // solutionQP.setZero();
    // solutionQP.head(m_full_nax) = M.inverse() * (C * qp);
    return_val = solutionQP;
  }

  if (!return_val.has_value()) {
    Eigen::VectorXd v(prb_dim);
    v.head(m_full_nax) = -(m_qp_in / m_dt);
    v.head(m_full_nax)
        .tail(m_nax)
        .cwiseMax(-m_limits.acc)
        .cwiseMin(m_limits.acc); // clamp on saturation
    v.tail<6>().setZero();
    v.segment(m_full_nax, m_full_nax) =
        M * v.head(m_full_nax) + C * qp_in + grav;
    return v;
  }

  // =============== Second level
  elastoplastic::EqualityConstraint eq_prev_level_1(prb_dim, 6, "Prev level 1");
  if (is_force_active) {
    eq_prev_level_1.A() = task_pseudo_admittance.A();
  } else {
    eq_prev_level_1.A() = task_motion_tracking.A();
  }
  eq_prev_level_1.b() = -eq_prev_level_1.A() * solutionQP;
  eq_set2.push_constraint(eq_prev_level_1);
  eq_set2.compute_set();
  elastoplastic::SolverQP solver2(prb_dim, sot2, eq_set2, ineq_set2);
  auto [sol_tmp, stat_tmp] = solver2.solve();

  if (stat_tmp != SolverStatus::EIQUADPROG_FAST_OPTIMAL) {
    RCLCPP_ERROR(get_node()->get_logger(), "Problem 2 unfeasible");
    return solutionQP;
  } else if (sol_tmp.hasNaN()) {
    RCLCPP_ERROR(get_node()->get_logger(), "NaN in the solution at level 2 !");
    return solutionQP;
  }

  solutionQP = sol_tmp;
  status = stat_tmp;

  // =============== Third level
  size_t task_prev_level_2_size =
      is_force_active ? task_gravity.size() : task_pseudo_admittance.size();
  elastoplastic::EqualityConstraint eq_prev_level_21(eq_prev_level_1);
  elastoplastic::EqualityConstraint eq_prev_level_22(
      prb_dim, task_prev_level_2_size, "Prev level 2");
  if (is_force_active) {
    eq_prev_level_22.A() = task_gravity.A();
  } else {
    eq_prev_level_22.A() = task_pseudo_admittance.A();
  }
  eq_prev_level_21.A() = eq_prev_level_1.A();
  eq_prev_level_21.b() = -eq_prev_level_1.A() * sol1;
  eq_prev_level_22.b() = -eq_prev_level_22.A() * solutionQP;
  eq_set3.push_constraint(eq_prev_level_21);
  eq_set3.push_constraint(eq_prev_level_22);
  eq_set3.compute_set();
  elastoplastic::SolverQP solver3(prb_dim, sot3, eq_set3, ineq_set3);
  std::tie(sol_tmp, stat_tmp) = solver3.solve();

  if (stat_tmp != SolverStatus::EIQUADPROG_FAST_OPTIMAL) {
    RCLCPP_ERROR(get_node()->get_logger(), "Problem 3 unfeasible");
    return solutionQP;
  } else if (sol_tmp.hasNaN()) {
    RCLCPP_ERROR(get_node()->get_logger(), "NaN in the solution at level 3 !");
    return solutionQP;
  }

  RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "task_admittance:\n"
          << task_pseudo_admittance.value(solutionQP).transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "task_gravity:\n"
                         << task_gravity.value(solutionQP).transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "J^T f\n"
                         << (J.transpose() * solutionQP.tail<6>()).transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "model:\n"
                         << eq_model.value(solutionQP).transpose());

  return solutionQP;
}
} // namespace elastoplastic
