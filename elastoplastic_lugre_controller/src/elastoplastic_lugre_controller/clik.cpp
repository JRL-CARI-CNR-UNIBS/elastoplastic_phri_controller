#include "elastoplastic_lugre_controller/elastoplastic_lugre_controller.hpp"
#include "elastoplastic_lugre_controller/sot.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Eigenvalues/EigenSolver.h>
#include <algorithm>
#include <elastoplastic_lugre_controller/elastoplastic_variable_model.hpp>
#include <elastoplastic_lugre_controller/sot.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/liegroup/special-euclidean.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <algorithm>

namespace elastoplastic {

pin::Motion frame_distance(const pin::SE3& A, const pin::SE3& B)
{
  // B - A
  pin::SE3 R = pin::SE3(A.rotation(), Eigen::Vector3d::Zero());
  // error in reference coordinates
  pin::Motion e_ref = pin::log6(A.inverse() * B);
  return R.act(e_ref);
}

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

std::optional<Eigen::VectorXd>
ElastoplasticController::optimize(Eigen::Vector6d &wrench_tool_in_world) {

  Eigen::Matrix6Xd J = Eigen::MatrixXd::Zero(6, m_model.nv);
  pin::getFrameJacobian(m_model, m_model_data, m_tool_id,
                        pin::LOCAL_WORLD_ALIGNED, J);
  Eigen::Vector6d acc_non_linear_in_world = pin::getFrameClassicalAcceleration(m_model, m_model_data, m_tool_id, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED);

  auto t_start_QP = get_node()->get_clock()->now();
  Eigen::Vector6d& enabled_axis = m_impedance.enabled_axis;

  auto t_start_qp = get_node()->get_clock()->now();
  const unsigned int prb_dim = m_model.nv + M_SE3;

  Eigen::Vector6d twist_tool_world_in_world = pin::getFrameVelocity(m_model, m_model_data, m_tool_id, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED).toVector();

  elastoplastic::Task task_cart_vel(prb_dim, M_SE3,
                                    "Cartesian velocity tracking");
  elastoplastic::Task task_cart_pos(prb_dim, M_SE3,
                                    "Cartesian position tracking");
  elastoplastic::Task task_joint_pos(prb_dim, m_model.nq,
                                     "Joint position tracking");
  elastoplastic::Task task_joint_vel(prb_dim, m_model.nv,
                                     "Joint velocity tracking");
  elastoplastic::Task task_minimize_cart_acc(prb_dim, M_SE3,
                                             "Cartesian minimize acceleration");
  elastoplastic::Task task_minimize_joint_acc(prb_dim, m_model.nv,
                                              "Joint minimize acceleration");
  elastoplastic::Task task_admittance(prb_dim, M_SE3, "Admittance");
  elastoplastic::Task task_minimize_jerk(prb_dim, M_SE3,
                                         "Cartesian minimize jerk");

  /**********************
   ** Task Definitions **
   **********************/

  // Task Cartesian : Minimize cartesian distance from reference twist
  task_cart_vel.A().middleCols<M_SE3>(m_model.nv) =
      Eigen::Matrix6d::Identity() * m_dt;
  task_cart_vel.b() = (m_computed_target_twist_tool_world_in_world -
                       m_reference_target_twist_tool_world_in_world);
  normalize(task_cart_vel);

  // Task Cartesian : Minimize difference between the real target and the
  // computed one
  pin::SE3 oMref(m_reference_target_T_world_tool.rotation(),
                m_reference_target_T_world_tool.translation());
  const pin::SE3 & oMtool = m_model_data.oMf[m_tool_id];
  Eigen::Vector6d ref_p_err = frame_distance(oMref, oMtool).toVector();

  task_cart_pos.A().middleCols<M_SE3>(m_model.nv) =
      Eigen::Matrix6d::Identity() * 0.5 * std::pow(m_dt, 2);
  task_cart_pos.b() << ref_p_err +
                           m_computed_target_twist_tool_world_in_world * m_dt;
  normalize(task_cart_pos);

  // Task Cartesian:
  elastoplastic::Task task_minimize_cart_vel(prb_dim, M_SE3);
  task_minimize_cart_vel.A().middleCols<M_SE3>(m_model.nv) =
      Eigen::Matrix6d::Identity() * m_dt;
  task_minimize_cart_vel.b() = twist_tool_world_in_world;
  normalize(task_minimize_cart_vel);

  task_minimize_cart_acc.A().middleCols<M_SE3>(m_model.nv).setIdentity();
  task_minimize_cart_acc.b().setZero();
  normalize(task_minimize_cart_acc);

  task_minimize_jerk.A().middleCols<M_SE3>(m_model.nv) =
      Eigen::Matrix6d::Identity() / m_dt;
  task_minimize_jerk.b() = -m_computed_target_acc_tool_world_in_world / m_dt;
  normalize(task_minimize_jerk);

  // Task: Admittance
  Eigen::Affine3d world_M_tool = Eigen::Affine3d(m_model_data.oMf[m_tool_id].toHomogeneousMatrix());
  Eigen::Matrix6d K = m_elastoplastic_model->compute_variable_matrices(world_M_tool);
  Eigen::Matrix6d &D = m_impedance.D;
  Eigen::Matrix6d &invM = m_impedance.invM;
  Eigen::Vector6d twist_error_tool_world_in_world =
      twist_tool_world_in_world -
      m_computed_target_twist_tool_world_in_world;
  Eigen::Vector6d pose_error_tool_world_in_world;
  pose_error_tool_world_in_world = frame_distance(pin::SE3(m_computed_target_T_world_tool.matrix()), oMtool).toVector();

  Eigen::Matrix6d adm = Eigen::Matrix6d::Identity() + invM * D * m_dt +
                        0.5 * invM * K * std::pow(m_dt, 2);
  task_admittance.A() << adm * J, -adm;
  task_admittance.b() << adm * acc_non_linear_in_world +
                             invM * D * twist_error_tool_world_in_world +
                             invM * K *
                                 (twist_error_tool_world_in_world * m_dt +
                                  m_elastoplastic_model->z() +
                                  pose_error_tool_world_in_world.cwiseProduct(
                                      Eigen::Vector6d::Ones() - enabled_axis)) -
                             invM * (wrench_tool_in_world);
  normalize(task_admittance);

  // Weighting matrix
  m_W.setIdentity();
  if (m_mobile_base->enabled) {
    auto logis = 1;
    // auto logis = m_logistic.get(m_velocity_base_in_base);
    // m_logis_prec = logis;
    m_W.diagonal()(2) *= 1e6;
    m_W.diagonal().head<2>() *= 1e3;
    double weight_coeff = (1.0 + m_parameters.clik.alpha_gain *
                                     m_elastoplastic_model->alpha() * logis);
    m_W.diagonal().head<2>() /= weight_coeff;
  }

  // Task: Minimize joint acceleration and weighting
  task_minimize_joint_acc.A().leftCols(m_model.nv).setIdentity();
  task_minimize_joint_acc.b().setZero();
  normalize(task_minimize_joint_acc);
  // task_minimize_joint_acc.W() *= m_W.transpose() * m_W;

  // Task: Joint Velocity
  task_joint_vel.A().leftCols(m_model.nv) +=
      -Eigen::MatrixXd::Identity(m_model.nv, m_model.nv) * m_dt;
  task_joint_vel.b() += (m_full_velocity_references - m_qp);
  normalize(task_joint_vel);
  task_joint_vel.W() *= m_W.transpose() * m_W;

  // Task: Joint Position
  task_joint_pos.A().leftCols(m_model.nv) +=
      -0.5 * Eigen::MatrixXd::Identity(m_model.nv, m_model.nv) *
      std::pow(m_dt, 2);
  task_joint_pos.b() = pin::difference(m_model, pin::integrate(m_model, m_q, m_qp * m_dt), m_full_position_references);
  normalize(task_joint_pos);
  task_joint_pos.W() *= m_W.transpose() * m_W;

  elastoplastic::Task task_minimize_base_acc(prb_dim, 3);
  task_minimize_base_acc.A().leftCols<3>().setIdentity();
  normalize(task_minimize_base_acc);

  /****************
   ** Task Stack **
   ****************/
  constexpr double STACK_LEVEL_STEP = 1e-3;
  constexpr int STACK_LEVEL_ZERO = -1;
  elastoplastic::Stack sot(prb_dim, STACK_LEVEL_STEP, STACK_LEVEL_ZERO);

  /* Variable stack */
  constexpr int CART_POS_LEVEL_OFFSET = 2;
  int cart_pos_level = STACK_LEVEL_ZERO + CART_POS_LEVEL_OFFSET;
  if (!m_elastoplastic_model->is_plastic() &&
      m_elastoplastic_model->to_restore() &&
      m_parameters.impedance.plastic_restoration) {
    RCLCPP_DEBUG_STREAM_THROTTLE(get_node()->get_logger(),
                                 *get_node()->get_clock(), 1, "Is restoring");
    cart_pos_level = STACK_LEVEL_ZERO + 1;
    m_W.setIdentity();
    task_joint_pos.W() *= m_W.transpose() * m_W;
    task_joint_vel.W() *= m_W.transpose() * m_W;
    task_minimize_joint_acc.W() *= m_W.transpose() * m_W;
  } else {
  }
  sot.insert_task(task_cart_pos, cart_pos_level, 1);

  /* Constant stack */
  sot.push_task(task_cart_vel, 4);
  sot.new_level();
  sot.push_task(task_minimize_cart_acc);
  sot.new_level();
  sot.push_task(task_admittance);
  sot.new_level();
  sot.push_task(task_minimize_base_acc, m_parameters.clik.joint_task.kv);
  sot.push_task(task_joint_vel, m_parameters.clik.joint_task.kv);
  sot.push_task(task_joint_pos, m_parameters.clik.joint_task.kp);
  sot.new_level();
  sot.push_task(task_minimize_jerk);
  sot.push_task(task_minimize_joint_acc);

  /********************
   ** EQ Constraints **
   ********************/
  elastoplastic::EqualitySet eq_set(prb_dim);

  eq_set.compute_set();

  /***********************
   ** DISEQ Constraints **
   ***********************/
  elastoplastic::InequalityConstraint ineq_qpp_max(prb_dim, m_model.nv,
                                                   "Joint Acceleration Max");
  elastoplastic::InequalityConstraint ineq_qpp_min(prb_dim, m_model.nv,
                                                   "Joint Acceleration Min");
  elastoplastic::InequalityConstraint ineq_qp_max(prb_dim, m_model.nv,
                                                  "Joint Velocity Max");
  elastoplastic::InequalityConstraint ineq_qp_min(prb_dim, m_model.nv,
                                                  "Joint Velocity Min");
  elastoplastic::InequalityConstraint ineq_q_max(prb_dim, m_arm_nax,
                                                 "Joint Position Max");
  elastoplastic::InequalityConstraint ineq_q_min(prb_dim, m_arm_nax,
                                                 "Joint Position Min");
  elastoplastic::InequalityConstraint ineq_xpp_max(
      prb_dim, M_SE3, "Cartesian Acceleration Max");
  elastoplastic::InequalityConstraint ineq_xpp_min(
      prb_dim, M_SE3, "Cartesian Acceleration Min");

  // Velocity
  ineq_qp_min.CI().leftCols(m_model.nv)
      << Eigen::MatrixXd::Identity(m_model.nv, m_model.nv) * m_dt;
  ineq_qp_min.ci().segment(m_mobile_base->nax(), m_arm_nax) =
      (m_qp.tail(m_arm_nax) + m_limits.vel);

  ineq_qp_max.CI().leftCols(m_model.nv)
      << -Eigen::MatrixXd::Identity(m_model.nv, m_model.nv) * m_dt;
  ineq_qp_max.ci().segment(m_mobile_base->nax(), m_arm_nax) =
      (m_limits.vel - m_qp.tail(m_arm_nax));

  // Acceleration
  ineq_qpp_min.CI().leftCols(m_model.nv)
      << Eigen::MatrixXd::Identity(m_model.nv, m_model.nv);
  ineq_qpp_min.ci().segment(m_mobile_base->nax(), m_arm_nax) = m_limits.acc;

  ineq_qpp_max.CI().leftCols(m_model.nv)
      << -Eigen::MatrixXd::Identity(m_model.nv, m_model.nv);
  ineq_qpp_max.ci().segment(m_mobile_base->nax(), m_arm_nax) = m_limits.acc;

  // Positions
  ineq_q_min.CI().block(0, m_mobile_base->nax(), m_arm_nax, m_arm_nax)
      << Eigen::MatrixXd::Identity(m_arm_nax, m_arm_nax) * 0.5 * m_dt * m_dt;
  ineq_q_min.ci().head(m_arm_nax) =
      (m_q.tail(m_arm_nax) + m_qp.tail(m_arm_nax) * m_dt) - m_limits.pos_lower;

  ineq_q_max.CI().block(0, m_mobile_base->nax(), m_arm_nax, m_arm_nax)
      << -Eigen::MatrixXd::Identity(m_arm_nax, m_arm_nax) * 0.5 * m_dt * m_dt;
  ineq_q_max.ci().head(m_arm_nax) =
      m_limits.pos_upper - (m_q.tail(m_arm_nax) + m_qp.tail(m_arm_nax) * m_dt);

  // Acceleration
  ineq_xpp_min.CI().middleCols<M_SE3>(m_model.nv) = Eigen::Matrix6d::Identity();
  ineq_xpp_min.ci() = Eigen::VectorXd::Constant(6, 10);

  ineq_xpp_max.CI().middleCols<M_SE3>(m_model.nv) =
      -Eigen::Matrix6d::Identity();
  ineq_xpp_max.ci() = Eigen::VectorXd::Constant(6, 10);

  // Move base limits to world
  if (m_mobile_base->enabled) {
    pin::SE3 world_R_base(m_world_M_base.rotation(), Eigen::Vector3d::Zero());
    Eigen::Vector6d max_vel_base_in_base =
        utils::twist_from_base_velocity(m_mobile_base->vel_limits);
    Eigen::Vector6d max_vel_base_in_world =
        world_R_base.act(pin::Motion(max_vel_base_in_base)).toVector();
    Eigen::Vector3d max_vel =
        utils::base_velocity_from_twist(max_vel_base_in_world);
    ineq_qp_min.ci().head<M_SE2>() << m_qp_in.head<M_SE2>() + max_vel;
    ineq_qp_min.ci().head<M_SE2>().cwiseMax(1e-9);
    ineq_qp_max.ci().head<M_SE2>() << max_vel - m_qp_in.head<M_SE2>();
    ineq_qp_max.ci().head<M_SE2>().cwiseMax(1e-9);

    Eigen::Vector6d max_acc_base_in_base =
        utils::twist_from_base_velocity(m_mobile_base->acc_limits);
    Eigen::Vector6d max_acc_base_in_world =
        world_R_base.act(pin::Motion(max_acc_base_in_base)).toVector();
    Eigen::Vector3d max_acc =
        utils::base_velocity_from_twist(max_acc_base_in_world);
    ineq_qpp_min.ci().head<M_SE2>() << max_acc;
    ineq_qpp_max.ci().head<M_SE2>() << max_acc;
    ineq_qpp_min.ci().head<M_SE2>().cwiseMax(1e-9);
    ineq_qpp_max.ci().head<M_SE2>().cwiseMax(1e-9);
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
    LOG_ERROR_THROTTLE_COUNT(
        get_node()->get_logger(), get_node()->get_clock(), 1.0,
        "Problem unfeasible. Solver status: "
            << status << ". Is G Positive Definite: "
            << (chol.info() == Eigen::ComputationInfo::Success));
    return std::nullopt;
  }

  if (solutionQP.hasNaN()) {
    RCLCPP_ERROR(get_node()->get_logger(), "NaN in the solution!");
    RCLCPP_DEBUG_STREAM(get_node()->get_logger(),
                        "Dump: "
                            << "\n## first round sol [qpp(" << m_model.nv
                            << "), slack(" << prb_dim - m_model.nv << ")]##\n"
                            << solutionQP.transpose()
                            << "\n## first round ret ##\n"
                            << status << "## G ## " << sot.G() << "\n## F ##"
                            << sot.F().transpose() << "\n## eq_set.CE() ## "
                            << eq_set.CE() << "\n ## eq_set.ce() ## "
                            << eq_set.ce().transpose() << "\n## CI ## "
                            << ineq_set.CI() << "\n## ci ##"
                            << ineq_set.ci().transpose());
    return std::nullopt;
  }

  // Should be useless but...
  constexpr double VIOLATION_TOLL = 1e-3;
  if (ineq_set.violations(solutionQP, VIOLATION_TOLL) != 0) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Constraint violated:");
    auto ineq_violated = ineq_set.which_violations(solutionQP, VIOLATION_TOLL);
    std::for_each(ineq_violated.begin(), ineq_violated.end(),
                  [this, &solutionQP](const InequalityConstraint &ineq) {
                    RCLCPP_ERROR_STREAM(
                        get_node()->get_logger(),
                        " - " << ineq.description() << " | values: "
                              << ineq.value(solutionQP).transpose());
                  });
    return std::nullopt;
  }

  m_admittance_value =
      task_admittance.value(solutionQP) + invM * (wrench_tool_in_world);
  return solutionQP;
}
} // namespace elastoplastic
