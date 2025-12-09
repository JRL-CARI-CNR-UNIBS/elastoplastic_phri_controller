#include "elastoplastic_lugre_controller/interpolation/interpolator.hpp"
#include "rdyn_core/spacevect_algebra.h"
#include <cmath>

#define DEBUG
namespace elastoplastic::utils::interpolation {

Interpolator::Interpolator(const Trajectory &plan) { this->set_plan(plan); }

Interpolator::Interpolator() : m_state(State::Empty) {}

void Interpolator::set_plan(const Trajectory &plan) {
  m_plan = plan;
  m_coeff_rot_spline = prepare_lie_spline(plan.pose, Eigen::Vector3d::Zero(),
                                          Eigen::Vector3d::Zero());
  m_state = State::Available;
}

Interpolator
Interpolator::from_msg(const moveit_msgs::msg::CartesianTrajectory &trj) {
  Trajectory plan;
  size_t size = trj.points.size();
  plan.clear();
  plan.resize(size);
  for (size_t idx = 0; idx < size; ++idx) {
    tf2::fromMsg(trj.points.at(idx).point.pose, plan.pose.at(idx));
    tf2::fromMsg(trj.points.at(idx).point.velocity, plan.twist.at(idx));
    //    tf2::fromMsg(trj.points.at(idx).point.acceleration,
    //    m_plan.acc.at(idx));
    plan.time.at(idx) =
        rclcpp::Time(trj.points.at(idx).time_from_start.sec,
                     trj.points.at(idx).time_from_start.nanosec);
  }
  std::cerr << "New interpolator created from msg, with " << size << "points."
            << std::endl;

  Interpolator inter(plan);
  return inter;
}

Interpolator
Interpolator::from_msg(const moveit_msgs::msg::CartesianTrajectory &trj,
                       const Eigen::Affine3d &start_pose) {
  Trajectory plan;
  size_t size = trj.points.size();
  plan.clear();
  plan.resize(size);
  for (size_t idx = 0; idx < size; ++idx) {
    // tf2::fromMsg(trj.points.at(idx).point.pose, plan.pose.at(idx));
    tf2::fromMsg(trj.points.at(idx).point.velocity, plan.twist.at(idx));
    //    tf2::fromMsg(trj.points.at(idx).point.acceleration,
    //    m_plan.acc.at(idx));
    plan.time.at(idx) =
        rclcpp::Time(trj.points.at(idx).time_from_start.sec,
                     trj.points.at(idx).time_from_start.nanosec);
    if (idx == 0) {
      plan.pose.at(0) = start_pose;
    } else {
      plan.pose.at(idx).translation() =
          plan.pose.at(idx - 1).translation() +
          (plan.time.at(idx) - plan.time.at(idx - 1)).seconds() *
              plan.twist.at(idx - 1).head<3>();
      plan.pose.at(idx).linear() = plan.pose.at(0).linear();
    }
  }
  std::cerr << "New interpolator created from msg, with " << size << "points."
            << std::endl;

  Interpolator inter(plan);
  return inter;
}

void Interpolator::start_plan(const rclcpp::Time &time) {
  m_plan.start = time;
  m_state = State::Started;
}

Interpolator Interpolator::clone_with_transform(
    const geometry_msgs::msg::TransformStamped &tf) {
  Trajectory new_plan;
  new_plan.resize(m_plan.size());
  for (size_t idx = 0; idx < new_plan.size(); ++idx) {
    //    tf2::doTransform(m_plan.pose.at(idx), new_plan.pose.at(idx),t_tf);
    Eigen::Isometry3d tf_eigen = tf2::transformToEigen(tf);
    new_plan.pose.at(idx) = m_plan.pose.at(idx) * tf_eigen;
    // map -> object * object -> grasp
    new_plan.twist.at(idx) =
        rdyn::spatialTranslation(m_plan.twist.at(idx), tf_eigen.translation());
    new_plan.time.at(idx) = m_plan.time.at(idx);
  }
  return Interpolator(new_plan);
}

// ----------------------------------------------------------------------------
// Cubic interpolation: returns θ(s) = as + bs² + cs³
// with a,b,c from eqs. (7a)–(7c), using Θ = log(CsᵀCf), Θ̇ = A(Θ) ωf
Eigen::Vector3d interpolateRotationVector(const Eigen::Matrix3d &Cs,
                                          const Eigen::Matrix3d &Cf,
                                          const Eigen::Vector3d &ws,
                                          const Eigen::Vector3d &wf, double s,
                                          double T) {
  // total rotation vector Θ = log(Csᵀ Cf)           (5d, 7b–c)
  Eigen::Vector3d theta = so3Log(Cs.transpose() * Cf);

  // Θ̇ = A(Θ) wf                                  (5e)
  Eigen::Vector3d theta_dot = left_jacobian(theta) * wf;

  // cubic coefficients                            (7a–7c)
  Eigen::Vector3d a = ws;
  Eigen::Vector3d b = (3.0 * theta - 2.0 * T * ws - T * theta_dot) / (T * T);
  Eigen::Vector3d c = (-2.0 * theta + T * ws + T * theta_dot) / (T * T * T);

  // evaluate θ(s)
  return a * s + b * s * s + c * s * s * s;
}

Interpolator::InterpolationResult
Interpolator::interpolate(const rclcpp::Time &now, Eigen::Vector6d &acc,
                          Eigen::Vector6d &twist, Eigen::Affine3d &pose) {
  if (m_state == State::Empty || m_state == State::Available) {
    std::cerr << "[Interpolator]: Interpolation not started";
    return InterpolationResult::InterpolatorNotStarted;
  }
  if (m_state == State::Terminated) {
    std::cerr << "[Interpolator]: Plan terminated";
    return InterpolationResult::InterpolatorNotStarted;
  }
  const rclcpp::Duration T = now - m_plan.start;
  const rclcpp::Time t_from_start = rclcpp::Time(T.nanoseconds());
  size_t idx;
  if (t_from_start > m_plan.time.back()) {
    idx = m_plan.time.size() - 1;
    pose = m_plan.pose.back();
    twist.setZero();
    return InterpolationResult::OK;
  } else {
    idx = std::distance(m_plan.time.begin(),
                        std::find_if(m_plan.time.begin(), m_plan.time.end(),
                                     [t_from_start](const auto &time) {
                                       return t_from_start < time;
                                     }));
    idx = idx == 0 ? 1 : idx;
  }
  if (idx >= m_plan.time.size()) {
    std::cerr << "[Interpolator]: Interpolation ended" << std::endl;
    pose = m_plan.pose.back();
    twist.setZero();
    return InterpolationResult::OK;
  }

  // TODO: Cambia tutti i tempi in nanoseconds (così da essere interi e non
  // float)
  const double delta_time =
      (m_plan.time.at(idx) - m_plan.time.at(idx - 1)).seconds();
  const double t = (t_from_start - m_plan.time.at(idx - 1)).seconds();
  const double s = t / delta_time;

  Eigen::Isometry3d pose_interp;
  const Eigen::Vector3d p0 = m_plan.pose.at(idx - 1).translation();
  const Eigen::Vector3d p1 = m_plan.pose.at(idx).translation();
  const Eigen::Vector3d v0 = m_plan.twist.at(idx - 1).head<3>();
  const Eigen::Vector3d v1 = m_plan.twist.at(idx).head<3>();

  //  double s0=0.0;
  //  double s1=1.0;
  //  double ds = 0.00; //formula vel
  //  double s = 0.0; // formula pos;

  acc.head<3>() =
      6 * (2 * p0 + delta_time * v0 - 2 * p1 + delta_time * v1) * s /
          std::pow(delta_time, 2) +
      2 * (-3 * p0 + 3 * p1 - 2 * delta_time * v0 - delta_time * v1) /
          std::pow(delta_time, 2);
  twist.head<3>() =
      (3 * (2 * p0 + delta_time * v0 - 2 * p1 + delta_time * v1) *
           std::pow(s, 2) / delta_time +
       2 * (-3 * p0 + 3 * p1 - 2 * delta_time * v0 - delta_time * v1) * s /
           delta_time +
       v0);
  pose.translation() =
      (2 * p0 + delta_time * v0 - 2 * p1 + delta_time * v1) * std::pow(s, 3) +
      (-3 * p0 + 3 * p1 - 2 * delta_time * v0 - delta_time * v1) *
          std::pow(s, 2) +
      delta_time * v0 * s + p0;

  const Eigen::Quaterniond r0 =
      Eigen::Quaterniond(m_plan.pose.at(idx - 1).linear());
  const Eigen::Quaterniond r1 =
      Eigen::Quaterniond(m_plan.pose.at(idx).linear());
  const Eigen::Vector3d vr0 = m_plan.twist.at(idx - 1).tail<3>();
  const Eigen::Vector3d vr1 = m_plan.twist.at(idx).tail<3>();

  auto rotateVector = [](const Eigen::Quaterniond &q,
                         const Eigen::Vector3d &v) {
    // Eigen::Quaterniond p(0., v.x(), v.y(), v.z());
    // return (q * p * q.conjugate()).vec();
    return q.conjugate() * v;
  };

  Eigen::Vector3d w0 = 0.5 * delta_time * rotateVector(r0, vr0);
  Eigen::Vector3d w1 = 0.5 * delta_time * rotateVector(r1, vr1);

  // ensure shortest‐path quaternion
  Eigen::Quaterniond q1m = r1;
  if (r0.dot(q1m) < 0.0)
    q1m.coeffs() *= -1.0;

  // “delta” in the Lie algebra
  Eigen::Vector3d delta = quat_log(r0.conjugate() * q1m);

  // Hermite coefficients
  Eigen::Vector3d C = w0;
  Eigen::Vector3d B = 3.0 * delta - 2.0 * w0 - w1;
  Eigen::Vector3d A = -2.0 * delta + w0 + w1;

  Eigen::Vector3d X = ((A * s + B) * s + C) * s;
  Eigen::Vector3d Xp = ((3.0 * A * s + 2.0 * B) * s + C);

  // map back to S3 and prepend q0
  pose.linear() = (r0 * quat_exp(X)).normalized().toRotationMatrix();
  twist.tail<3>() = r0.toRotationMatrix() * left_jacobian(X) * Xp;

  /* Hermitian Cubic Spline */
  // double w1 = 3 * s * s - 2 * s * s * s;
  // double w2 = s * s * s - 2 * s * s + s;
  // double w3 = s * s * s - s * s;

  // double q1 = (6 * s - 6 * s * s) / delta_time;
  // double q2 = (3 * s * s - 4 * s + 1) / delta_time;
  // double q3 = (3 * s * s - 2 * s) / delta_time;

  // double m1 = (6 - 12 * s) / std::pow(delta_time, 2);
  // double m2 = (6 * s - 4) / std::pow(delta_time, 2);
  // double m3 = (6 * s - 2) / std::pow(delta_time, 2);

  // Eigen::AngleAxisd r1_sub_r0 = Eigen::AngleAxisd(r1.inverse() * r0);
  // Eigen::Vector3d wvec = w1 * r1_sub_r0.angle() * r1_sub_r0.axis() + w2 * vr0
  // + w3 * vr1; pose.linear() = Eigen::AngleAxisd(wvec.norm(),
  // wvec.normalized()).toRotationMatrix() * r0; if (v1 == v0) {
  //   twist.tail<3>() = v0;
  //   acc.tail<3>().setZero();
  // } else {
  //   twist.tail<3>() = q1 * r1_sub_r0.angle() * r1_sub_r0.axis() + q2 * vr0 +
  //   q3 * vr1; acc.tail<3>() = m1 * r1_sub_r0.angle() * r1_sub_r0.axis() + m2
  //   * vr0 + m3 * vr1;
  // }

  // Eigen::Vector3d pinterp =
  // interpolateRotationVector(m_plan.pose.at(idx - 1).linear(),
  // m_plan.pose.at(idx).linear(), m_plan.twist.at(idx - 1).tail<3>(),
  // m_plan.twist.at(idx).tail<3>(), s, delta_time); pose.linear() =
  // m_plan.pose.at(idx - 1).linear() * Eigen::AngleAxisd(pinterp.norm(),
  // pinterp.normalized()).toRotationMatrix(); twist.tail<3>() = pinterp /
  // delta_time; acc.tail<3>().setZero();

  /* Lie Spline ?? */

  // std::array<Eigen::Vector3d,3> coeff;
  // coeff[0] = m_coeff_rot_spline.a.at(idx);
  // coeff[1] = m_coeff_rot_spline.b.at(idx);
  // coeff[2] = m_coeff_rot_spline.c.at(idx);
  // const auto [mat, vec] = lie_spline(s, delta_time, coeff,
  // m_plan.pose.at(idx-1).linear()); o_pose.linear() = mat; o_twist.tail<3>() =
  // vec;

  /*******************************************************************************************************
   * Interactive Control of Interpolations for Animation and Modeling - Gabriel
   *Hanotaux, Bemard Peroche *
   *******************************************************************************************************/

  //  Eigen::Quaterniond qi,qim1,qip1,qip2;

  //  Eigen::Quaterniond qsi;
  //  Eigen::Quaterniond qsj;

  //  Eigen::Vector3d qp0, qv0, qp1, qv1;
  //  Eigen::AngleAxisd qri_squared, qli_squared;
  //  Eigen::Matrix<double,12,1> hermite_p;
  //  Eigen::Matrix4d hermite; Eigen::Matrix<double,3,4> dhermite;
  //  hermite << 2,delta_time,-2,delta_time,
  //             -3,-2*delta_time,3,-delta_time,
  //             0,delta_time,0,0,
  //             1,0,0,0;
  //  dhermite << 6/delta_time,3,-2/delta_time,1,
  //              -6/delta_time,-4,6/delta_time,-2,
  //              0,1,0,0;
  //  const Eigen::Vector4d s_vec({s*s*s, s*s, s, 1});
  //  if(idx == m_plan.twist.size()-1)
  //  {
  //    qi =   Eigen::Quaterniond(m_plan.pose.at(idx-1).linear());
  //    qip1 = Eigen::Quaterniond(m_plan.pose.at(idx).linear());
  //    qim1 = Eigen::Quaterniond(m_plan.pose.at(idx-2).linear());

  //    qli_squared = (qip1.inverse()*qi);
  //    qri_squared = (qim1.inverse()*qip1);
  //  }
  //  else if(idx == 1)
  //  {
  //    qip1 =   Eigen::Quaterniond(m_plan.pose.at(idx).linear());
  //    qip2 = Eigen::Quaterniond(m_plan.pose.at(idx+1).linear());
  //    qi = Eigen::Quaterniond(m_plan.pose.at(idx-1).linear());

  //    qli_squared = (qip2.inverse()*qi);
  //    qri_squared = (qi.inverse()*qip1);
  //  }
  //  else
  //  {
  //    qi =   Eigen::Quaterniond(m_plan.pose.at(idx-1).linear());
  //    qip1 = Eigen::Quaterniond(m_plan.pose.at(idx).linear());
  //    qim1 = Eigen::Quaterniond(m_plan.pose.at(idx-2).linear());
  //    qip2 = Eigen::Quaterniond(m_plan.pose.at(idx+1).linear());

  //    qli_squared = (qip2.inverse()*qi);
  //    qri_squared = (qim1.inverse()*qip1);
  //  }
  //  hermite_p.segment<3>(0) =
  //  quat_log(Eigen::Quaterniond(m_plan.pose.at(idx-1).linear()));
  //  hermite_p.segment<3>(3) = 0.5 * qri_squared.axis() *
  //  qri_squared.angle()/2; hermite_p.segment<3>(6) =
  //  quat_log(Eigen::Quaterniond(m_plan.pose.at(idx).linear()));
  //  hermite_p.segment<3>(9) = 0.5 * qli_squared.axis() *
  //  qli_squared.angle()/2; const Eigen::Quaterniond q_s =
  //  quat_exp(Eigen::Vector3d(
  //                             s_vec.transpose()*hermite*hermite_p({0,3,6,9})
  //                             ,
  //                             s_vec.transpose()*hermite*hermite_p({1,4,7,10}),
  //                             s_vec.transpose()*hermite*hermite_p({2,5,8,11}))
  //                             );
  //  const Eigen::Quaterniond q_ds = quat_exp(Eigen::Vector3d(
  //                             s_vec.tail<3>().transpose()*dhermite*hermite_p({0,3,6,9})
  //                             ,
  //                             s_vec.tail<3>().transpose()*dhermite*hermite_p({1,4,7,10}),
  //                             s_vec.tail<3>().transpose()*dhermite*hermite_p({2,5,8,11}))
  //                             );
  //  o_pose.linear() = q_s.toRotationMatrix();
  //  o_twist.tail<3>() = 2*(q_s.inverse() * q_ds).vec();

  /*********
   * SQUAD *
   *********/

  // Eigen::Quaterniond qi, qip1, qim1, qj, qjp1, qjm1;

  // Eigen::Quaterniond qsi;
  // Eigen::Quaterniond qsj;
  // if (idx == m_plan.twist.size() - 2) {
  //   qi = Eigen::Quaterniond(m_plan.pose.at(idx - 1).linear());
  //   qip1 = Eigen::Quaterniond(m_plan.pose.at(idx).linear());
  //   qim1 = Eigen::Quaterniond(m_plan.pose.at(idx - 2).linear());

  //   qsi = qi * quat_exp(-(quat_log(qi.inverse() * qip1) +
  //   quat_log(qi.inverse() * qim1)) / 4.0); qsj =
  //   Eigen::Quaterniond(m_plan.pose.back().linear());
  // } else if (idx == 1) {
  //   qj = Eigen::Quaterniond(m_plan.pose.at(idx).linear());
  //   qjp1 = Eigen::Quaterniond(m_plan.pose.at(idx + 1).linear());
  //   qjm1 = Eigen::Quaterniond(m_plan.pose.at(idx - 1).linear());

  //   qsi = Eigen::Quaterniond(m_plan.pose.front().linear());
  //   qsj = qj * quat_exp(-(quat_log(qj.inverse() * qjp1) +
  //   quat_log(qj.inverse() * qjm1)) / 4.0);
  // } else {
  //   qi = Eigen::Quaterniond(m_plan.pose.at(idx - 1).linear());
  //   qip1 = Eigen::Quaterniond(m_plan.pose.at(idx).linear());
  //   qim1 = Eigen::Quaterniond(m_plan.pose.at(idx - 2).linear());
  //   qj = qip1;
  //   qjp1 = Eigen::Quaterniond(m_plan.pose.at(idx + 1).linear());
  //   qjm1 = qi;

  //   qsi = qi * quat_exp(-(quat_log(qi.inverse() * qip1) +
  //   quat_log(qi.inverse() * qim1)) / 4.0); qsj = qj *
  //   quat_exp(-(quat_log(qj.inverse() * qjp1) + quat_log(qj.inverse() * qjm1))
  //   / 4.0);
  // }
  // const Eigen::Quaterniond qr = (qi.slerp(s, qip1)).slerp(2 * s * (1 - s),
  // qsi.slerp(s, qsj)); o_twist.tail<3>() = 2 /
  // (dt)*quat_log(Eigen::Quaterniond(t_q_prev.linear()).inverse() *
  // qr.normalized()); o_pose.linear() = qr.toRotationMatrix();

  /*******************************************************
   * Smooth Attitude Interpolation, SciPy, March 5, 2019 *
   *******************************************************/
  // const Eigen::Vector3d rv0 = m_plan.twist.at(idx - 1).tail<3>();
  // const Eigen::Vector3d rv1 = m_plan.twist.at(idx).tail<3>();
  // if ((rv1 - rv0).norm() < 1e-9) {
  //    // if rotation is too small, set to zero
  // o_pose.linear() = Eigen::Matrix3d::Identity();
  // o_twist.tail<3>() = Eigen::Vector3d::Zero();
  // return InterpolationResult::OK;
  // }
  // Eigen::AngleAxisd htheta;
  // htheta = (m_plan.pose.at(idx - 1).linear().transpose()) *
  // m_plan.pose.at(idx).linear(); const Eigen::Vector3d diff_rotvec =
  // htheta.axis() * htheta.angle() / 2; const Eigen::Vector3d d_diff_rotvec =
  // left_jacobian(diff_rotvec) * rv1; const double& Tf =
  // m_plan.time.at(idx).seconds(); const Eigen::Vector3d a = rv0; const
  // Eigen::Vector3d b = (3 * diff_rotvec - 2 * Tf * rv0 - Tf * d_diff_rotvec) /
  // (Tf * Tf); const Eigen::Vector3d c = (-2 * diff_rotvec + Tf * rv0 + Tf *
  // d_diff_rotvec) / (Tf * Tf * Tf); const Eigen::Vector3d th = a * s + b * s *
  // s + c * s * s * s; const Eigen::Vector3d dth = (a + 2 * b * s + 3 * c * s *
  // s) / delta_time; o_pose.linear() = m_plan.pose.at(idx - 1).linear() *
  // Eigen::AngleAxisd(th.norm(), 2 * th.normalized()).toRotationMatrix();
  // o_twist.tail<3>() = left_jacobian_inv(th) * dth;

  return InterpolationResult::OK;
}

} // namespace elastoplastic::utils::interpolation
