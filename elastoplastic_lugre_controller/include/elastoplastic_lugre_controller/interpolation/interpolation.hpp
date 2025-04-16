#ifndef ELASTOPLASTIC_CONTROLLER__INTERPOLATION_HPP
#define ELASTOPLASTIC_CONTROLLER__INTERPOLATION_HPP

#include "elastoplastic_lugre_controller/interpolation/operators.hpp"

namespace elastoplastic::utils::interpolation {

struct Spline3Coeff{
  std::vector<Eigen::Vector3d> a;
  std::vector<Eigen::Vector3d> b;
  std::vector<Eigen::Vector3d> c;
  Spline3Coeff(const size_t size): a(size), b(size), c(size) {}
  Spline3Coeff() = default;
};

inline Spline3Coeff
prepare_lie_spline(const std::vector<Eigen::Affine3d>& t_poses,
                   const Eigen::Vector3d& t_init_w,
                   const Eigen::Vector3d& t_init_a)
{
  assert(not (t_poses.size() < 2));
  std::vector<Eigen::Matrix3d> r(t_poses.size()), A(t_poses.size());
  for(size_t idx = 1; idx < t_poses.size(); ++idx)
  {
    r.at(idx) = rot_log(t_poses.at(idx-1).linear().transpose() * t_poses.at(idx).linear());
    A.at(idx) = ang_vel_coeff_in_body_frame(r.at(idx));
  }

  Spline3Coeff coeff(t_poses.size());
  coeff.c.at(1) = t_init_w;
  coeff.b.at(1) = t_init_a * 0.5;
  coeff.a.at(1) = unskew(r.at(1)) - coeff.b.at(1) - coeff.c.at(1);
  for(size_t idx = 2; idx < t_poses.size(); ++idx)
  {
    const Eigen::Vector3d s = unskew(r.at(idx));
    const Eigen::Vector3d t = 3*coeff.a.at(idx-1) + 2*coeff.b.at(idx-1) + coeff.c.at(idx-1);
    const Eigen::Vector3d u = 3*coeff.a.at(idx-1) + 2*coeff.b.at(idx-1);
    const double s_norm = s.norm();
    coeff.c.at(idx) = A.at(idx-1)*coeff.c.at(idx-1);
    coeff.b.at(idx) = 0.5 * (u - (s.dot(t))/std::pow(s_norm,4) * (2*cos(s_norm) + s_norm*sin(s_norm) - 2) * (s.cross(t)) - (1-cos(s_norm))/std::pow(s_norm,2) * (s.cross(u))
                       + (s.dot(t))/std::pow(s_norm,5) * (3*sin(s_norm) - s_norm*cos(s_norm) - 2*s_norm) * (s.cross(s.cross(t)))
                       + (s_norm - sin(s_norm))/std::pow(s_norm,3) * (t.cross(s.cross(t)) + s.cross(s.cross(u))) );
    coeff.a.at(idx) = s - coeff.b.at(idx) - coeff.c.at(idx);
  }
  return coeff;
}

inline
std::tuple<Eigen::Matrix3d, Eigen::Vector3d>
lie_spline(const double t_s,
                const double t_delta_t,
                const std::array<Eigen::Vector3d,3> coeff,
                const Eigen::Matrix3d& t_R0)
{
  const Eigen::Vector3d r = coeff[0] * t_s*t_s*t_s + coeff[1] * t_s*t_s + coeff[2] * t_s;
  Eigen::Matrix3d pose = t_R0 * rot_exp(r);
  Eigen::Vector3d ang_vel = ang_vel_coeff_in_body_frame(skew(r)) * (3*coeff[0] * t_s*t_s + 2*coeff[1] * t_s + coeff[2]) / t_delta_t;
  return std::tuple<Eigen::Matrix3d, Eigen::Vector3d>(pose, ang_vel);
}

} // namespace elastoplastic::utils::interpolation

#endif // ELASTOPLASTIC_CONTROLLER__INTERPOLATION_HPP
