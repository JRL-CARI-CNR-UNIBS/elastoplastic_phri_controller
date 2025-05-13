#ifndef ELASTOPLASTIC_CONTROLLER__OPERATORS_HPP
#define ELASTOPLASTIC_CONTROLLER__OPERATORS_HPP

#include <Eigen/Geometry>

namespace elastoplastic::utils::interpolation {

// skew: ℝ³ → so(3)
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d mat;
  mat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return mat;
}

// unskew: so(3) → ℝ³
inline Eigen::Vector3d vee(const Eigen::Matrix3d& m) {
  Eigen::Vector3d v;
  v << m(2, 1), m(0, 2), m(1, 0);
  return v;
}

// Left Jacobian A(θ) (eq. 3b)
inline Eigen::Matrix3d left_jacobian(const Eigen::Vector3d& theta) {
  double th = theta.norm();
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  if (th < 1e-8) {
    // Taylor: A ≈ I + ½[θ]×
    return I + 0.5 * hat(theta);
  }
  Eigen::Matrix3d K = hat(theta);
  double half = 0.5 * th;
  double cot_half = 1.0 / std::tan(half);
  double f = (1.0 - half * cot_half) / (th * th);
  return I + 0.5 * K + f * (K * K);
}

// Log map SO(3) → ℝ³ (extract rotation vector)
inline Eigen::Vector3d so3Log(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  return aa.axis() * aa.angle();
}

inline Eigen::Matrix3d left_jacobian_inv(const Eigen::Vector3d& v) {
  const double angle = v.norm();
  return Eigen::Matrix3d::Identity() - (1 - cos(angle)) / std::pow(angle, 2) * hat(v) +
         (angle - sin(angle)) / std::pow(angle, 3) * hat(v) * hat(v);
}

inline
Eigen::Vector3d quat_log(const Eigen::Quaterniond& q)
{
    constexpr double eps = 1e-9;
    double length = sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z());

    if (length < eps)
    {
        return Eigen::Vector3d({q.x(), q.y(), q.z()});
    }
    else
    {
        double halfangle = acos(std::min(std::max(q.w(), -1.0), 1.0));
        return Eigen::Vector3d({q.x(), q.y(), q.z()})/length * halfangle;
    }
}

inline
Eigen::Vector3d quat_log_min(const Eigen::Quaterniond& q, const Eigen::Vector3d& v0)
{
  // Probably not working
  int n = 1;
  double err, perr; err = perr = std::numeric_limits<double>::infinity();
  Eigen::Vector3d v, vmin;
  vmin = quat_log(q);
  if(vmin.norm() < 1e-9) return vmin;
  const double halfangle = vmin.norm();
  perr = std::sqrt(vmin.dot(v0));
  short int verse = -1;
  do
  {
    v = vmin.normalized() * verse*halfangle*n*2*M_PI;
    err = std::sqrt(v.dot(v0));
    if(err > perr)
    {
      vmin = v;
      if(verse != 1)
      {
        verse = 1;
        n = 0;
      }
      else
      {
        break;
      }
    }
    else
    {
      perr = err;
    }
    n++;
  }while(true);

  return vmin;
}

inline
Eigen::Quaterniond quat_exp(const Eigen::Vector3d& v)
{
    double halfangle = std::sqrt(v(1)*v(1) + v(2)*v(2) + v(0)*v(0));
    constexpr double eps = 1e-9;
    if (halfangle < eps)
    {
        return Eigen::Quaterniond({1.0, v(0), v(1), v(2)}).normalized();
    }
    else
    {
        double c = cos(halfangle);
        double s = sin(halfangle) / halfangle;
        return Eigen::Quaterniond(c, s * v(0), s * v(1), s * v(2));
    }
}

inline
Eigen::Matrix3d rot_log(const Eigen::Matrix3d& R)
{
  assert(R.trace() != -1);
  double phi = acos(R.trace() - 1);
  return phi/(2*sin(phi))*(R - R.transpose());
}

inline
Eigen::Matrix3d rot_exp(const Eigen::Vector3d& r)
{
  const double norm = r.norm();
  return Eigen::Matrix3d::Identity() + sin(norm) / norm * hat(r) + (1 - cos(norm)) / (norm * norm) * hat(r) * hat(r);
}

inline
Eigen::Matrix3d ang_vel_coeff_in_body_frame(const Eigen::Matrix3d& r)
{
  const double norm = vee(r).norm();
  return Eigen::Matrix3d::Identity() - (1-cos(norm))/(norm*norm) * r + (norm - sin(norm))/(norm*norm*norm)*r*r;
}

inline
Eigen::Matrix3d ang_vel_coeff_in_fixed_frame(const Eigen::Matrix3d& r)
{
  const double norm = vee(r).norm();
  return Eigen::Matrix3d::Identity() + (1-cos(norm))/(norm*norm) * r + (norm - sin(norm))/(norm*norm*norm)*r*r;
}

} // namespace elastoplastic::utils::interpolation

#endif // ELASTOPLASTIC_CONTROLLER__OPERATORS_HPP
