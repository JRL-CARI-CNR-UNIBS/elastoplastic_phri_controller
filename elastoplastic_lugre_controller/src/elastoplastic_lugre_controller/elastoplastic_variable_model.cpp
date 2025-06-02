#include "elastoplastic_lugre_controller/elastoplastic_variable_model.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"

#include "fmt/format.h"

namespace elastoplastic {
ElastoplasticModel::ElastoplasticModel(const ElastoplasticModelData& data)
    : m_inertia_inv(data.inertia_inv), m_k(data.k), m_d(data.d), m_z_max(data.z_max), m_z_kmax(data.z_kmax),
      m_z_start(data.z_start), m_z(0), m_leak_coefficient(data.leak_coefficient), m_to_restore(false), m_was_plastic(false) {
  std::transform(data.enable_axis.begin(), data.enable_axis.end(), m_enable_axis.begin(),
                 [](const bool b) { return static_cast<double>(b); });
}

double ElastoplasticModel::alpha(const double z) const {
  const double& z_ba = m_z_start;
  const double& z_ss = m_z_kmax;
  if (std::abs(z) < z_ba) {
    return 0.0;
  } else if (std::abs(z) >= z_ss) {
    return 1.0;
  } else {
    return 0.5 * std::sin(M_PI * ((z - (z_ba + z_ss) / 2) / (z_ss - z_ba))) + 0.5;
  }
}

double ElastoplasticModel::alpha() const { return alpha(m_z); }

void ElastoplasticModel::clear() {
  m_z = 0;
}

double ElastoplasticModel::z() const { return m_z; }

bool ElastoplasticModel::is_plastic() const { return m_z >= m_z_kmax; }

bool ElastoplasticModel::became_plastic() const { return !m_was_plastic && is_plastic(); }

bool ElastoplasticModel::to_restore() const { return m_to_restore; }

void ElastoplasticModel::restore() { m_to_restore = false; }

double ElastoplasticModel::compute_zp(const double z, const double u, const double dt) const {
  double leak = z >= 0.90 * m_z_kmax ? 1.0 : 0.0;
  // double leak{1.0};
  // double leak{0.0};
  // double leak = std::abs(u) >= 1e-3 ? 0.0 : 1.0;
  // double zp = u * (1 - z / m_z_max * utils::sgn(u)) - leak * m_k.norm() * z / m_z_max;
  double zp = u * (1 - z / m_z_max * utils::sgn(u)) - leak * m_leak_coefficient * z;
  if (z < m_z_max && z + zp * dt > m_z_max) {
    zp = (m_z_max - z) / dt;
  } else if (z > 0 && z + zp * dt < 0) {
    zp = -z / dt;
  }
  return zp;
}

Eigen::Matrix6d ElastoplasticModel::compute_k(const double z) const { return m_k * (1 - alpha(z)); }

Eigen::Matrix6d ElastoplasticModel::compute_coeff_in_b(const Eigen::Matrix6d& M, const Eigen::Affine3d& T_a_b) const {
  Eigen::Matrix6d T6; // [[T, 0], [0, T]]
  T6 << T_a_b.linear(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), T_a_b.linear();
  return T6 * M * T6.transpose();
}

std::tuple<Eigen::Matrix6d, Eigen::Matrix6d> ElastoplasticModel::compute_variable_matrices(const Eigen::Affine3d& T_a_b) const {
  Eigen::Matrix6d k_in_base = compute_coeff_in_b(compute_k(m_z), T_a_b);
  Eigen::Matrix6d d_in_base = compute_coeff_in_b(m_d, T_a_b);

  return std::make_tuple(k_in_base, d_in_base);
}

Eigen::Vector6d ElastoplasticModel::compute_impedance(const Eigen::Vector6d& x, const Eigen::Vector6d& v,
                                                      const Eigen::Vector6d& f, const Eigen::Affine3d& T_a_b) const {
  Eigen::Vector6d fe = f.cwiseProduct(m_enable_axis);

  auto [k_in_base, d_in_base] = compute_variable_matrices(T_a_b);
  // std::cout << "k: " << k_in_base.diagonal()(0) << ", x: " << x(0) << ", d: " << d_in_base(0, 0) << ", v: " << v(0)
  //           << ", fe: " << fe(0) << std::endl;
  return m_inertia_inv * (fe - k_in_base * x - d_in_base * v);
}

double ElastoplasticModel::update_z(const double uin, const double period) {
  m_was_plastic = is_plastic();
  double ret_zp = this->compute_zp(m_z, uin, period);
  m_z =
    utils::rk4([this, &period](const double& xin, const double& puin) -> double { return this->compute_zp(xin, puin, period); },
               m_z, uin, period);
  m_z = std::max(0.0, m_z); // Non dovrebbe servire, però...
  m_to_restore |= this->is_plastic();
  return ret_zp;
}

std::tuple<Eigen::Vector6d, Eigen::Vector6d, Eigen::Vector6d>
ElastoplasticModel::update(const Eigen::Vector6d& x, const Eigen::Vector6d& v, const Eigen::Vector6d& f,
                           const Eigen::Affine3d T_a_b, const double period) {
  Eigen::Vector6d xspp = compute_impedance(x, v, f, T_a_b);
  Eigen::Vector6d fe = f.cwiseProduct(m_enable_axis);
  double Pin = fe.transpose() * v;
  // Integrate z
  this->update_z(Pin, period);
  // Saturation
  // m_z = std::max(0.0, m_z); // Non dovrebbe essere necessario
  // Integrate acc
  auto [xs, xsp] = utils::rk4_double(
    [this, T_a_b](const Eigen::Vector6d& xin, const Eigen::Vector6d& vin, const Eigen::Vector6d& uin) -> Eigen::Vector6d {
      return this->compute_impedance(xin, vin, uin, T_a_b);
    },
    x, v, f, period);
  // std::cout << "xpp: " << xspp.transpose() << "\nxsp: " << xsp.transpose() << std::endl;
  return std::make_tuple(xs, xsp, xspp);
}

} // namespace elastoplastic
