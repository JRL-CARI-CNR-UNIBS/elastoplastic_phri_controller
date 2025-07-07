#include "elastoplastic_lugre_controller/elastoplastic_variable_model.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"

#include <algorithm>

#include "fmt/core.h"

namespace elastoplastic {
ElastoplasticModel::ElastoplasticModel(const ElastoplasticModelData& data)
    : m_inertia_inv(data.inertia_inv), m_k(data.k), m_d(data.d), m_z_max(data.z_max), m_z_kmax(data.z_kmax),
      m_z_start(data.z_start), m_z(0), m_reset_buffer(data.buffer_size), m_reset_threshold(data.reset_threshold),
      m_to_restore(false), m_was_plastic(false) {
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

std::pair<double, double> ElastoplasticModel::get_reset_buffer_status() const {
  return std::make_pair(std::accumulate(m_reset_buffer.begin(), m_reset_buffer.end(), 0), m_reset_buffer.full());
}

double ElastoplasticModel::compute_zp(const double z, const double u, const double /*dt*/) const {
  auto aswitch = [this](const double z) {
    const double& z_ss = 1.00 * m_z_kmax;
    const double& z_ba = 1.02 * m_z_kmax;
    if (std::abs(z) < z_ba) {
      return 1.0;
    } else if (std::abs(z) >= z_ss) {
      return 0.0;
    } else {
      return 0.5 * std::sin(M_PI * ((z - (z_ba + z_ss) / 2) / (z_ba - z_ss))) + 0.5;
    }
  };
  double zp = u * (1 - alpha(z) * z / m_z_max * utils::sgn(u)) * aswitch(z);
  // * (-0.5 * std ::atan(1000 * (z - m_z_kmax)) / M_PI_2 + 0.5);
  // if (z < m_z_max && z + zp * dt > m_z_max) {
  //   zp = (m_z_max - z) / dt;
  // } else if (z > 0 && z + zp * dt < 0) {
  //   zp = -z / dt;
  // }
  // if (is_plastic() && zp < 0) {
  //   zp = 0;
  // }
  return zp;
}

Eigen::Matrix6d ElastoplasticModel::compute_k(const double z) const { return m_k * (1 - alpha(z)); }

Eigen::Matrix6d ElastoplasticModel::compute_coeff_in_b(const Eigen::Matrix6d& M, const Eigen::Affine3d& T_b_a) const {
  Eigen::Matrix6d T6; // [[T, 0], [0, T]]
  T6 << T_b_a.linear(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), T_b_a.linear();
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
  // m_z = std::max(0.0, m_z); // Non dovrebbe servire, però...
  m_to_restore |= this->is_plastic();
  return ret_zp;
}


bool ElastoplasticModel::reset(const Eigen::Vector6d& f, const Eigen::Vector6d& v) {
  if (is_plastic()) {
    m_reset_buffer.push_back(f.dot(v));
    if (m_reset_buffer.full() && std::accumulate(m_reset_buffer.begin(), m_reset_buffer.end(), 0.0) < m_reset_threshold) {
      m_reset_buffer.clear();
      m_z = 0;
      return true;
    }
  }
  return false;
}


} // namespace elastoplastic
