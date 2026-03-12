#include "elastoplastic_lugre_controller/elastoplastic_variable_model.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"

#include <numeric>

namespace elastoplastic {
ElastoplasticModel::ElastoplasticModel(const ElastoplasticModelData &data)
    : m_k(data.k),
      m_z_max(data.z_max), m_z_kmax(data.z_kmax), m_z_start(data.z_start),
      m_reset_buffer(data.buffer_size), m_reset_threshold(data.reset_threshold),
      m_to_restore(false), m_was_plastic(false) {
  m_z.setZero();
  std::transform(data.enable_axis.begin(), data.enable_axis.end(),
                 m_enable_axis.begin(),
                 [](const bool b) { return static_cast<double>(b); });
}

ElastoplasticModel::ElastoplasticModel(const Eigen::Matrix6d& K, const double z_max, const double z_kmax, const double z_start, const size_t reset_buffer_size, const double reset_threshold) 
    : m_k(K), m_z_max(z_max), m_z_kmax(z_kmax), m_z_start(z_start), m_reset_buffer(reset_buffer_size), m_reset_threshold(reset_threshold), m_to_restore(false), m_was_plastic(false)
      {
          m_z.setZero();
      }

double ElastoplasticModel::alpha(const double z) const {
  const double &z_ba = m_z_start;
  const double &z_ss = m_z_kmax;
  if (std::abs(z) < z_ba) {
    return 0.0;
  } else if (std::abs(z) >= z_ss) {
    return 1.0;
  } else {
    return 0.5 * std::sin(M_PI * ((z - (z_ba + z_ss) / 2) / (z_ss - z_ba))) +
           0.5;
  }
}

double ElastoplasticModel::alpha() const { return alpha(m_z.norm()); }

void ElastoplasticModel::clear() {
  m_z.setZero();
  m_reset_buffer.clear();
}

Eigen::Vector6d ElastoplasticModel::z() const { return m_z; }

bool ElastoplasticModel::is_plastic() const { return m_z.norm() >= m_z_kmax; }

bool ElastoplasticModel::became_plastic() const {
  return !m_was_plastic && is_plastic();
}

bool ElastoplasticModel::to_restore() const { return m_to_restore; }

void ElastoplasticModel::restore() { m_to_restore = false; }

std::pair<double, double> ElastoplasticModel::get_reset_buffer_status() const {
  return std::make_pair(
      std::accumulate(m_reset_buffer.begin(), m_reset_buffer.end(), 0.0),
      (double)m_reset_buffer.size() / (double)m_reset_buffer.capacity());
}

Eigen::Vector6d ElastoplasticModel::compute_zp(const Eigen::Vector6d &z,
                                               const Eigen::Vector6d &u,
                                               const double dt) const {
  Eigen::Vector6d zp = (u - alpha(z.norm()) * z / m_z_max * u.norm());
  if (is_plastic() && (z + zp * dt).norm() < z.norm()) {
    zp.setZero();
  }
  return zp;
}

Eigen::Matrix6d ElastoplasticModel::compute_k(const double z) const {
  return m_k * (1 - alpha(z));
}

Eigen::Matrix6d
ElastoplasticModel::compute_coeff_in_b(const Eigen::Matrix6d &M,
                                       const Eigen::Affine3d &T_b_a) const {
  Eigen::Matrix6d T6; // [[T, 0], [0, T]]
  T6 << T_b_a.linear(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
      T_b_a.linear();
  return T6 * M * T6.transpose();
}

Eigen::Matrix6d ElastoplasticModel::compute_variable_matrices(
    const Eigen::Affine3d &T_a_b) const {
  Eigen::Matrix6d k_in_base = compute_coeff_in_b(compute_k(m_z.norm()), T_a_b);

  return k_in_base;
}

Eigen::Vector6d ElastoplasticModel::update_z(const Eigen::Vector6d &uin,
                                             const double period) {
  m_was_plastic = is_plastic();
  Eigen::Vector6d ret_zp = this->compute_zp(m_z, uin, period);
  // m_z = utils::rk4(
      // [this, &period](const Eigen::Vector6d &x_in, const Eigen::Vector6d &pu_in)
          // -> Eigen::Vector6d { return this->compute_zp(x_in, pu_in, period); },
      // m_z, uin, period);
  m_z += ret_zp * period;
  // m_z = std::max(0.0, m_z); // Non dovrebbe servire, però...
  m_to_restore |= this->is_plastic();
  return ret_zp;
}

bool ElastoplasticModel::reset(const Eigen::Vector6d &f,
                               const Eigen::Vector6d &v) {
  if (is_plastic()) {
    m_reset_buffer.push_back(f.dot(v));
    if (m_reset_buffer.full() &&
        std::accumulate(m_reset_buffer.begin(), m_reset_buffer.end(), 0.0) <
            m_reset_threshold) {
      clear();
      return true;
    }
  }
  return false;
}

} // namespace elastoplastic
