#ifndef ELASTOPLASTIC_CONTROLLER__ELASTOPLASTIC_VARIABLE_MODEL
#define ELASTOPLASTIC_CONTROLLER__ELASTOPLASTIC_VARIABLE_MODEL

#include "Eigen/Dense"

#include <boost/circular_buffer.hpp>

#ifdef BUILD_TESTING
#include "gtest/gtest.h"
#endif

namespace Eigen {
using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;
} // namespace Eigen

namespace elastoplastic {
struct ElastoplasticModelData {
  Eigen::Matrix6d inertia_inv;
  Eigen::Matrix6d k;
  Eigen::Matrix6d d;

  // z_start < z_kmax <= z_max
  double z_max;
  double z_kmax;
  double z_start;

  double reset_threshold;
  size_t buffer_size;

  std::vector<bool> enable_axis;
  ElastoplasticModelData()
      : z_max(0.0), z_kmax(0.0), z_start(0.0), reset_threshold(0.0),
        buffer_size(0) {
    inertia_inv.setZero();
    k.setZero();
    d.setZero();
    enable_axis.resize(6);
    std::fill(enable_axis.begin(), enable_axis.end(), true);
  }
};

class ElastoplasticModel {
private:
  Eigen::Matrix6d m_inertia_inv;
  Eigen::Matrix6d m_k;
  Eigen::Matrix6d m_var_k;
  Eigen::Matrix6d m_d;
  double m_z_max;
  double m_z_kmax;
  double m_z_start;

  Eigen::Vector6d m_z;

  boost::circular_buffer<double> m_reset_buffer;
  double m_reset_threshold;

  // double m_leak_coefficient;
  bool m_to_restore;
  bool m_was_plastic;

  Eigen::Vector6d m_enable_axis;

  Eigen::Matrix6d compute_k(const double z) const;
  Eigen::Vector6d compute_zp(const Eigen::Vector6d &z, const Eigen::Vector6d &u,
                             const double dt) const;
  Eigen::Matrix6d compute_coeff_in_b(const Eigen::Matrix6d &M,
                                     const Eigen::Affine3d &T_a_b) const;

#ifdef BUILD_TESTING
  FRIEND_TEST(ElastoplasticModelTest, privateComputeK);
  FRIEND_TEST(ElastoplasticModelTest, privateComputeZp);
  FRIEND_TEST(ElastoplasticModelTest, withSmallForceNoRef);
  FRIEND_TEST(ElastoplasticModelTest, withHighForceNoRef);
#endif

public:
  bool reset(const Eigen::Vector6d &f, const Eigen::Vector6d &v);
  double alpha(const double z) const;
  double alpha() const;
  void clear();
  Eigen::Vector6d z() const;
  bool is_plastic() const;
  bool became_plastic() const;
  bool to_restore() const;
  void restore();
  Eigen::Matrix6d get_inertia_inv() const { return m_inertia_inv; }
  Eigen::Vector6d get_enabled_axis() const { return m_enable_axis; }
  std::pair<double, double> get_reset_buffer_status() const;

  Eigen::Matrix6d compute_variable_matrices(const Eigen::Affine3d &T_a_b) const;
  std::tuple<Eigen::Matrix6d, Eigen::Matrix6d> get_matrices() const;
  Eigen::Vector6d update_z(const Eigen::Vector6d &uin, const double period);

  ElastoplasticModel(const ElastoplasticModelData &data);
  ElastoplasticModel(const Eigen::Matrix6d& K, const double z_max, const double z_kmax, const double z_start, const size_t reset_buffer_size, const double reset_threshold);
  ElastoplasticModel() = delete;
};

} // namespace elastoplastic

#endif
