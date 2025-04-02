#ifndef ELASTOPLASTIC_LUGRE_CONTROLLER__UTILS_HPP
#define ELASTOPLASTIC_LUGRE_CONTROLLER__UTILS_HPP

#include "Eigen/Core"
#include "elastoplastic_lugre_controller/elastoplastic_model_6d.hpp"
#include "elastoplastic_parameters.hpp"

namespace Eigen {
using Vector6d = Vector<double,6>;
}

namespace elastoplastic {

constexpr static double K_ABS_EPSILON{1e-12};
constexpr static double K_REL_EPSILON{1e-8};

/**
 *  Floating point comparison
 *  https://www.learncpp.com/cpp-tutorial/relational-operators-and-floating-point-comparisons/
 */
template <typename T> constexpr T constAbs(T x) { return (x < 0 ? -x : x); }

// Return true if the difference between a and b is within epsilon percent of the larger of a and b
constexpr bool approximately_equal_rel(double a, double b, double relEpsilon) {
  return (constAbs(a - b) <= (std::max(constAbs(a), constAbs(b)) * relEpsilon));
}

// Return true if the difference between a and b is less than or equal to absEpsilon, or within relEpsilon percent of the larger
// of a and b
constexpr bool almost_equal(double a, double b, double absEpsilon = K_ABS_EPSILON, double relEpsilon = K_REL_EPSILON) {
  // Check if the numbers are really close -- needed when comparing numbers near zero.
  if (constAbs(a - b) <= absEpsilon)
    return true;

  // Otherwise fall back to Knuth's algorithm
  return approximately_equal_rel(a, b, relEpsilon);
}

constexpr bool almost_zero(const double a, const double absEpsilon = K_ABS_EPSILON) { return a < absEpsilon; }

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

Eigen::MatrixXd& regularize(Eigen::MatrixXd& m) {
  m += Eigen::MatrixXd::Identity(m.rows(), m.cols()) * K_REL_EPSILON * m.trace() / m.cols();
  return m;
}

Eigen::VectorXd base_velocity_from_twist(const Eigen::Vector6d& p_w) { return p_w({0, 1, 5}); }

Eigen::Vector<double, 6> twist_from_base_velocity(const Eigen::Vector3d& p_v) {
  return Eigen::Vector6d {p_v(0), p_v(1), 0, 0, 0, p_v(2)};
}

ElastoplasticModelData get_model_data(const elastoplastic_controller::Params& a_params)
{
  ElastoplasticModelData data;
  // std::vector<double> inertia = a_params.impedance.inertia;
  data.inertia_inv = Eigen::Vector6d(a_params.impedance.inertia.data()).cwiseInverse();
  data.lugre.sigma_0 = a_params.impedance.sigma_0;
  data.lugre.sigma_1 = a_params.impedance.sigma_1;
  data.lugre.sigma_2 = a_params.impedance.sigma_2;
  data.lugre.z_ba = a_params.impedance.z_ba;
  data.lugre.z_ss = a_params.impedance.z_ss;
  data.lugre.tau_w = a_params.impedance.tau_w;
  data.reset_condition.reset_window_size =
      a_params.impedance.reset_condition.reset_window_size;
  data.reset_condition.reset_threshold = a_params.impedance.reset_condition.reset_threshold;
  std::copy(a_params.impedance.enable_axis.begin(), a_params.impedance.enable_axis.end(), data.enable_axis.begin());
  return data;
}

} // namespace elastoplastic

#endif // ELASTOPLASTIC_LUGRE_CONTROLLER__UTILS_HPP
