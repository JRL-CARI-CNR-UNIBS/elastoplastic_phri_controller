#ifndef ELASTOPLASTIC_LUGRE_CONTROLLER__UTILS_HPP
#define ELASTOPLASTIC_LUGRE_CONTROLLER__UTILS_HPP

#include "Eigen/Core"
#include "elastoplastic_lugre_controller/elastoplastic_model_6d.hpp"
#include "elastoplastic_parameters.hpp"

namespace Eigen {
using Vector6d = Vector<double,6>;
}

namespace elastoplastic
{

Eigen::VectorXd base_velocity_from_twist(const Eigen::Vector6d & p_w)
{
  return p_w({0, 1, 5});
}

Eigen::Vector<double, 6> twist_from_base_velocity(const Eigen::Vector3d & p_v)
{
  return Eigen::Vector6d {p_v(0), p_v(1), 0, 0, 0, p_v(2)};
}

ElastoplasticModelData get_model_data(const elastoplastic_controller::Params& a_params)
{
  ElastoplasticModelData data;
  std::vector<double> inertia = a_params.impedance.inertia;
  data.inertia_inv = Eigen::Map<Eigen::Vector6d>(inertia.data(), 6).cwiseInverse();
  data.lugre.linear.sigma_0 = a_params.impedance.linear.sigma_0;
  data.lugre.linear.sigma_1 = a_params.impedance.linear.sigma_1;
  data.lugre.linear.sigma_2 = a_params.impedance.linear.sigma_2;
  data.lugre.angular.sigma_0 = a_params.impedance.angular.sigma_0;
  data.lugre.angular.sigma_1 = a_params.impedance.angular.sigma_1;
  data.lugre.angular.sigma_2 = a_params.impedance.angular.sigma_2;
  data.lugre.z_ba = a_params.impedance.z_ba;
  data.lugre.z_ss = a_params.impedance.z_ss;
  data.lugre.tau_w = a_params.impedance.tau_w;
  data.reset_condition.reset_window_size =
      a_params.impedance.reset_condition.reset_window_size;
  data.reset_condition.reset_threshold = a_params.impedance.reset_condition.reset_threshold;
  return data;
}

}

#endif // ELASTOPLASTIC_LUGRE_CONTROLLER__UTILS_HPP
