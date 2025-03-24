#include "elastoplastic_lugre_controller/elastoplastic_model_6d.hpp"

#include <numeric>

#include <fmt/ranges.h>
#include <fmt/color.h>

namespace elastoplastic {

ElastoplasticModel6D::ElastoplasticModel6D(const ElastoplasticModelData& data)
    : m_model_params(data)
{
  m_state.clear();

  m_sigma_0.setZero();
  m_sigma_1.setZero();
  m_sigma_2.setZero();
  m_sigma_0.diagonal().head<3>() = Eigen::Vector3d::Constant(m_model_params.lugre.linear.sigma_0);
  m_sigma_0.diagonal().tail<3>() = Eigen::Vector3d::Constant(m_model_params.lugre.angular.sigma_0);
  m_sigma_1.diagonal().head<3>() = Eigen::Vector3d::Constant(m_model_params.lugre.linear.sigma_1);
  m_sigma_1.diagonal().tail<3>() = Eigen::Vector3d::Constant(m_model_params.lugre.angular.sigma_1);
  m_sigma_2.diagonal().head<3>() = Eigen::Vector3d::Constant(m_model_params.lugre.linear.sigma_2);
  m_sigma_2.diagonal().tail<3>() = Eigen::Vector3d::Constant(m_model_params.lugre.angular.sigma_2);
  std::transform(data.enable_axis.begin(), data.enable_axis.end(), m_enable_axis.begin(),[](const bool b){
    return static_cast<double>(b);
  });
}

double ElastoplasticModel6D::alpha(const double z) const
{
  const double& z_ba = m_model_params.lugre.z_ba;
  const double& z_ss = m_model_params.lugre.z_ss;
  if (std::abs(z) < z_ba)
  {
    return 0.0;
  }
  else if (std::abs(z) >= z_ss)
  {
    return 1.0;
  }
  else
  {
    return 0.5*std::sin(M_PI*((z-(z_ba+z_ss)/2)/(z_ss-z_ba)))+0.5;
  }
}


double ElastoplasticModel6D::dalpha(const double z) const
{
  const double& z_ba = m_model_params.lugre.z_ba;
  const double& z_ss = m_model_params.lugre.z_ss;
  if (std::abs(z) < z_ba)
  {
    return 0.0;
  }
  else if (std::abs(z) >= z_ss)
  {
    return 0.0;
  }
  else
  {
    return 0.5*std::cos(M_PI*((z-(z_ba+z_ss)/2)/(z_ss-z_ba)));
  }
};

Eigen::Vector6d ElastoplasticModel6D::update(const Eigen::Vector6d& velocity, const Eigen::Vector6d& force, const double period)
{
  Eigen::Vector6d enabled_velocity = velocity.cwiseProduct(m_enable_axis);
  Eigen::Vector6d enabled_force = force.cwiseProduct(m_enable_axis);
  Eigen::Matrix<double, 7, 1> alpha_with_r;
  alpha_with_r << m_state.z, m_state.r;
  m_last_alpha = Eigen::Vector6d::Constant(alpha(alpha_with_r.norm()));
  ModelState d_dt;
  d_dt.r = dalpha(m_state.z.norm());
  Eigen::Vector6d c_v = m_last_alpha.cwiseProduct(m_state.z) * enabled_velocity.norm() / m_model_params.lugre.z_ss;
  d_dt.z = enabled_velocity - c_v;
  d_dt.w = m_last_alpha.cwiseProduct(m_state.z - m_state.w) / m_model_params.lugre.tau_w;

  m_last_friction_force = m_sigma_0 * (m_state.z - m_state.w)
                        + m_sigma_1 * d_dt.z
                        + m_sigma_2 * enabled_velocity;

  Eigen::Vector6d acc;
  acc = m_model_params.inertia_inv.cwiseProduct(enabled_force - m_last_friction_force);

  m_state.z += d_dt.z * period;
  m_state.w += d_dt.w * period;
  m_state.r += d_dt.r * period;

  reset_condition(enabled_velocity, enabled_force, period);
  return acc;
}

bool ElastoplasticModel6D::reset_condition(const Eigen::Vector6d& velocity, const Eigen::Vector6d& force, const double period)
{
  bool reset_status = false;
  if(m_last_alpha.maxCoeff() > 0)
  {
    const size_t window_reset_size = (size_t) std::ceil(m_model_params.reset_condition.reset_window_size/(period*1e3));
    m_reset_window.emplace_back(force.transpose() * velocity);
    if(m_reset_window.size() > window_reset_size)
    {
      m_reset_window.pop_front();
    }
    const double reset_value =
        std::accumulate(m_reset_window.begin(),
                        m_reset_window.end(),
                        0.0,
                        [&period](const double d, const double x) -> double
                          {
                            return d + x*period;
                          }
                        );

    if(m_reset_window.size() >= window_reset_size &&
        reset_value < m_model_params.reset_condition.reset_threshold)
    {
      m_state.clear();
      m_reset_window.clear();
      reset_status = true;
    }
  }
  return reset_status;
}

}
