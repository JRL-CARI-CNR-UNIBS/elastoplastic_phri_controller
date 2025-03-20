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
  // fmt::print(fmt::fg(fmt::color::cyan), "v: {}, w: {}, period: {}\n", v, w, period);
  Eigen::Matrix<double, 7, 1> alpha_with_r;
  alpha_with_r << m_state.z, m_state.r;
  m_last_alpha = Eigen::Vector6d::Constant(alpha(alpha_with_r.norm()));
  ModelState d_dt;
  d_dt.r = dalpha(m_state.z.norm());
  Eigen::Vector6d c_v = m_last_alpha.cwiseProduct(m_state.z) * velocity.norm() / m_model_params.lugre.z_ss;
  d_dt.z = velocity - c_v;
  d_dt.w = m_last_alpha.cwiseProduct(m_state.z - m_state.w) / m_model_params.lugre.tau_w;

  m_last_friction_force = m_sigma_0 * (m_state.z - m_state.w)
                        + m_sigma_1 * d_dt.z
                        + m_sigma_2 * velocity;


  // fmt::print(fmt::fg(fmt::color::cyan), "\n+----------------------\n"
  //                                         "Velocity:    {:8.4e} | {:8.4e} | {:8.4e}\n"
  //                                         "Input Force: {:8.4e} | {:8.4e} | {:8.4e}\n"
  //                                         "Friction:    {:8.4e} | {:8.4e} | {:8.4e}\n"
  //                                         "+----------------------\n",
  //     velocity(0), velocity(1), velocity(2),
  //     force(0), force(1), force(2),
  //     m_last_friction_force(0),m_last_friction_force(1),m_last_friction_force(2));

  Eigen::Vector6d acc;
  acc = m_model_params.inertia_inv.cwiseProduct(force - m_last_friction_force);


  m_state.z += d_dt.z * period;
  m_state.w += d_dt.w * period;
  m_state.r += d_dt.r * period;
  // fmt::print(fmt::fg(fmt::color::cyan), "z: {}, w: {}, r: {}\n", m_state.z, m_state.w, m_state.r);

  reset_condition(velocity, force, period);
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
