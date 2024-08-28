#include "elastoplastic_lugre_controller/elastoplastic_model.hpp"

#include <numeric>

namespace elastoplastic {

ElastoplasticModel::ElastoplasticModel(const ElastoplasticModelData& data)
    : m_model_params(data)
{
  m_state.clear();
}

double ElastoplasticModel::alpha(const double z) const
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


double ElastoplasticModel::dalpha(const double z) const
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

Eigen::Vector3d ElastoplasticModel::update(const Eigen::Vector3d& v, const Eigen::Vector3d& w, double period)
{
  Eigen::Vector4d alpha_with_r;
  alpha_with_r << m_state.z, m_state.r;
  m_last_alpha = Eigen::Vector3d::Constant(alpha(alpha_with_r.norm()));
  ModelState d_dt;
  d_dt.r = dalpha(m_state.z.norm());
  d_dt.z = v - m_last_alpha.cwiseProduct(m_state.z) * v.norm() / m_model_params.lugre.z_ss;
  d_dt.w = m_last_alpha.cwiseProduct(m_state.z - m_state.w) / m_model_params.lugre.tau_w;

  m_last_friction_force = m_model_params.lugre.sigma_0 * (m_state.z - m_state.w)
                        + m_model_params.lugre.sigma_1 * d_dt.z
                        + m_model_params.lugre.sigma_2 * v;

  Eigen::Vector3d acc;
  acc = m_model_params.inertia_inv.cwiseProduct(w - m_last_friction_force);


  m_state.z += d_dt.z * period;
  m_state.w += d_dt.w * period;
  m_state.r += d_dt.r * period;

  // Reset condition
  if(m_last_alpha.maxCoeff() > 0)
  {
    const size_t window_reset_size = (size_t) std::ceil(m_model_params.reset_condition.reset_window_size/(period*1e3));
    m_reset_window.emplace_back(w.transpose() * v);
    if(m_reset_window.size() > window_reset_size)
    {
      m_reset_window.pop_front();
    }
    const double reset_value = std::accumulate(m_reset_window.begin(),
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
    }
    else
    {
      if(!m_reset_window.empty())
      {
        m_reset_window.clear();
      }
    }
  }

  return acc;
}

}
