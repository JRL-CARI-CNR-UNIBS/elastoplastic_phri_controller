#ifndef ELASTOPLASTIC_MODEL_6D_H
#define ELASTOPLASTIC_MODEL_6D_H

#include <eigen3/Eigen/Core>
#include <deque>

namespace Eigen {
using Vector6d = Matrix<double, 6, 1>;
}

namespace elastoplastic {

struct ElastoplasticModelData
{
  Eigen::Vector6d inertia_inv;
  struct {
    struct Impedance {
        double sigma_0;
        double sigma_1;
        double sigma_2;
    } linear, angular;
    double z_ss;
    double z_ba;
    double tau_w;
  } lugre;
  struct {
    double reset_window_size;
    double reset_threshold;
  } reset_condition;
};

class ElastoplasticModel6D
{
public:
  ElastoplasticModel6D(const ElastoplasticModelData& data);

  double alpha(const double z) const;
  double alpha() const {return m_last_alpha.maxCoeff();}
  double dalpha(const double z) const;

  void clear() {
    m_state.clear();
  }

  Eigen::Vector6d update(const Eigen::Vector6d& v, const Eigen::Vector6d& f, const double period);

  const Eigen::Vector6d& z() const {return m_state.z;}
  const Eigen::Vector6d& w() const {return m_state.w;}
  const double& r() const {return m_state.r;}
  const Eigen::Vector6d& friction_force(){return m_last_friction_force;}
  const ElastoplasticModelData& params(){return m_model_params;}

protected:

  bool reset_condition(const Eigen::Vector6d& v, const Eigen::Vector6d& f, const double period);

  const ElastoplasticModelData m_model_params;
  Eigen::Matrix<double, 6, 6> m_sigma_0, m_sigma_1, m_sigma_2;

  Eigen::Vector6d m_last_alpha;
  Eigen::Vector6d m_last_friction_force;

  struct ModelState{
    Eigen::Vector6d z;
    Eigen::Vector6d w;
    double r;

    void clear() {
      z.setZero();
      w.setZero();
      r = 0;
    }
  } m_state;

  std::deque<double> m_reset_window;
};

} // namespace elastoplastic

#endif // ELASTOPLASTIC_MODEL_6D_H
