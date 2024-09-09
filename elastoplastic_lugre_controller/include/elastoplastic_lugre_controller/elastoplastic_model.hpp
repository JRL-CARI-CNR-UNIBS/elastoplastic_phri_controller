#ifndef ELASTOPLASTIC_MODEL_HPP
#define ELASTOPLASTIC_MODEL_HPP

#include <eigen3/Eigen/Core>
#include <deque>

namespace elastoplastic {

struct ElastoplasticModelData
{
  Eigen::Vector3d inertia_inv;
  struct {
    double sigma_0;
    double sigma_1;
    double sigma_2;
    double z_ss;
    double z_ba;
    double tau_w;
  } lugre;
  struct {
    double reset_window_size;
    double reset_threshold;
  } reset_condition;
};

class ElastoplasticModel
{
public:
  ElastoplasticModel(const ElastoplasticModelData& data);

  double alpha(const double z) const;
  double alpha() const {return m_last_alpha.maxCoeff();}
  double dalpha(const double z) const;

  void clear() {
    m_state.clear();
  }

  Eigen::Vector3d update(const Eigen::Vector3d& v, const Eigen::Vector3d& f, double period);

  const Eigen::Vector3d& z() const {return m_state.z;}
  const Eigen::Vector3d& w() const {return m_state.w;}
  const double& r() const {return m_state.r;}
  const Eigen::Vector3d& friction_force(){return m_last_friction_force;}
  const ElastoplasticModelData& params(){return m_model_params;}

protected:

  bool check_reset_condition();

  const ElastoplasticModelData m_model_params;

  Eigen::Vector3d m_last_alpha;
  Eigen::Vector3d m_last_friction_force;

  struct ModelState{
    Eigen::Vector3d z;
    Eigen::Vector3d w;
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

#endif // ELASTOPLASTIC_MODEL_HPP
