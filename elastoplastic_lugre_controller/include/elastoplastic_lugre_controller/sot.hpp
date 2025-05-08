#ifndef ELASTOPLASTIC_CONTORLLER__SOT_HPP
#define ELASTOPLASTIC_CONTORLLER__SOT_HPP

#include "Eigen/Core"

namespace elastoplastic {

class Task {
private:
  const size_t m_prb_dim, m_task_dim;
  Eigen::MatrixXd m_Gt;
  Eigen::VectorXd m_Ft;
  Eigen::MatrixXd m_Ad;
  Eigen::VectorXd m_bd;

public:
  Task(const size_t problem_size, const size_t task_size) : m_prb_dim(problem_size), m_task_dim(task_size) {
    m_Gt = Eigen::MatrixXd::Zero(m_prb_dim, m_prb_dim);
    m_Ft = Eigen::VectorXd::Zero(m_prb_dim);
    m_Ad = Eigen::MatrixXd::Zero(m_task_dim, m_prb_dim);
    m_bd = Eigen::VectorXd::Zero(m_task_dim);
  }

  Eigen::MatrixXd& A() { return m_Ad; }
  Eigen::VectorXd& b() { return m_bd; }
  Eigen::MatrixXd G() const { return m_Gt; }
  Eigen::VectorXd F() const { return m_Ft; }
  void update_task(void) {
    m_Gt = m_Ad.transpose() * m_Ad;
    m_Ft = m_bd.transpose() * m_Ad;
  }
};

class Stack {
private:
  const size_t m_prb_dim;
  int m_level;
  const double m_level_step;
  Eigen::MatrixXd m_G;
  Eigen::MatrixXd m_F;

public:
  Stack(const size_t problem_size, const double level_step = 1e-3)
      : m_prb_dim(problem_size), m_level(0), m_level_step(level_step) {
    clear();
  }
  Eigen::MatrixXd G() const { return m_G; }
  Eigen::MatrixXd F() const { return m_F; }
  double level_step() const { return m_level_step; }
  int new_level(void) { return ++m_level; }
  void push_task(Task& t) {
    t.update_task();
    m_G += t.G() * std::pow(m_level_step, m_level);
    m_F += t.F() * std::pow(m_level_step, m_level);
  }
  void clear(void) {
    m_G = Eigen::MatrixXd::Zero(m_prb_dim, m_prb_dim);
    m_F = Eigen::VectorXd::Zero(m_prb_dim);
    m_level = 0;
  }
};

} // namespace elastoplastic

#endif // SOT_HPP
