#ifndef ELASTOPLASTIC_CONTORLLER__SOT_HPP
#define ELASTOPLASTIC_CONTORLLER__SOT_HPP

#include "Eigen/Dense"
#include "eiquadprog/eiquadprog-fast.hpp"
#include <numeric>

namespace elastoplastic {

class Task {
private:
  const size_t m_prb_dim, m_task_dim;
  Eigen::MatrixXd m_Ad;
  Eigen::VectorXd m_bd;
  Eigen::MatrixXd m_W;

public:
  Task(const size_t problem_size, const size_t task_size)
      : m_prb_dim(problem_size), m_task_dim(task_size), m_Ad(task_size, problem_size), m_bd(task_size),
        m_W(task_size, task_size) {
    m_Ad.setZero();
    m_bd.setZero();
    m_W.setIdentity();
  }

  Eigen::MatrixXd& A() { return m_Ad; }
  Eigen::VectorXd& b() { return m_bd; }
  const Eigen::MatrixXd& A() const { return m_Ad; }
  const Eigen::VectorXd& b() const { return m_bd; }
  Eigen::MatrixXd& W() { return m_W; }
  Eigen::VectorXd value(const Eigen::VectorXd& x) { return m_Ad * x + m_bd; }
  double cost(const Eigen::VectorXd& x) const {
    return (x.transpose() * m_Ad.transpose() * m_W * m_Ad * x + m_bd.transpose() * m_W * m_Ad * x).eval()(0);
  }
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> update_task(void) {
    Eigen::MatrixXd Gt = m_Ad.transpose() * m_W * m_Ad;
    Eigen::VectorXd Ft = m_bd.transpose() * m_W * m_Ad;
    return std::make_pair(Gt, Ft);
  }
  size_t size() const { return m_task_dim; }
  size_t problem_size() const { return m_prb_dim; }
};


class Stack {
private:
  const size_t m_prb_dim;
  int m_level;
  const double m_level_step;
  Eigen::MatrixXd m_G;
  Eigen::MatrixXd m_F;

public:
  Stack(const size_t problem_size, const double level_step = 1e-3, const double level_zero = 0)
      : m_prb_dim(problem_size), m_level(level_zero), m_level_step(level_step) {
    clear();
  }
  Eigen::MatrixXd G() const { return m_G; }
  Eigen::MatrixXd F() const { return m_F; }
  double level_step() const { return m_level_step; }
  int new_level(void) { return ++m_level; }
  void push_task(Task& t, const double relative_task_weight = 1.0) {
    auto [G, F] = t.update_task();
    m_G += G * relative_task_weight * std::pow(m_level_step, m_level);
    m_F += F * relative_task_weight * std::pow(m_level_step, m_level);
  }
  void clear(void) {
    m_G = Eigen::MatrixXd::Zero(m_prb_dim, m_prb_dim);
    m_F = Eigen::VectorXd::Zero(m_prb_dim);
    m_level = 0;
  }
};

using EqualityConstraint = Task;
class EqualitySet {
private:
  std::vector<std::reference_wrapper<const EqualityConstraint>> m_eq;
  const size_t m_prb_size;
  Eigen::MatrixXd m_CE;
  Eigen::VectorXd m_ce;
  void reset(const size_t dim) {
    m_CE.resize(dim, m_prb_size);
    m_ce.resize(dim);
    m_CE.setZero();
    m_ce.setZero();
  }

public:
  EqualitySet(const size_t problem_size) : m_prb_size(problem_size), m_CE(0, problem_size), m_ce(0) {}
  Eigen::MatrixXd CE() const { return m_CE; }
  Eigen::VectorXd ce() const { return m_ce; }
  size_t size() const { return m_ce.size(); }
  size_t problem_size() const { return m_prb_size; }
  void clear() {
    reset(0);
    m_eq.clear();
  }

  void push_constraint(const Task& ec) { m_eq.push_back(ec); }
  void compute_set() {
    size_t eq_size = std::accumulate(m_eq.begin(), m_eq.end(), 0,
                                     [](const size_t acc, const EqualityConstraint& eq) -> size_t { return acc + eq.size(); });
    if (eq_size > 0) {
      this->reset(eq_size);
      m_CE(Eigen::seqN(0, m_eq.at(0).get().size()), Eigen::all) << m_eq.at(0).get().A();
      m_ce.segment(0, m_eq.at(0).get().size()) << m_eq.at(0).get().b();
      for (size_t idx = 1; idx < m_eq.size(); ++idx) {
        m_CE(Eigen::seqN(m_eq.at(idx - 1).get().size(), m_eq.at(idx).get().size()), Eigen::all) << m_eq.at(idx).get().A();
        m_ce.segment(m_eq.at(idx - 1).get().size(), m_eq.at(idx).get().size()) << m_eq.at(idx).get().b();
      }
    } else {
      // No equality constraints: https://github.com/liuq/QuadProgpp/issues/3
      reset(1);
      m_CE.setConstant(std::numeric_limits<double>::epsilon());
      m_ce.setConstant(std::numeric_limits<double>::epsilon());
    }
  }
};


class InequalityConstraint {
private:
  Eigen::MatrixXd m_CI;
  Eigen::VectorXd m_ci;
  const size_t m_prb_size;
  const size_t m_constr_size;

public:
  InequalityConstraint(const size_t problem_size, const size_t constr_size)
      : m_CI(constr_size, problem_size), m_ci(constr_size), m_prb_size(problem_size), m_constr_size(constr_size) {
    m_CI.setZero();
    m_ci.setZero();
  }

  Eigen::MatrixXd CI() const { return m_CI; }
  Eigen::VectorXd ci() const { return m_ci; }
  size_t size() const { return m_constr_size; }
  size_t problem_size() const { return m_prb_size; }
};

class InequalitySet {
private:
  std::vector<std::reference_wrapper<const InequalityConstraint>> m_neq;
  const size_t m_prb_size;
  Eigen::MatrixXd m_CI;
  Eigen::VectorXd m_ci;
  void reset(const size_t dim) {
    m_CI.resize(dim, m_prb_size);
    m_ci.resize(dim);
    m_CI.setZero();
    m_ci.setZero();
  }

public:
  InequalitySet(const size_t problem_size) : m_prb_size(problem_size), m_CI(0, problem_size), m_ci(0) {}
  Eigen::MatrixXd CI() const { return m_CI; }
  Eigen::VectorXd ci() const { return m_ci; }
  size_t size() const { return m_ci.size(); }
  size_t problem_size() const { return m_prb_size; }
  void clear() {
    reset(0);
    m_neq.clear();
  }

  void push_constraint(const InequalityConstraint& ic) { m_neq.push_back(ic); }
  void compute_set() {
    size_t neq_size =
      std::accumulate(m_neq.begin(), m_neq.end(), 0,
                      [](const size_t acc, const InequalityConstraint& ineq) -> size_t { return acc + ineq.size(); });
    this->reset(neq_size);
    m_CI(Eigen::seqN(0, m_neq.at(0).get().size()), Eigen::all) << m_neq.at(0).get().CI();
    m_ci.segment(0, m_neq.at(0).get().size()) << m_neq.at(0).get().ci();
    for (size_t idx = 1; idx < m_neq.size(); ++idx) {
      m_CI(Eigen::seqN(m_neq.at(idx - 1).get().size(), m_neq.at(idx).get().size()), Eigen::all) << m_neq.at(idx).get().CI();
      m_ci.segment(m_neq.at(idx - 1).get().size(), m_neq.at(idx).get().size()) << m_neq.at(idx).get().ci();
    }
  }
};

using SolverStatus = eiquadprog::solvers::EiquadprogFast_status;
class SolverQP {
private:
  eiquadprog::solvers::EiquadprogFast m_solver;
  const size_t m_prb_dim;
  const Stack& m_stack;
  const EqualitySet& m_eq;
  const InequalitySet& m_ineq;

public:
  SolverQP(const size_t problem_size, const Stack& stack, const EqualitySet& eq, const InequalitySet& ineq)
      : m_prb_dim(problem_size), m_stack(stack), m_eq(eq), m_ineq(ineq) {}
  std::pair<Eigen::VectorXd, SolverStatus> solve() {
    m_solver.reset(m_prb_dim, m_eq.size(), m_ineq.size());
    Eigen::VectorXd sol;
    SolverStatus status = m_solver.solve_quadprog(m_stack.G(), m_stack.F(), m_eq.CE(), m_eq.ce(), m_ineq.CI(), m_ineq.ci(), sol);
    return std::make_pair(sol, status);
  }
};

} // namespace elastoplastic

#endif // SOT_HPP
