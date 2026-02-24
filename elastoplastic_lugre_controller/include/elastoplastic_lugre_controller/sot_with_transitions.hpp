#ifndef ELASTOPLASTIC_CONTROLLER__SOT_HPP
#define ELASTOPLASTIC_CONTROLLER__SOT_HPP

#include <Eigen/Dense>
// #define EIQGUADPROG_TRACE_SOLVER
#include "eiquadprog/eiquadprog-fast.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace elastoplastic {

constexpr double TRANSITIONING_ERROR = 1.0e-9;

class Task {
private:
  const size_t m_prb_dim, m_task_dim;
  Eigen::MatrixXd m_Ad;
  Eigen::VectorXd m_bd;
  Eigen::MatrixXd m_W;
  std::string m_description;

public:
  Task() = delete;
  Task(const size_t problem_size, const size_t task_size,
       const std::string &description)
      : m_prb_dim(problem_size), m_task_dim(task_size),
        m_Ad(task_size, problem_size), m_bd(task_size),
        m_W(task_size, task_size), m_description(description) {
    m_Ad.setZero();
    m_bd.setZero();
    m_W.setIdentity();
  }
  Task(const size_t problem_size, const size_t task_size)
      : Task(problem_size, task_size, "") {}

  Eigen::MatrixXd &A() { return m_Ad; }
  Eigen::VectorXd &b() { return m_bd; }
  const Eigen::MatrixXd &A() const { return m_Ad; }
  const Eigen::VectorXd &b() const { return m_bd; }
  Eigen::MatrixXd &W() { return m_W; }
  const Eigen::MatrixXd &W() const { return m_W; }

  Eigen::VectorXd value(const Eigen::VectorXd &x) { return m_Ad * x + m_bd; }

  const std::string &describe() { return m_description; }
  const std::string &describe() const { return m_description; }

  double cost(const Eigen::VectorXd &x) const {
    return (x.transpose() * m_Ad.transpose() * m_W * m_Ad * x +
            m_Ad.transpose() * m_W * m_bd * x)
        .eval()(0);
  }

  std::pair<Eigen::MatrixXd, Eigen::VectorXd> update_task(void) const {
    Eigen::MatrixXd Gt = m_Ad.transpose() * m_W * m_Ad;
    Eigen::VectorXd Ft = m_Ad.transpose() * m_W * m_bd;
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
  Eigen::VectorXd m_F;
  std::vector<std::tuple<Eigen::MatrixXd, Eigen::VectorXd, std::string>>
      m_stack;

public:
  Stack(const size_t problem_size, const double level_step = 1e-3,
        const double level_zero = 0)
      : m_prb_dim(problem_size), m_level(level_zero), m_level_step(level_step) {
    clear();
  }

  Eigen::MatrixXd G() const { return m_G; }
  Eigen::VectorXd F() const { return m_F; }
  double level_step() const { return m_level_step; }
  int new_level(void) { return ++m_level; }
  int level(void) const { return m_level; }

  void push_task(Task &t, const double relative_task_weight = 1.0) {
    auto [G, F] = t.update_task();
    const double s = relative_task_weight * std::pow(m_level_step, m_level);
    m_stack.emplace_back(std::make_tuple(G * s, F * s, t.describe()));
    m_G += G * s;
    m_F += F * s;
  }

  void insert_task(Task &t, const int level,
                   const double relative_task_weight = 1.0) {
    auto [G, F] = t.update_task();
    const double s = relative_task_weight * std::pow(m_level_step, level);
    m_stack.emplace_back(std::make_tuple(G * s, F * s, t.describe()));
    m_G += G * s;
    m_F += F * s;
  }

  // NOTE: preserved original misspelling ("contibutions") for backwards compat.
  std::vector<std::pair<std::string, double>>
  contibutions(const Eigen::VectorXd &q) {
    std::vector<std::pair<std::string, double>> c(m_stack.size());
    double final_cost =
        (q.transpose() * m_G * q + 2.0 * m_F.transpose() * q)(0, 0);

    std::transform(m_stack.begin(), m_stack.end(), c.begin(),
                   [&q, &final_cost](const auto &tup) {
                     double cost =
                         (q.transpose() * std::get<0>(tup) * q +
                          2.0 * std::get<1>(tup).transpose() * q)(0, 0);
                     return std::make_pair(std::get<2>(tup), cost / final_cost);
                   });
    return c;
  }

  /// Number of quadratic terms currently in the stack (after weighting).
  size_t task_count() const { return m_stack.size(); }

  /// Remove the most recently pushed/inserted task term from the stack.
  /// Useful for temporary "transition" tasks injected for a single solve.
  void pop_task() {
    if (m_stack.empty()) {
      return;
    }
    m_G -= std::get<0>(m_stack.back());
    m_F -= std::get<1>(m_stack.back());
    m_stack.pop_back();
  }

  /// Pop tasks until the stack size equals `target_size`.
  void pop_to_size(const size_t target_size) {
    while (m_stack.size() > target_size) {
      pop_task();
    }
  }

  void clear(void) {
    prepare();
    m_level = 0;
  }

  /// Reset quadratic accumulator AND the stored task-term list.
  /// (The original header reset only m_G/m_F, which makes m_stack inconsistent.)
  void prepare(void) {
    m_G = Eigen::MatrixXd::Zero(m_prb_dim, m_prb_dim);
    m_F = Eigen::VectorXd::Zero(m_prb_dim);
    m_stack.clear();
  }

  void symmetrize(void) { m_G = (m_G + m_G.transpose()) * 0.5; }

  void regularize(const double eps = 1e-8) {
    m_G.diagonal() += Eigen::VectorXd::Constant(m_prb_dim, eps);
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
  EqualitySet() = delete;
  EqualitySet(const size_t problem_size)
      : m_prb_size(problem_size), m_CE(0, problem_size), m_ce(0) {}

  const Eigen::MatrixXd &CE() const { return m_CE; }
  const Eigen::VectorXd &ce() const { return m_ce; }
  Eigen::MatrixXd &CE() { return m_CE; }
  Eigen::VectorXd &ce() { return m_ce; }

  size_t size() const { return m_ce.size(); }
  size_t problem_size() const { return m_prb_size; }

  void clear() {
    reset(0);
    m_eq.clear();
  }

  void push_constraint(const Task &ec) { m_eq.push_back(ec); }

  void compute_set() {
    size_t eq_size = std::accumulate(
        m_eq.begin(), m_eq.end(), 0,
        [](const size_t acc, const EqualityConstraint &eq) -> size_t {
          return acc + eq.size();
        });

    if (eq_size > 0) {
      this->reset(eq_size);
      size_t level = 0;
      for (size_t idx = 0; idx < m_eq.size(); ++idx) {
        m_CE.middleRows(level, m_eq.at(idx).get().size())
            << m_eq.at(idx).get().A();
        m_ce.segment(level, m_eq.at(idx).get().size())
            << m_eq.at(idx).get().b();
        level += m_eq.at(idx).get().size();
      }
    } else {
      // No equality constraints: keep empty matrices (0 rows).
      reset(0);
    }
  }
};

class InequalityConstraint {
private:
  Eigen::MatrixXd m_CI;
  Eigen::VectorXd m_ci;
  const size_t m_prb_size;
  const size_t m_constr_size;
  std::string m_description;

public:
  InequalityConstraint() = delete;

  InequalityConstraint(const size_t problem_size, const size_t constr_size,
                       const std::string &description)
      : m_CI(constr_size, problem_size), m_ci(constr_size),
        m_prb_size(problem_size), m_constr_size(constr_size),
        m_description(description) {
    m_CI.setZero();
    m_ci.setZero();
  }

  InequalityConstraint(const size_t problem_size, const size_t constr_size)
      : InequalityConstraint(problem_size, constr_size, "No Description") {}

  std::string description() const { return m_description; }
  Eigen::MatrixXd &CI() { return m_CI; }
  Eigen::VectorXd &ci() { return m_ci; }
  const Eigen::MatrixXd &CI() const { return m_CI; }
  const Eigen::VectorXd &ci() const { return m_ci; }

  size_t size() const { return m_constr_size; }
  size_t problem_size() const { return m_prb_size; }

  Eigen::VectorXd value(const Eigen::VectorXd &x) const {
    return m_CI * x + m_ci;
  }

  size_t violations(
      const Eigen::VectorXd &x,
      const double toll = std::numeric_limits<double>::epsilon()) const {
    return (value(x).array() < -toll).count();
  }
};

class InequalitySet {
private:
  using InequalityVectorRefConst =
      std::vector<std::reference_wrapper<const InequalityConstraint>>;

  InequalityVectorRefConst m_neq;
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
  InequalitySet() = delete;

  InequalitySet(const size_t problem_size)
      : m_prb_size(problem_size), m_CI(0, problem_size), m_ci(0) {}

  Eigen::MatrixXd CI() const { return m_CI; }
  Eigen::VectorXd ci() const { return m_ci; }

  size_t size() const { return m_ci.size(); }
  size_t problem_size() const { return m_prb_size; }

  void clear() {
    reset(0);
    m_neq.clear();
  }

  Eigen::VectorXd values(const Eigen::VectorXd &q) const {
    return CI() * q + ci();
  }

  void push_constraint(const InequalityConstraint &ic) { m_neq.push_back(ic); }

  void compute_set() {
    size_t neq_size = std::accumulate(
        m_neq.begin(), m_neq.end(), 0,
        [](const size_t acc, const InequalityConstraint &ineq) -> size_t {
          return acc + ineq.size();
        });

    this->reset(neq_size);
    size_t level = 0;

    for (size_t idx = 0; idx < m_neq.size(); ++idx) {
      m_CI.middleRows(level, m_neq.at(idx).get().size())
          << m_neq.at(idx).get().CI();
      m_ci.segment(level, m_neq.at(idx).get().size())
          << m_neq.at(idx).get().ci();
      level += m_neq.at(idx).get().size();
    }
  }

  size_t violations(
      const Eigen::VectorXd &x,
      const double toll = std::numeric_limits<double>::epsilon()) {
    return std::accumulate(
        m_neq.begin(), m_neq.end(), 0,
        [&x, &toll](const size_t acc, const InequalityConstraint &neq)
            -> size_t { return acc + neq.violations(x, toll); });
  }

  InequalityVectorRefConst which_violations(
      const Eigen::VectorXd &x,
      const double toll = std::numeric_limits<double>::epsilon()) {
    InequalityVectorRefConst v;
    v.reserve(size());
    std::copy_if(m_neq.begin(), m_neq.end(), std::back_inserter(v),
                 [&x, &toll](const InequalityConstraint &ineq) {
                   return ineq.violations(x, toll);
                 });
    return v;
  }

  int redundancies() {
    Eigen::BDCSVD<Eigen::MatrixXd> svd(m_CI, Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);
    return std::max(m_CI.rows(), m_CI.cols()) - svd.nonzeroSingularValues();
  }
};

using SolverStatus = eiquadprog::solvers::EiquadprogFast_status;

class SolverQP {
private:
  eiquadprog::solvers::EiquadprogFast m_solver;
  const size_t m_prb_dim;
  const Stack &m_stack;
  const EqualitySet &m_eq;
  const InequalitySet &m_ineq;

public:
  SolverQP() = delete;

  SolverQP(const size_t problem_size, const Stack &stack, const EqualitySet &eq,
           const InequalitySet &ineq)
      : m_prb_dim(problem_size), m_stack(stack), m_eq(eq), m_ineq(ineq) {}

  std::pair<Eigen::VectorXd, SolverStatus> solve() {
    m_solver.reset(m_prb_dim, m_eq.size(), m_ineq.size());
    Eigen::VectorXd sol;
    SolverStatus status =
        m_solver.solve_quadprog(m_stack.G(), m_stack.F(), m_eq.CE(), m_eq.ce(),
                                m_ineq.CI(), m_ineq.ci(), sol);
    return std::make_pair(sol, status);
  }
};

/// A dedicated task for smooth task switching (paper Eq. (33)):
///   min || x - x_ref ||^2
/// expressed using the wrapper convention residual = A*x + b:
///   A = I, b = -x_ref
class SolutionTrackingTask : public Task {
public:
  explicit SolutionTrackingTask(const size_t problem_size,
                                const std::string &description =
                                    "Solution tracking (smooth transition)")
      : Task(problem_size, problem_size, description) {
    A().setIdentity();
    W().setIdentity();
    b().setZero();
  }

  void set_reference(const Eigen::Ref<const Eigen::VectorXd> &x_ref) {
    if (static_cast<size_t>(x_ref.size()) != problem_size()) {
      throw std::invalid_argument(
          "SolutionTrackingTask: x_ref has wrong dimension");
    }
    b() = -x_ref;
  }
};

/// Implements task switching as in:
/// "A Distributed Processing Approach for Smooth Task Transitioning in Strict
/// Hierarchical Control", using only the task-transition mechanism.
///
/// Pattern:
///   1) Solve SoT B -> x_B
///   2) Inject last-priority tracking task into SoT A: min ||x - x_B||^2
///   3) Solve SoT A (with tracking) -> x_AB (smoothly approaches x_B)
class SmoothTaskTransitionSolver {
public:
  struct TransitionSolution {
    Eigen::VectorXd x_B;
    SolverStatus status_B{SolverStatus::EIQUADPROG_FAST_INFEASIBLE};

    Eigen::VectorXd x_AB;
    SolverStatus status_AB{SolverStatus::EIQUADPROG_FAST_INFEASIBLE};

    bool is_transitioning;
  };

private:
  const size_t m_prb_dim;

  Stack &m_stackA;
  EqualitySet &m_eqA;
  InequalitySet &m_ineqA;

  Stack &m_stackB;
  EqualitySet &m_eqB;
  InequalitySet &m_ineqB;

  SolutionTrackingTask m_tracking;
  double m_tracking_weight{1.0};

  // If set, overrides the computed injection level.
  // Default: lowest priority = stackA.level() + 1
  bool m_has_level_override{false};
  int m_tracking_level_override{0};

  bool m_solve_only_B{false};

public:
  SmoothTaskTransitionSolver(const size_t problem_size, Stack &stackA,
                             EqualitySet &eqA, InequalitySet &ineqA,
                             Stack &stackB, EqualitySet &eqB,
                             InequalitySet &ineqB)
      : m_prb_dim(problem_size), m_stackA(stackA), m_eqA(eqA),
        m_ineqA(ineqA), m_stackB(stackB), m_eqB(eqB), m_ineqB(ineqB),
        m_tracking(problem_size) {}

  void set_tracking_weight(const double w) { m_tracking_weight = w; }

  /// Force the level used to inject the tracking task into SoT A.
  /// If not set, the solver injects at (stackA.level() + 1).
  void set_tracking_level(const int level) {
    m_has_level_override = true;
    m_tracking_level_override = level;
  }

  /// Disable level override (use stackA.level()+1 again).
  void clear_tracking_level_override() { m_has_level_override = false; }

  void set_solve_only_B(bool b) { m_solve_only_B = b;}

  TransitionSolution solve() {
    TransitionSolution out;

    // 1) Solve SoT B
    SolverQP solverB(m_prb_dim, m_stackB, m_eqB, m_ineqB);
    auto [xB, stB] = solverB.solve();
    out.x_B = xB;
    out.status_B = stB;

    if(m_solve_only_B)
    {
        out.is_transitioning = false;
        return out;
    }

    // If B is infeasible/failed, fall back to solving A without injection.
    if (stB != SolverStatus::EIQUADPROG_FAST_OPTIMAL) {
      SolverQP solverA(m_prb_dim, m_stackA, m_eqA, m_ineqA);
      auto [xA, stA] = solverA.solve();
      out.x_AB = xA;
      out.status_AB = stA;
      out.is_transitioning = false;
      return out;
    }

    // 2) Solve A with injected tracking-to-xB
    auto out2 = solve_with_target(xB);
    out.x_AB = out2.x_AB;
    out.status_AB = out2.status_AB;
    out.is_transitioning = (out.x_AB - out.x_B).norm() < TRANSITIONING_ERROR ? false : true;
    return out;
  }

  /// Solve SoT A after injecting a last-priority "track x_target" task.
  /// This is useful if you compute x_target elsewhere (e.g., another thread).
  TransitionSolution solve_with_target(
      const Eigen::Ref<const Eigen::VectorXd> &x_target) {
    TransitionSolution out;
    out.x_B = x_target;
    out.status_B = SolverStatus::EIQUADPROG_FAST_OPTIMAL;

    if (static_cast<size_t>(x_target.size()) != m_prb_dim) {
      out.status_AB = SolverStatus::EIQUADPROG_FAST_INFEASIBLE;
      out.x_AB = Eigen::VectorXd::Zero(m_prb_dim);
      return out;
    }

    const size_t base_count = m_stackA.task_count();

    // Configure tracking task: residual = x - x_target
    m_tracking.set_reference(x_target);

    const int inject_level =
        m_has_level_override ? m_tracking_level_override : (m_stackA.level() + 1);

    // Inject as lowest-priority term
    m_stackA.insert_task(m_tracking, inject_level, m_tracking_weight);

    // Solve A (+tracking)
    SolverQP solverA(m_prb_dim, m_stackA, m_eqA, m_ineqA);
    auto [xAB, stAB] = solverA.solve();
    out.x_AB = xAB;
    out.status_AB = stAB;

    // Remove injected term to restore original SoT A
    m_stackA.pop_to_size(base_count);

    return out;
  }
};

} // namespace elastoplastic

#endif // ELASTOPLASTIC_CONTROLLER__SOT_HPP