#ifndef ELASTOPLASTIC_CONTROLLER__UTILS_HPP
#define ELASTOPLASTIC_CONTROLLER__UTILS_HPP

#include "Eigen/Core"
#include "elastoplastic_lugre_controller/elastoplastic_variable_model.hpp"
#include "elastoplastic_parameters.hpp"

// Helpers to get unique names when using __LINE__
#define _CONCAT(a, b) a##b
#define UNIQUE_NAME(base) _CONCAT(base, __LINE__)

// Throttled INFO with a call-count
#define LOG_ERROR_THROTTLE_COUNT(logger, clock, period_sec, message)                                                             \
  do {                                                                                                                           \
    /* static variables per call-site */                                                                                         \
    static rclcpp::Time UNIQUE_NAME(_last_time_) = rclcpp::Time(0, 0, RCL_ROS_TIME);                                             \
    static size_t UNIQUE_NAME(_call_count_) = 0;                                                                                 \
                                                                                                                                 \
    /* increment counter */                                                                                                      \
    UNIQUE_NAME(_call_count_)++;                                                                                                 \
                                                                                                                                 \
    /* current time */                                                                                                           \
    auto UNIQUE_NAME(_now_) = clock->now();                                                                                      \
                                                                                                                                 \
    /* if enough time has elapsed… */                                                                                          \
    if ((UNIQUE_NAME(_now_) - UNIQUE_NAME(_last_time_)).seconds() >= (period_sec)) {                                             \
      /* log the count + your message */                                                                                         \
      RCLCPP_ERROR_STREAM((logger), "Throttled over " << (period_sec)                                                            \
                                                      << "s: "                                                                   \
                                                         "("                                                                     \
                                                      << UNIQUE_NAME(_call_count_) << " calls) " << message);                    \
                                                                                                                                 \
      /* reset */                                                                                                                \
      UNIQUE_NAME(_last_time_) = UNIQUE_NAME(_now_);                                                                             \
      UNIQUE_NAME(_call_count_) = 0;                                                                                             \
    }                                                                                                                            \
  } while (false)


namespace Eigen {
using Vector6d = Vector<double,6>;
using Vector12d = Vector<double, 12>;
using Matrix12d = Matrix<double, 12, 12>;
using Matrix126d = Matrix<double, 12, 6>;
using Matrix612d = Matrix<double, 6, 12>;
using Matrix12Xd = Matrix<double, 12, Eigen::Dynamic>;
}

namespace elastoplastic::utils {

constexpr static double K_ABS_EPSILON{1e-12};
constexpr static double K_REL_EPSILON{1e-8};

struct Logistic {
  double max;
  double slope;
  Eigen::Array3d inflection;

  double get(const Eigen::Array3d& v) { return (max / (1 + Eigen::exp(slope * (v.abs() - inflection)))).minCoeff(); }
};

/* == LIE OPERATORS == */
inline Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
  Eigen::Matrix3d W;
  W << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
  return W;
}

inline Eigen::Vector3d vee(const Eigen::Matrix3d& W) { return Eigen::Vector3d({W(2, 1), W(0, 2), W(1, 0)}); }

inline Eigen::Affine3d lieExp(const Eigen::Matrix<double, 6, 1>& xi) {
  const Eigen::Vector3d omega = xi.template head<3>();
  const Eigen::Vector3d v = xi.template tail<3>();
  const double theta = omega.norm();

  Eigen::Matrix3d R;
  Eigen::Matrix3d J; // left‐Jacobian

  // compute rotation and left‐Jacobian
  if (theta < 1e-10) {
    // small‐angle approximations
    Eigen::Matrix3d W = hat(omega);
    R = Eigen::Matrix3d::Identity() + W + 0.5 * (W * W);
    J = Eigen::Matrix3d::Identity() + 0.5 * W + (1.0 / 6.0) * (W * W);
  } else {
    Eigen::Matrix3d W = hat(omega);
    Eigen::Matrix3d W2 = W * W;
    const double s = std::sin(theta);
    const double c = std::cos(theta);

    // Rodrigues’ formula
    R = Eigen::Matrix3d::Identity() + (s / theta) * W + ((1 - c) / (theta * theta)) * W2;

    // left‐Jacobian J = I + (1−cosθ)/θ² W + (θ−sinθ)/θ³ W²
    J = Eigen::Matrix3d::Identity() + ((1 - c) / (theta * theta)) * W + ((theta - s) / (theta * theta * theta)) * W2;
  }

  // build the final transform
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = R;
  T.translation() = J * v;
  return T;
}

/* == FLOATS == */
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


// Eigen::MatrixXd& regularize(Eigen::MatrixXd& m) {
//   m += Eigen::MatrixXd::Identity(m.rows(), m.cols()) * K_REL_EPSILON * m.trace() / m.cols();
//   return m;
// }


inline Eigen::Vector3d base_velocity_from_twist(const Eigen::Vector6d& p_w) { return p_w({0, 1, 5}); }


inline Eigen::Vector6d twist_from_base_velocity(const Eigen::Vector3d& p_v) {
  return Eigen::Vector6d {p_v(0), p_v(1), 0, 0, 0, p_v(2)};
}

/**
 * @brief Convert from Rototranslation matrix to 6d vector. Angular convention: rotation vector (= angle * axis)
 * @param m
 * @return
 */
inline Eigen::Vector6d vector_from_affine(const Eigen::Affine3d& m) {
  Eigen::AngleAxisd aa(m.linear());
  Eigen::Vector6d v;
  v << m.translation(), aa.angle() * aa.axis();
  return v;
}

/**
 * @brief Convert from 6d vector to Rototranslation matrix.
 *        Angular convention: rotation vector (= angle * axis)
 * @param v  6d vector [tx, ty, tz,  ωx,  ωy,  ωz] where ω = angle*axis
 * @return   Affine3d with translation and rotation
 */
inline Eigen::Affine3d affine_from_vector(const Eigen::Vector6d& v) {
  // Extract rotation-vector
  Eigen::Vector3d rot = v.tail<3>();
  double angle = rot.norm();

  // Start with identity, set translation
  Eigen::Affine3d m = Eigen::Affine3d::Identity();
  m.translation() = v.head<3>();

  // If there's a non-zero rotation, build the AngleAxis
  if (angle > std::numeric_limits<double>::epsilon()) {
    Eigen::Vector3d axis = rot / angle;
    Eigen::AngleAxisd aa(angle, axis);
    m.linear() = aa.toRotationMatrix();
  }
  // else leave m.linear() == identity

  return m;
}

/**
 * \brief One Runge–Kutta-4 integration step.
 *
 * Template parameters are deduced automatically.
 *
 * @tparam  Func  any callable object with signature  T (const T& x, const T& u)
 * @tparam  T     the state (and input) type – can be a scalar or any Eigen vector/matrix
 *
 * @param   fun   derivative function  dx/dt = fun(x,u)
 * @param   x     current state
 * @param   u     current input (held constant over the step)
 * @param   dt    time step [s]
 *
 * @return  state after one RK4 step of length dt
 */
template <typename Func, typename T> constexpr T rk4(Func&& fun, const T& x, const T& u, double dt) {
  const T k1 = std::forward<Func>(fun)(x, u);
  const T k2 = std::forward<Func>(fun)(x + (dt * 0.5) * k1, u);
  const T k3 = std::forward<Func>(fun)(x + (dt * 0.5) * k2, u);
  const T k4 = std::forward<Func>(fun)(x + dt * k3, u);

  return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/* ──────────────────────────────────────────────────────────────────────────────
   A lightweight “state” wrapper with the arithmetic we need
 ──────────────────────────────────────────────────────────────────────────── */
template <typename T> struct Rk4State2 {
  T x; // position (integral of velocity)
  T v; // velocity (integral of acceleration)

  /* algebra needed by RK4 -------------------------------------------------- */
  friend constexpr Rk4State2 operator+(const Rk4State2& a, const Rk4State2& b) { return {a.x + b.x, a.v + b.v}; }

  friend constexpr Rk4State2 operator*(double k, const Rk4State2& s) { return {k * s.x, k * s.v}; }
};

/* ──────────────────────────────────────────────────────────────────────────────
   Double integration RK4 step
 ──────────────────────────────────────────────────────────────────────────── */
/**
 * \brief One 4th-order Runge–Kutta step that advances both **position** and
 * **velocity** when you only have an acceleration function.
 *
 * The acceleration functor may depend on position, velocity and an external
 * input held constant over the step.
 *
 * @tparam  Acc   callable  a = Acc(x, v, u)
 * @tparam  T     arithmetic or Eigen-like type
 *
 * @param   acc   acceleration function
 * @param   x     current position
 * @param   v     current velocity
 * @param   u     constant input over the step (same type as x)
 * @param   dt    time step [s]
 *
 * @return  pair  {x_next, v_next}
 */
template <typename Acc, typename T>
constexpr std::pair<T, T> rk4_double(Acc&& acc, // a = acc(x,v,u)
                                     const T& x, const T& v, const T& u, double dt) {
  using S = Rk4State2<T>;

  /* first-order system:  d/dt [x;v] = [v; a(x,v,u)] */
  auto sys = [&](const S& s, const T& u_in) -> S { return {s.v, std::forward<Acc>(acc)(s.x, s.v, u_in)}; };

  const S s0{x, v};

  const S k1 = sys(s0, u);
  const S k2 = sys(s0 + 0.5 * dt * k1, u);
  const S k3 = sys(s0 + 0.5 * dt * k2, u);
  const S k4 = sys(s0 + dt * k3, u);

  const S s_next = s0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

  return {s_next.x, s_next.v};
}

inline void get_frame_distance(const Eigen::Affine3d& T_wa, const Eigen::Affine3d& T_wb, Eigen::Ref<Eigen::Vector6d> v) {
  v.head<3>() = T_wa.translation() - T_wb.translation();
  Eigen::AngleAxisd aa(T_wa.linear().transpose() * T_wb.linear());
  v.tail<3>() = T_wa.linear() * (aa.angle() * aa.axis());
}

inline Eigen::Vector6d get_frame_distance(const Eigen::Affine3d& T_wa, const Eigen::Affine3d& T_wb) {
  Eigen::Vector6d v;
  get_frame_distance(T_wa, T_wb, v);
  return v;
}

inline ElastoplasticModelData get_model_data(const elastoplastic_controller::Params& params, const double update_rate) {
  ElastoplasticModelData data;
  std::copy(params.impedance.inertia.begin(), params.impedance.inertia.end(), data.inertia_inv.diagonal().begin());
  std::copy(params.impedance.k.begin(), params.impedance.k.end(), data.k.diagonal().begin());
  std::copy(params.impedance.d.begin(), params.impedance.d.end(), data.d.diagonal().begin());
  data.z_max = params.impedance.z_max;
  data.z_start = params.impedance.z_start;
  data.z_kmax = params.impedance.z_kmax;
  data.enable_axis = params.impedance.enable_axis;
  data.buffer_size = static_cast<size_t>(params.impedance.reset.time * update_rate);
  data.reset_threshold = params.impedance.reset.threshold;
  return data;
}

const std::string MOBILE_BASE_URDF = R"(<?xml version='1.0'?>
<robot name='base'>
<link name='x_base'/>
<link name='y_base'/>
<link name='rz_base'/>
<link name='mount_link'/>
<joint name='move_x' type='prismatic'>
  <parent link='x_base'/>
  <child link='y_base'/>
  <origin xyz='0 0 0'/>
  <axis xyz='1 0 0'/>
  <limit lower='-1e10' upper='1e10' effort='1e10' velocity='1e10'/>
</joint>
<joint name='move_y' type='prismatic'>
  <parent link='y_base'/>
  <child link='rz_base'/>
  <origin xyz='0 0 0'/>
  <axis xyz='0 1 0'/>
  <limit lower='-1e10' upper='1e10' effort='1e10' velocity='1e10'/>
</joint>
<joint name='rot_z' type='revolute'>
  <parent link='rz_base'/>
  <child link='mount_link'/>
  <origin xyz='0 0 0'/>
  <axis xyz='0 0 1'/>
  <limit lower='-1e10' upper='1e10' effort='1e10' velocity='1e10'/>
</joint>
</robot>
    )";

} // namespace elastoplastic::utils

#endif // ELASTOPLASTIC_CONTROLLER__UTILS_HPP
