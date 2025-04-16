#ifndef ELASTOPLASTIC_CONTROLLER__INTERPOLATOR_HPP
#define ELASTOPLASTIC_CONTROLLER__INTERPOLATOR_HPP

#include "interpolation.hpp"
#include "interpolation_data_types.hpp"
#include "operators.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/cartesian_trajectory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace elastoplastic::utils::interpolation {

class Interpolator {
public:

  enum class State
  {
    Empty,
    Available,
    Started,
    Terminated
  };

  enum class InterpolationResult
  {
    OK,
    InterpolatorNotStarted
  };

  Interpolator(const Trajectory& plan);
  Interpolator();

  void set_plan(const Trajectory& plan);
  Trajectory get_plan()
  {
    return m_plan;
  }
  static Interpolator from_msg(const moveit_msgs::msg::CartesianTrajectory& trj);

  void start_plan(const rclcpp::Time& t_time);
  void end_plan() {m_state = State::Terminated;};

  bool is_empty() {return m_state == State::Empty;}
  bool is_ready() {return m_state == State::Available;}
  bool is_plan_started(){return m_state == State::Started;}
  bool is_plan_ended(){return m_state == State::Terminated;}

  Interpolator clone_with_transform(const geometry_msgs::msg::TransformStamped& t_tf);

  InterpolationResult interpolate(const rclcpp::Time& t_t, Eigen::Vector6d& o_twist, Eigen::Affine3d& o_pose);

private:
  Trajectory m_plan;
  State m_state;
  Spline3Coeff m_coeff_rot_spline;
};

} // namespace elastoplastic::utils::interpolation

#endif // ELASTOPLASTIC_CONTROLLER__INTERPOLATOR_HPP
