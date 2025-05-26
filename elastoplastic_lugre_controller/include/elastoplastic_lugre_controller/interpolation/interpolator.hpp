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

  void expect_plan_in_frame(const std::string& s) { m_expected_reference_frame = s; }
  void start_plan(const rclcpp::Time& time);
  void end_plan() {m_state = State::Terminated;};

  bool is_empty() { return m_state == State::Empty; }
  bool is_ready() {return m_state == State::Available;}
  bool is_plan_started(){return m_state == State::Started;}
  bool is_plan_ended(){return m_state == State::Terminated;}

  Interpolator clone_with_transform(const geometry_msgs::msg::TransformStamped& tf);

  InterpolationResult interpolate(const rclcpp::Time& now, Eigen::Vector6d& acc, Eigen::Vector6d& twist, Eigen::Affine3d& pose);

private:
  Trajectory m_plan;
  State m_state;
  Spline3Coeff m_coeff_rot_spline;
  std::string m_expected_reference_frame;
};

} // namespace elastoplastic::utils::interpolation

#endif // ELASTOPLASTIC_CONTROLLER__INTERPOLATOR_HPP
