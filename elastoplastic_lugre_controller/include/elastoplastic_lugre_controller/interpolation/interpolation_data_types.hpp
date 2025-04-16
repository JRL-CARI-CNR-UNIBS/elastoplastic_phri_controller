#ifndef ELASTOPLASTIC_CONTROLLER__DATA_TYPES_HPP
#define ELASTOPLASTIC_CONTROLLER__DATA_TYPES_HPP

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>

#include <rdyn_core/internal/types.h>

namespace elastoplastic::utils::interpolation {
class Interpolator;
typedef std::shared_ptr<Interpolator> InterpolatorPtr;

  struct Trajectory{
    std::vector<Eigen::Affine3d> pose;
    std::vector<Eigen::Vector6d> twist;
    std::vector<Eigen::Vector6d> acc;
    std::vector<rclcpp::Time> time;
    rclcpp::Time start;
    void clear()
    {
      pose.clear();
      twist.clear();
      acc.clear();
      time.clear();
    }
    void resize(long n)
    {
      pose.resize(n);
      twist.resize(n);
      acc.resize(n);
      time.resize(n);
    }
    size_t size()
    {
      return time.size();
    }
  };
  typedef std::shared_ptr<Trajectory> TrajectoryPtr;
  typedef Trajectory* TrajectoryRawPtr;
  } // namespace elastoplastic::utils::interpolation

#endif // ELASTOPLASTIC_CONTROLLER__DATA_TYPES_HPP
