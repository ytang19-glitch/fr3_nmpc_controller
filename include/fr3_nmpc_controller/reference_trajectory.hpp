#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace fr3_nmpc_controller
{

struct ReferenceSample
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  bool finished{false};
};

class ReferenceTrajectory
{
public:
  bool setTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<std::string> & expected_joint_names,
    std::string & error);

  [[nodiscard]] bool empty() const;
  [[nodiscard]] double duration() const;
  [[nodiscard]] ReferenceSample sample(double time_from_start_seconds) const;

private:
  trajectory_msgs::msg::JointTrajectory trajectory_;
  std::size_t dof_{0};
};

}  // namespace fr3_nmpc_controller
