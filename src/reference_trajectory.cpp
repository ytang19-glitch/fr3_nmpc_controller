#include "fr3_nmpc_controller/reference_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace fr3_nmpc_controller
{
namespace
{

double seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + 1.0e-9 * static_cast<double>(duration.nanosec);
}

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

}  // namespace

bool ReferenceTrajectory::setTrajectory(
  const trajectory_msgs::msg::JointTrajectory & input,
  const std::vector<std::string> & expected_joint_names,
  std::string & error)
{
  if (expected_joint_names.empty()) {
    error = "Expected joint list is empty.";
    return false;
  }
  if (input.points.size() < 2U) {
    error = "Trajectory must contain at least two points.";
    return false;
  }
  if (input.joint_names.size() != expected_joint_names.size()) {
    error = "Trajectory joint count does not match the configured joint count.";
    return false;
  }

  std::unordered_map<std::string, std::size_t> source_index;
  for (std::size_t i = 0; i < input.joint_names.size(); ++i) {
    if (!source_index.emplace(input.joint_names[i], i).second) {
      error = "Trajectory contains a duplicate joint name: " + input.joint_names[i];
      return false;
    }
  }

  trajectory_msgs::msg::JointTrajectory reordered;
  reordered.header = input.header;
  reordered.joint_names = expected_joint_names;
  reordered.points.reserve(input.points.size());

  double previous_time = -std::numeric_limits<double>::infinity();
  for (std::size_t point_index = 0; point_index < input.points.size(); ++point_index) {
    const auto & source = input.points[point_index];
    const double point_time = seconds(source.time_from_start);
    if (!std::isfinite(point_time) || point_time < 0.0 || point_time <= previous_time) {
      error = "Trajectory time_from_start must be finite, non-negative, and strictly increasing.";
      return false;
    }
    previous_time = point_time;

    if (source.positions.size() != input.joint_names.size() ||
      source.velocities.size() != input.joint_names.size())
    {
      std::ostringstream stream;
      stream << "Point " << point_index
             << " must contain positions and velocities for every joint.";
      error = stream.str();
      return false;
    }
    if (!source.accelerations.empty() && source.accelerations.size() != input.joint_names.size()) {
      error = "Acceleration arrays must be empty or contain one value per joint.";
      return false;
    }
    if (!allFinite(source.positions) || !allFinite(source.velocities) ||
      (!source.accelerations.empty() && !allFinite(source.accelerations)))
    {
      error = "Trajectory contains NaN or infinite values.";
      return false;
    }

    trajectory_msgs::msg::JointTrajectoryPoint target;
    target.time_from_start = source.time_from_start;
    target.positions.resize(expected_joint_names.size());
    target.velocities.resize(expected_joint_names.size());
    target.accelerations.resize(expected_joint_names.size(), 0.0);

    for (std::size_t joint = 0; joint < expected_joint_names.size(); ++joint) {
      const auto found = source_index.find(expected_joint_names[joint]);
      if (found == source_index.end()) {
        error = "Missing expected joint: " + expected_joint_names[joint];
        return false;
      }
      const std::size_t source_joint = found->second;
      target.positions[joint] = source.positions[source_joint];
      target.velocities[joint] = source.velocities[source_joint];
      if (!source.accelerations.empty()) {
        target.accelerations[joint] = source.accelerations[source_joint];
      }
    }
    reordered.points.push_back(std::move(target));
  }

  trajectory_ = std::move(reordered);
  dof_ = expected_joint_names.size();
  error.clear();
  return true;
}

bool ReferenceTrajectory::empty() const
{
  return trajectory_.points.empty();
}

double ReferenceTrajectory::duration() const
{
  if (empty()) {
    return 0.0;
  }
  return seconds(trajectory_.points.back().time_from_start);
}

ReferenceSample ReferenceTrajectory::sample(double query_time) const
{
  ReferenceSample result;
  if (empty()) {
    return result;
  }

  result.position.resize(dof_);
  result.velocity.resize(dof_);
  result.acceleration.resize(dof_);

  if (query_time <= seconds(trajectory_.points.front().time_from_start)) {
    const auto & first = trajectory_.points.front();
    result.position = first.positions;
    result.velocity = first.velocities;
    result.acceleration = first.accelerations;
    return result;
  }

  if (query_time >= duration()) {
    const auto & last = trajectory_.points.back();
    result.position = last.positions;
    result.velocity.assign(dof_, 0.0);
    result.acceleration.assign(dof_, 0.0);
    result.finished = true;
    return result;
  }

  const auto upper = std::upper_bound(
    trajectory_.points.begin(), trajectory_.points.end(), query_time,
    [](double time, const trajectory_msgs::msg::JointTrajectoryPoint & point) {
      return time < seconds(point.time_from_start);
    });
  const auto lower = std::prev(upper);

  const double t0 = seconds(lower->time_from_start);
  const double t1 = seconds(upper->time_from_start);
  const double dt = t1 - t0;
  const double s = std::clamp((query_time - t0) / dt, 0.0, 1.0);
  const double s2 = s * s;
  const double s3 = s2 * s;

  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;

  for (std::size_t joint = 0; joint < dof_; ++joint) {
    const double q0 = lower->positions[joint];
    const double q1 = upper->positions[joint];
    const double v0 = lower->velocities[joint];
    const double v1 = upper->velocities[joint];

    result.position[joint] = h00 * q0 + h10 * dt * v0 + h01 * q1 + h11 * dt * v1;
    result.velocity[joint] =
      ((6.0 * s2 - 6.0 * s) * q0 +
      (3.0 * s2 - 4.0 * s + 1.0) * dt * v0 +
      (-6.0 * s2 + 6.0 * s) * q1 +
      (3.0 * s2 - 2.0 * s) * dt * v1) / dt;
    result.acceleration[joint] =
      ((12.0 * s - 6.0) * q0 +
      (6.0 * s - 4.0) * dt * v0 +
      (-12.0 * s + 6.0) * q1 +
      (6.0 * s - 2.0) * dt * v1) / (dt * dt);
  }

  return result;
}

}  // namespace fr3_nmpc_controller
