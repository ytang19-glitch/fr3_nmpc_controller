#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "fr3_nmpc_controller/reference_trajectory.hpp"

namespace fr3_nmpc_controller
{

class ReferenceMonitorNode : public rclcpp::Node
{
public:
  ReferenceMonitorNode()
  : Node("fr3_nmpc_reference_monitor")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
        "fr3_joint5", "fr3_joint6", "fr3_joint7"});
    const auto state_topic = declare_parameter<std::string>("state_topic", "/joint_states");
    const auto reference_topic =
      declare_parameter<std::string>("reference_topic", "/nmpc/reference_trajectory");
    const double sample_rate_hz = declare_parameter<double>("sample_rate_hz", 100.0);

    if (joint_names_.empty()) {
      throw std::runtime_error("joint_names parameter must not be empty");
    }
    if (!std::isfinite(sample_rate_hz) || sample_rate_hz <= 0.0 || sample_rate_hz > 1000.0) {
      throw std::runtime_error("sample_rate_hz must be in (0, 1000]");
    }

    state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      state_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        measured_state_ = *message;
        have_state_ = true;
      });

    trajectory_subscription_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      reference_topic, rclcpp::QoS(1).reliable(),
      [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message) {
        std::string error;
        ReferenceTrajectory candidate;
        if (!candidate.setTrajectory(*message, joint_names_, error)) {
          RCLCPP_ERROR(get_logger(), "Rejected reference trajectory: %s", error.c_str());
          return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        trajectory_ = std::move(candidate);
        trajectory_start_ = now();
        have_trajectory_ = true;
        RCLCPP_INFO(
          get_logger(), "Accepted %.3f s reference trajectory.", trajectory_.duration());
      });

    reference_publisher_ =
      create_publisher<sensor_msgs::msg::JointState>("/nmpc/reference_state", 10);
    error_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/nmpc/tracking_error", 10);

    const auto timer_period = std::chrono::duration<double>(1.0 / sample_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&ReferenceMonitorNode::update, this));

    RCLCPP_INFO(
      get_logger(),
      "Dry-run reference monitor started. It does not publish torque or position commands.");
  }

private:
  void update()
  {
    sensor_msgs::msg::JointState measured;
    ReferenceSample reference;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_state_ || !have_trajectory_) {
        return;
      }
      measured = measured_state_;
      const double elapsed = (now() - trajectory_start_).seconds();
      reference = trajectory_.sample(elapsed);
    }

    std::unordered_map<std::string, std::size_t> measured_index;
    for (std::size_t index = 0; index < measured.name.size(); ++index) {
      measured_index.emplace(measured.name[index], index);
    }

    sensor_msgs::msg::JointState reference_message;
    reference_message.header.stamp = now();
    reference_message.name = joint_names_;
    reference_message.position = reference.position;
    reference_message.velocity = reference.velocity;
    reference_message.effort = reference.acceleration;

    std_msgs::msg::Float64MultiArray error_message;
    error_message.data.resize(joint_names_.size());

    for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
      const auto found = measured_index.find(joint_names_[joint]);
      if (found == measured_index.end() || found->second >= measured.position.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Measured JointState is missing one or more configured FR3 joints.");
        return;
      }
      error_message.data[joint] = reference.position[joint] - measured.position[found->second];
    }

    reference_publisher_->publish(reference_message);
    error_publisher_->publish(error_message);

    if (reference.finished && !completion_reported_) {
      const auto maximum_error = std::max_element(
        error_message.data.begin(), error_message.data.end(),
        [](double left, double right) {return std::abs(left) < std::abs(right);});
      RCLCPP_INFO(
        get_logger(), "Reference finished; maximum final joint error is %.6f rad.",
        maximum_error == error_message.data.end() ? 0.0 : std::abs(*maximum_error));
      completion_reported_ = true;
    } else if (!reference.finished) {
      completion_reported_ = false;
    }
  }

  std::vector<std::string> joint_names_;
  std::mutex mutex_;
  sensor_msgs::msg::JointState measured_state_;
  ReferenceTrajectory trajectory_;
  rclcpp::Time trajectory_start_{0, 0, RCL_ROS_TIME};
  bool have_state_{false};
  bool have_trajectory_{false};
  bool completion_reported_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fr3_nmpc_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fr3_nmpc_controller::ReferenceMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
