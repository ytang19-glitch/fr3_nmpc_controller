#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [f"fr3_joint{index}" for index in range(1, 8)]


class MinimumJerkPublisher(Node):
    def __init__(self) -> None:
        super().__init__("minimum_jerk_reference_publisher")
        self.declare_parameter("joint_index", 0)
        self.declare_parameter("displacement", 0.02)
        self.declare_parameter("duration", 8.0)
        self.declare_parameter("samples", 81)
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("reference_topic", "/nmpc/reference_trajectory")

        self.joint_index = int(self.get_parameter("joint_index").value)
        self.displacement = float(self.get_parameter("displacement").value)
        self.duration = float(self.get_parameter("duration").value)
        self.samples = int(self.get_parameter("samples").value)

        if not 0 <= self.joint_index < len(JOINT_NAMES):
            raise ValueError("joint_index must be between 0 and 6")
        if not math.isfinite(self.displacement) or abs(self.displacement) > 0.10:
            raise ValueError("displacement must be finite and no larger than 0.10 rad")
        if not math.isfinite(self.duration) or self.duration < 2.0:
            raise ValueError("duration must be finite and at least 2.0 s")
        if self.samples < 3 or self.samples > 1001:
            raise ValueError("samples must be between 3 and 1001")

        state_topic = str(self.get_parameter("state_topic").value)
        reference_topic = str(self.get_parameter("reference_topic").value)
        self.publisher = self.create_publisher(JointTrajectory, reference_topic, 1)
        self.subscription = self.create_subscription(
            JointState, state_topic, self.state_callback, 10
        )
        self.published = False
        self.get_logger().info("Waiting for a complete FR3 JointState message.")

    def state_callback(self, message: JointState) -> None:
        if self.published:
            return

        index_by_name = {name: index for index, name in enumerate(message.name)}
        if any(name not in index_by_name for name in JOINT_NAMES):
            return
        if len(message.position) < len(message.name):
            return

        q0 = [message.position[index_by_name[name]] for name in JOINT_NAMES]
        if not all(math.isfinite(value) for value in q0):
            self.get_logger().error("JointState contains a non-finite position.")
            return

        trajectory = self.build_trajectory(q0)
        self.publisher.publish(trajectory)
        self.published = True
        self.get_logger().info(
            "Published a dry-run minimum-jerk reference: joint=%d, delta=%.4f rad, "
            "duration=%.2f s. This topic does not command the robot.",
            self.joint_index,
            self.displacement,
            self.duration,
        )

    def build_trajectory(self, q0: list[float]) -> JointTrajectory:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = JOINT_NAMES

        for sample_index in range(self.samples):
            time_seconds = self.duration * sample_index / (self.samples - 1)
            s = time_seconds / self.duration
            blend = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
            blend_rate = (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / self.duration
            blend_acceleration = (
                60.0 * s - 180.0 * s**2 + 120.0 * s**3
            ) / self.duration**2

            point = JointTrajectoryPoint()
            point.positions = list(q0)
            point.velocities = [0.0] * len(JOINT_NAMES)
            point.accelerations = [0.0] * len(JOINT_NAMES)
            point.positions[self.joint_index] += self.displacement * blend
            point.velocities[self.joint_index] = self.displacement * blend_rate
            point.accelerations[self.joint_index] = (
                self.displacement * blend_acceleration
            )

            whole_seconds = int(time_seconds)
            point.time_from_start.sec = whole_seconds
            point.time_from_start.nanosec = int(
                round((time_seconds - whole_seconds) * 1.0e9)
            )
            if point.time_from_start.nanosec == 1_000_000_000:
                point.time_from_start.sec += 1
                point.time_from_start.nanosec = 0
            message.points.append(point)

        return message


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MinimumJerkPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
