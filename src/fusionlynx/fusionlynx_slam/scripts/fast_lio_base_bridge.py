#!/usr/bin/env python3
"""Normalize FAST-LIO's lidar pose into an isolated base_link odometry path."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def _quaternion_multiply(
    lhs: tuple[float, float, float, float],
    rhs: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quaternion_inverse(
    value: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, w = value
    norm_sq = x * x + y * y + z * z + w * w
    if norm_sq <= 1.0e-12:
        raise ValueError("cannot invert a zero quaternion")
    return (-x / norm_sq, -y / norm_sq, -z / norm_sq, w / norm_sq)


def _rotate(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, vector_quaternion),
        _quaternion_inverse(quaternion),
    )
    return rotated[0], rotated[1], rotated[2]


def _compose(
    lhs_translation: tuple[float, float, float],
    lhs_rotation: tuple[float, float, float, float],
    rhs_translation: tuple[float, float, float],
    rhs_rotation: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    rotated_rhs = _rotate(lhs_rotation, rhs_translation)
    return (
        tuple(lhs_translation[index] + rotated_rhs[index] for index in range(3)),
        _quaternion_multiply(lhs_rotation, rhs_rotation),
    )


class FastLioBaseBridge(Node):
    def __init__(self) -> None:
        super().__init__("fast_lio_base_bridge")
        self.declare_parameter("input_topic", "/fast_lio/odom")
        self.declare_parameter("output_topic", "/fast_lio/base_odom")
        self.declare_parameter("input_world_frame", "camera_init")
        self.declare_parameter("input_child_frame", "lidar_link")
        self.declare_parameter("output_odom_frame", "fast_lio_odom")
        self.declare_parameter("output_base_frame", "base_link")
        self.declare_parameter("tf_lookup_timeout_sec", 0.05)
        self.declare_parameter("publish_tf", True)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.input_world_frame = self.get_parameter("input_world_frame").value
        self.input_child_frame = self.get_parameter("input_child_frame").value
        self.output_odom_frame = self.get_parameter("output_odom_frame").value
        self.output_base_frame = self.get_parameter("output_base_frame").value
        self.tf_lookup_timeout = Duration(
            seconds=float(self.get_parameter("tf_lookup_timeout_sec").value)
        )
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self.subscription = self.create_subscription(
            Odometry,
            self.input_topic,
            self._handle_odom,
            20,
        )

        self.alignment_translation: tuple[float, float, float] | None = None
        self.alignment_rotation: tuple[float, float, float, float] | None = None
        self.last_stamp_ns = 0
        self.last_tf_error_ns = 0
        self.get_logger().info(
            f"FAST-LIO base validation bridge: {self.input_topic} "
            f"({self.input_world_frame} -> {self.input_child_frame}) -> "
            f"{self.output_topic} ({self.output_odom_frame} -> "
            f"{self.output_base_frame}); base extrinsic is read from TF"
        )

    def _handle_odom(self, message: Odometry) -> None:
        if (
            self.input_world_frame
            and message.header.frame_id
            and message.header.frame_id != self.input_world_frame
        ):
            self.get_logger().error(
                f"Rejecting FAST-LIO odometry frame "
                f"'{message.header.frame_id}', expected "
                f"'{self.input_world_frame}'"
            )
            return
        if (
            self.input_child_frame
            and message.child_frame_id
            and message.child_frame_id != self.input_child_frame
        ):
            self.get_logger().error(
                f"Rejecting FAST-LIO child frame '{message.child_frame_id}', "
                f"expected '{self.input_child_frame}'"
            )
            return

        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp_ns <= 0 or stamp_ns <= self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns

        raw_translation = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        raw_rotation = (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        )
        try:
            base_from_lidar_tf = self.tf_buffer.lookup_transform(
                self.output_base_frame,
                self.input_child_frame,
                rclpy.time.Time.from_msg(message.header.stamp),
                self.tf_lookup_timeout,
            )
        except TransformException as error:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_error_ns > 2_000_000_000:
                self.get_logger().warn(
                    f"Waiting for TF {self.output_base_frame} <- "
                    f"{self.input_child_frame} at stamp "
                    f"{message.header.stamp.sec}."
                    f"{message.header.stamp.nanosec:09d}: {error}"
                )
                self.last_tf_error_ns = now_ns
            return

        base_to_lidar_translation = (
            base_from_lidar_tf.transform.translation.x,
            base_from_lidar_tf.transform.translation.y,
            base_from_lidar_tf.transform.translation.z,
        )
        base_to_lidar_rotation = (
            base_from_lidar_tf.transform.rotation.x,
            base_from_lidar_tf.transform.rotation.y,
            base_from_lidar_tf.transform.rotation.z,
            base_from_lidar_tf.transform.rotation.w,
        )
        # lookup_transform(base, lidar) returns T_base_lidar.  The pose
        # composition below needs T_lidar_base, while velocity components
        # reported in the lidar child frame need the forward rotation.
        lidar_to_base_rotation = _quaternion_inverse(base_to_lidar_rotation)
        lidar_to_base_translation = _rotate(
            lidar_to_base_rotation,
            tuple(-value for value in base_to_lidar_translation),
        )
        lidar_translation, lidar_rotation = _compose(
            raw_translation,
            raw_rotation,
            lidar_to_base_translation,
            lidar_to_base_rotation,
        )

        if self.alignment_translation is None:
            self.alignment_translation = lidar_translation
            self.alignment_rotation = lidar_rotation

        assert self.alignment_rotation is not None
        assert self.alignment_translation is not None
        inverse_alignment_rotation = _quaternion_inverse(self.alignment_rotation)
        relative_translation = tuple(
            lidar_translation[index] - self.alignment_translation[index]
            for index in range(3)
        )
        output_translation = _rotate(inverse_alignment_rotation, relative_translation)
        output_rotation = _quaternion_multiply(
            inverse_alignment_rotation,
            lidar_rotation,
        )

        output = Odometry()
        output.header = message.header
        output.header.frame_id = self.output_odom_frame
        output.child_frame_id = self.output_base_frame
        output.pose = message.pose
        output.pose.pose.position.x = output_translation[0]
        output.pose.pose.position.y = output_translation[1]
        output.pose.pose.position.z = output_translation[2]
        (
            output.pose.pose.orientation.x,
            output.pose.pose.orientation.y,
            output.pose.pose.orientation.z,
            output.pose.pose.orientation.w,
        ) = output_rotation
        output.twist = message.twist
        output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = _rotate(
            base_to_lidar_rotation,
            (
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            ),
        )
        output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = _rotate(
            base_to_lidar_rotation,
            (
                message.twist.twist.angular.x,
                message.twist.twist.angular.y,
                message.twist.twist.angular.z,
            ),
        )
        self.publisher.publish(output)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = output.child_frame_id
            transform.transform.translation.x = output_translation[0]
            transform.transform.translation.y = output_translation[1]
            transform.transform.translation.z = output_translation[2]
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ) = output_rotation
            self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = FastLioBaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
