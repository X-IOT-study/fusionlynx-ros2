#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from motioncore_msgs.msg import EncoderRaw


class EncoderJointStatePublisher(Node):
    def __init__(self):
        super().__init__("encoder_joint_state_publisher")

        self.declare_parameter(
            "joint_names",
            [
                "base_link_to_wheel_fl_link",
                "base_link_to_wheel_fr_link",
                "base_link_to_wheel_rl_link",
                "base_link_to_wheel_rr_link",
            ],
        )
        self.declare_parameter("encoder_topic", "/motioncore/encoder_raw")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("ticks_per_rev", 2464)
        self.declare_parameter("encoder_sign", [1, 1, 1, 1])
        self.declare_parameter("zero_offset_ticks", [0, 0, 0, 0])

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.encoder_topic = self.get_parameter("encoder_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.ticks_per_rev = int(self.get_parameter("ticks_per_rev").value)
        self.encoder_sign = [int(v) for v in self.get_parameter("encoder_sign").value]
        self.zero_offset_ticks = [int(v) for v in self.get_parameter("zero_offset_ticks").value]

        if len(self.joint_names) != 4:
            raise RuntimeError("joint_names must contain 4 elements")
        if len(self.encoder_sign) != 4:
            raise RuntimeError("encoder_sign must contain 4 elements")
        if len(self.zero_offset_ticks) != 4:
            raise RuntimeError("zero_offset_ticks must contain 4 elements")
        if self.publish_rate <= 0.0:
            raise RuntimeError("publish_rate must be positive")
        if self.ticks_per_rev <= 0:
            raise RuntimeError("ticks_per_rev must be positive")

        self.rad_per_tick = (2.0 * math.pi) / float(self.ticks_per_rev)
        self.reference_ticks: List[int] | None = None
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.last_stamp = self.get_clock().now().to_msg()

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.sub = self.create_subscription(
            EncoderRaw,
            self.encoder_topic,
            self.on_encoder,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            "encoder_joint_state_publisher configured: "
            f"encoder_topic={self.encoder_topic} publish_rate={self.publish_rate}Hz "
            f"ticks_per_rev={self.ticks_per_rev} frame_id={self.frame_id}"
        )

    def _encode_to_positions(self, msg: EncoderRaw):
        current_ticks = [
            int(msg.encoder_1),
            int(msg.encoder_2),
            int(msg.encoder_3),
            int(msg.encoder_4),
        ]

        if self.reference_ticks is None:
            self.reference_ticks = [
                current_ticks[i] * self.encoder_sign[i] - self.zero_offset_ticks[i]
                for i in range(4)
            ]
            self.get_logger().info(
                "encoder reference initialized: "
                f"ticks=({current_ticks[0]}, {current_ticks[1]}, {current_ticks[2]}, {current_ticks[3]})"
            )

        normalized_ticks = [
            current_ticks[i] * self.encoder_sign[i] - self.zero_offset_ticks[i]
            for i in range(4)
        ]
        self.joint_positions = [
            (normalized_ticks[i] - self.reference_ticks[i]) * self.rad_per_tick
            for i in range(4)
        ]
        self.last_stamp = msg.header.stamp

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.last_stamp
        msg.header.frame_id = self.frame_id
        msg.name = self.joint_names
        msg.position = list(self.joint_positions)
        self.pub.publish(msg)

    def on_encoder(self, msg: EncoderRaw):
        self._encode_to_positions(msg)
        self._publish()

    def on_timer(self):
        self._publish()


def main():
    rclpy.init()
    node = EncoderJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
