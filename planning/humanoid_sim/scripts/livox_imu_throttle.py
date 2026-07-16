#!/usr/bin/env python3
"""
livox_imu_throttle.py — 将 sim_module 的 1000Hz /livox/imu 抽稀为 ~200Hz

用途:
  FastLIO 单线程 + 默认 IMU 队列 depth=10，直接订 1000Hz 易在 ICP 期间丢包。
  本节点订原始话题，按仿真时间节流后发到 /livox/imu_200，供 FastLIO 使用。

不改动:
  - sim_module（仍可发 1000Hz /livox/imu）
  - /imu/data（RL body IMU）

用法:
  ros2 run humanoid_sim livox_imu_throttle.py
  # 或
  python3 livox_imu_throttle.py --ros-args \\
    -p input_topic:=/livox/imu -p output_topic:=/livox/imu_200 -p output_hz:=200.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class LivoxImuThrottle(Node):
    def __init__(self):
        super().__init__("livox_imu_throttle")

        self.declare_parameter("input_topic", "/livox/imu")
        self.declare_parameter("output_topic", "/livox/imu_200")
        self.declare_parameter("output_hz", 200.0)

        self._input_topic = self.get_parameter("input_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        output_hz = float(self.get_parameter("output_hz").value)
        if output_hz <= 0.0:
            raise RuntimeError("output_hz must be > 0")
        self._period = 1.0 / output_hz
        self._last_pub_t = None
        self._in_count = 0
        self._out_count = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,  # 输入仍可能是 1000Hz，加大以免本节点自己丢
        )
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self._pub = self.create_publisher(Imu, self._output_topic, out_qos)
        self.create_subscription(Imu, self._input_topic, self._cb, sensor_qos)

        self.get_logger().info(
            f"IMU throttle: {self._input_topic} -> {self._output_topic} @ {output_hz:.0f} Hz "
            f"(period={self._period*1000:.1f} ms, sim-time stamp)"
        )

    def _cb(self, msg: Imu):
        self._in_count += 1
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_pub_t is not None and (t - self._last_pub_t) < self._period:
            return
        self._last_pub_t = t
        self._pub.publish(msg)
        self._out_count += 1
        if self._out_count == 1 or self._out_count % 1000 == 0:
            ratio = self._out_count / max(self._in_count, 1)
            self.get_logger().info(
                f"throttled frames={self._out_count} in={self._in_count} "
                f"keep_ratio={ratio:.3f} last_sim_t={t:.3f}",
                throttle_duration_sec=5.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = LivoxImuThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
