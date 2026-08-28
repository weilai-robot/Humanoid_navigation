#!/usr/bin/env python3
"""cloud_retimer.py — 重盖点云时间戳为当前时刻并转发

问题 (CI run 33059189624 及此前):
  FastLIO 的 /cloud_registered_body 时间戳 = lidar_end_time (扫描结束的
  仿真时刻)。RTF<0.5 + 处理积压下, 点云到达时 odom_bridge 已发布大量更
  新的 TF; TF 缓冲区(默认10s)中最旧条目晚于点云时间戳时, nav2 MessageFilter
  报 "the timestamp on the message is earlier than all the data in the
  transform cache" 并永久丢弃观测 — 观测源 transform_tolerance 只是等待
  窗口, 对"早于缓冲区最旧条目"无效。costmap 因此对动态障碍半盲:
  dyn_person(0.5,-3.0)/dyn_crate(3.5,-3.0) 不在静态地图, 恰压在通道A绕行
  线上 → 每场景 1 次碰撞 + D 场景摔倒。

方案:
  订阅 /cloud_registered_body, 把 header.stamp 重盖为当前时钟(即 TF 流
  最新时刻), 转发到 /cloud_registered_body_fresh。代价: 扫描时刻与重盖
  时刻间机器人已位移 (0.4m/s × ~0.3s ≈ 0.12m), 对障碍标记可接受
  (墙体/纸箱尺度 >> 0.12m, 且偏保守方向)。nav2 观测源改订 fresh 话题。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2


class CloudRetimer(Node):
    def __init__(self):
        super().__init__('cloud_retimer')
        self.declare_parameter('input_topic', '/cloud_registered_body')
        self.declare_parameter('output_topic', '/cloud_registered_body_fresh')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(PointCloud2, out, qos)
        self.create_subscription(PointCloud2, inp, self._cb, qos)
        self._in = 0
        self._out = 0
        self.create_timer(10.0, self._stat)
        self.get_logger().info(f'cloud_retimer: {inp} -> {out} (stamp := now)')

    def _cb(self, msg: PointCloud2):
        self._in += 1
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)
        self._out += 1

    def _stat(self):
        self.get_logger().info(
            f'relayed {self._out}/{self._in} clouds (10s window)',
            throttle_duration_sec=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CloudRetimer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
