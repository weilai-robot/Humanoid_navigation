#!/usr/bin/env python3
"""
TF + Odometry 桥接节点：
  1. 将 FastLIO2 的 camera_init->body 位姿转换为 Nav2 标准的 odom->base_footprint TF
  2. 发布标准 nav_msgs/Odometry 话题到 /odom（携带速度信息，MPPI 控制器必需）

工作原理：
  FastLIO2 发布 /Odometry (frame: camera_init, child: body)
  本节点：
    - 广播 TF: odom -> base_footprint（Costmap、规划器等依赖）
    - 发布 /odom 话题：包含位姿 + 速度（MPPI 等局部规划器依赖 twist 做轨迹预测）

TF 树最终结构：
  map ──(static)──> odom ──(本节点)──> base_footprint ──(URDF)──> base_link ──> lidar_link / imu_link
                     │
  map ──(static)──> camera_init ──(FastLIO2)──> body   （FastLIO2 自身的分支，OctoMap 使用）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        # --- 参数 ---
        self.declare_parameter('body_to_footprint_z', -1.31)  # body 到 base_footprint 的 Z 偏移
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('input_topic', '/Odometry')     # FastLIO2 的里程计话题
        self.declare_parameter('output_topic', '/odom')        # 输出的标准里程计话题

        self.body_to_footprint_z = self.get_parameter('body_to_footprint_z').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 发布标准 /odom 话题（MPPI 等局部规划器需要其中的速度信息）
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)

        # 添加速度平滑滤波(EMA滤波器)状态变量
        self.alpha_v = 0.2
        self.filtered_twist_linear_x = 0.0
        self.filtered_twist_linear_y = 0.0
        self.filtered_twist_linear_z = 0.0
        self.filtered_twist_angular_x = 0.0
        self.filtered_twist_angular_y = 0.0
        self.filtered_twist_angular_z = 0.0

        # 保活机制：缓存最后一帧的位姿数据，FastLIO2停止时仍持续发布TF
        # self._last_footprint_x = None
        # self._last_footprint_y = None
        # self._last_footprint_z = None
        # self._last_flat_quat = None
        # self._keepalive_timer = self.create_timer(0.1, self._keepalive_callback)  # 10Hz

        # 订阅 FastLIO2 的 /Odometry
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.sub = self.create_subscription(
            Odometry, input_topic, self.odom_callback, qos
        )

        self.get_logger().info(
            f'OdomBridge 启动:\n'
            f'  输入: {input_topic}\n'
            f'  输出TF: {self.odom_frame} -> {self.base_frame}\n'
            f'  输出话题: {output_topic}\n'
            f'  Z偏移: {self.body_to_footprint_z}m'
        )

    def odom_callback(self, msg: Odometry):
        """
        将 FastLIO2 的 Odometry (camera_init->body) 转换为：
          1. TF: odom -> base_footprint
          2. 话题: /odom (nav_msgs/Odometry)，包含位姿和速度

        转换逻辑：
        T(odom->base_footprint) = T(camera_init->body) * T(body->base_footprint)
        其中 T(body->base_footprint) 是纯 Z 轴平移 (-1.31m)
        """
        # 提取 FastLIO2 的位姿（camera_init 坐标系下 body 的位置）
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # 将 body->base_footprint 的偏移（在 body 局部坐标系下）转换到世界坐标系
        # body 到 base_footprint 在 body 局部坐标系下 is (0, 0, -1.31)
        # 需要用 body 的旋转矩阵将其旋转到世界坐标系
        quat = [ori.x, ori.y, ori.z, ori.w]
        rot = Rotation.from_quat(quat)
        local_offset = np.array([0.0, 0.0, self.body_to_footprint_z])
        world_offset = rot.apply(local_offset)

        # base_footprint 在 odom 坐标系下的位置
        footprint_x = pos.x + world_offset[0]
        footprint_y = pos.y + world_offset[1]
        # footprint_z = pos.z + world_offset[2]
        # 直接加 body_to_footprint_z(-1.31) 会让脚底板去到 Z=-1.31（地下）
        # 减去 body_to_footprint_z（即 +1.31）将 odom 系的初始平面拉回地面 Z=0
        footprint_z = pos.z + world_offset[2] - self.body_to_footprint_z

        stamp = msg.header.stamp

        
        # 强制 base_footprint 只保留 Yaw 角 (抹平 Pitch 和 Roll 以满足 Nav2 平面代价地图要求)
        r = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
        euler = r.as_euler('xyz', degrees=False)
        yaw = euler[2]
        flat_quat = Rotation.from_euler('xyz', [0, 0, yaw]).as_quat()

        # ========== 1. 广播 TF: odom -> base_footprint ==========
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = footprint_x
        t.transform.translation.y = footprint_y
        t.transform.translation.z = footprint_z
        # t.transform.rotation = ori  # 旋转保持不变（body 和 base_footprint 朝向一致）
        t.transform.rotation.x = flat_quat[0]
        t.transform.rotation.y = flat_quat[1]
        t.transform.rotation.z = flat_quat[2]
        t.transform.rotation.w = flat_quat[3]
        self.tf_broadcaster.sendTransform(t)
        
        # self._last_footprint_x = footprint_x
        # self._last_footprint_y = footprint_y
        # self._last_footprint_z = footprint_z
        # self._last_flat_quat = flat_quat

        # ========== 2. 发布 /odom 话题（含速度信息） ==========
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # 位姿：转换后的 base_footprint 位置
        odom_msg.pose.pose.position.x = footprint_x
        odom_msg.pose.pose.position.y = footprint_y
        odom_msg.pose.pose.position.z = footprint_z
        # odom_msg.pose.pose.orientation = ori
        odom_msg.pose.pose.orientation.x = flat_quat[0]
        odom_msg.pose.pose.orientation.y = flat_quat[1]
        odom_msg.pose.pose.orientation.z = flat_quat[2]
        odom_msg.pose.pose.orientation.w = flat_quat[3]
        odom_msg.pose.covariance = msg.pose.covariance  # 保留原始协方差

        # 速度：直接转发 FastLIO2 的速度（body 系下的速度 ≈ base_footprint 系下的速度）
        # TODO 以后换成人形机器人可能要进行修改，重新计算雷达位置到base_footprint的速度映射
        # odom_msg.twist = msg.twist

        # 速度：平滑滤波 (EMA 滤波，过滤高频抖动)
        raw_tw = msg.twist.twist
        self.filtered_twist_linear_x = self.alpha_v * raw_tw.linear.x + (1 - self.alpha_v) * self.filtered_twist_linear_x
        self.filtered_twist_linear_y = self.alpha_v * raw_tw.linear.y + (1 - self.alpha_v) * self.filtered_twist_linear_y
        self.filtered_twist_linear_z = self.alpha_v * raw_tw.linear.z + (1 - self.alpha_v) * self.filtered_twist_linear_z
        self.filtered_twist_angular_x = self.alpha_v * raw_tw.angular.x + (1 - self.alpha_v) * self.filtered_twist_angular_x
        self.filtered_twist_angular_y = self.alpha_v * raw_tw.angular.y + (1 - self.alpha_v) * self.filtered_twist_angular_y
        self.filtered_twist_angular_z = self.alpha_v * raw_tw.angular.z + (1 - self.alpha_v) * self.filtered_twist_angular_z

        odom_msg.twist.twist.linear.x = self.filtered_twist_linear_x
        odom_msg.twist.twist.linear.y = self.filtered_twist_linear_y
        odom_msg.twist.twist.linear.z = self.filtered_twist_linear_z
        odom_msg.twist.twist.angular.x = self.filtered_twist_angular_x
        odom_msg.twist.twist.angular.y = self.filtered_twist_angular_y
        odom_msg.twist.twist.angular.z = self.filtered_twist_angular_z
        odom_msg.twist.covariance = msg.twist.covariance

        self.odom_pub.publish(odom_msg)

    # def _keepalive_callback(self):
    #     if self._last_footprint_x is None:
    #         return
    #     now_stamp = self.get_clock().now().to_msg()

    #     t = TransformStamped()
    #     t.header.stamp = now_stamp
    #     t.header.frame_id = self.odom_frame
    #     t.child_frame_id = self.base_frame
    #     t.transform.translation.x = self._last_footprint_x
    #     t.transform.translation.y = self._last_footprint_y
    #     t.transform.translation.z = self._last_footprint_z
    #     t.transform.rotation.x = self._last_flat_quat[0]
    #     t.transform.rotation.y = self._last_flat_quat[1]
    #     t.transform.rotation.z = self._last_flat_quat[2]
    #     t.transform.rotation.w = self._last_flat_quat[3]
    #     self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
