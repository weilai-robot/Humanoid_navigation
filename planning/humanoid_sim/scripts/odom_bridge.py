#!/usr/bin/env python3
"""
TF + Odometry + cmd_vel 桥接节点 (Ground-Truth 驱动版)：
  1. 订阅 MuJoCo /mujoco/ground_truth (Float64MultiArray)，用机器人真实位姿
     发布 Nav2 标准的 odom->base_footprint TF（零漂移，替代发散的 FastLIO2）
  2. 发布标准 nav_msgs/Odometry 话题到 /odom（携带差分速度，MPPI 控制器必需）
  3. 中继 Nav2 /cmd_vel → aimrt_main /cmd_vel_limiter，附加加速度限幅（防速度跳变致 RL 摔倒）

工作原理：
  MuJoCo sim_module 发布 /mujoco/ground_truth:
    data = [sim_t, x, y, z, roll, pitch, yaw, rtf, collisions, cum_dist]
    其中 (x, y, z) 是 free joint (pelvis) 在 MuJoCo 世界系的位置
  本节点采用【相对原点法】：
    - 首帧记录 (x0, y0) 作为 odom 原点偏移
    - odom->base_footprint.translation = (x - x0, y - y0, 0)
    - 保持 odom 坐标系语义与原 FastLIO2 一致（原点 = 机器人初始位置）
    - 配合 map->odom 静态 identity TF，使 map(goal) 坐标 = 相对起点的位移

TF 树最终结构：
  map ──(静态 identity)──> odom ──(本节点, GT)──> base_footprint ──(静态 +0.65z)──> base_link ──> lidar_link

cmd_vel 链路：
  Nav2 /cmd_vel ──(VelocityRateLimiter)──> /cmd_vel_limiter ──> aimrt_main ControlModule
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation


class VelocityRateLimiter:
    """速度变化率限制器（加速度限幅）

    将 Nav2 输出的速度指令平滑到 RL 步态可跟踪的加速度范围内，
    防止 MPPI 轨迹切换或规划器重置时产生的速度跳变冲击 RL 策略导致摔倒。

    参数选择依据 (与 nav2_mujoco.yaml MPPI 配置对齐):
      max_ax = 1.5 m/s²   — 匹配 MPPI ax_max=1.5，不干扰正常加速
      max_ay = 0.5 m/s²   — 人形侧步保守值 (当前 DiffDrive vy=0，不触发)
      max_az = 2.0 rad/s² — 略大于 MPPI az_max=1.0，仅截断异常跳变
    """

    def __init__(self, max_ax=1.5, max_ay=0.5, max_az=2.0):
        self.max_ax = max_ax
        self.max_ay = max_ay
        self.max_az = max_az
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0
        self._last_time = None  # None 表示首帧，直接透传不限幅

    def limit(self, vx, vy, wz, now_sec):
        """对 (vx, vy, wz) 施加加速度限幅，返回限幅后的值。"""
        if self._last_time is None:
            self._last_vx = vx
            self._last_vy = vy
            self._last_wz = wz
            self._last_time = now_sec
            return vx, vy, wz

        dt = now_sec - self._last_time
        if dt < 1e-6:
            return self._last_vx, self._last_vy, self._last_wz

        ax = (vx - self._last_vx) / dt
        ay = (vy - self._last_vy) / dt
        az = (wz - self._last_wz) / dt

        ax = max(-self.max_ax, min(self.max_ax, ax))
        ay = max(-self.max_ay, min(self.max_ay, ay))
        az = max(-self.max_az, min(self.max_az, az))

        self._last_vx += ax * dt
        self._last_vy += ay * dt
        self._last_wz += az * dt
        self._last_time = now_sec

        return self._last_vx, self._last_vy, self._last_wz


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        # --- 参数 ---
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('input_topic', '/mujoco/ground_truth')  # MuJoCo GT 话题
        self.declare_parameter('output_topic', '/odom')
        # 保留兼容参数 (GT 模式不使用，但 launch 仍会传入)
        self.declare_parameter('body_to_footprint_z', -1.25)

        # --- cmd_vel relay 参数 ---
        self.declare_parameter('enable_cmd_vel_relay', True)
        self.declare_parameter('cmd_vel_input_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_output_topic', '/cmd_vel_limiter')
        self.declare_parameter('max_ax', 1.5)    # m/s²，匹配 MPPI ax_max
        self.declare_parameter('max_ay', 0.5)    # m/s²，侧步保守值
        self.declare_parameter('max_az', 2.0)    # rad/s²，略大于 MPPI az_max

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 发布标准 /odom 话题（MPPI 等局部规划器需要其中的速度信息）
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)

        # 速度平滑滤波(EMA滤波器) — GT 差分速度轻微去噪
        self.alpha_v = 0.3

        # --- 相对原点状态 (首帧 GT 位置作为 odom 原点) ---
        self._origin_set = False
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._prev_sim_t = None
        self._prev_odom_x = 0.0
        self._prev_odom_y = 0.0
        self._prev_yaw = 0.0
        self._filt_vx = 0.0
        self._filt_wz = 0.0

        # 订阅 MuJoCo ground truth
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.sub = self.create_subscription(
            Float64MultiArray, input_topic, self.gt_callback, qos
        )

        # --- cmd_vel relay: /cmd_vel → /cmd_vel_limiter (加速度限幅) ---
        self._enable_cmd_vel_relay = self.get_parameter('enable_cmd_vel_relay').value
        cmd_vel_input = self.get_parameter('cmd_vel_input_topic').value
        cmd_vel_output = self.get_parameter('cmd_vel_output_topic').value

        if self._enable_cmd_vel_relay:
            max_ax = self.get_parameter('max_ax').value
            max_ay = self.get_parameter('max_ay').value
            max_az = self.get_parameter('max_az').value
            self._rate_limiter = VelocityRateLimiter(max_ax, max_ay, max_az)
            self._cmd_vel_sub = self.create_subscription(
                Twist, cmd_vel_input, self._cmd_vel_relay_cb, 10
            )
            self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_output, 10)

        self.get_logger().info(
            f'OdomBridge (GT) 启动:\n'
            f'  输入: {input_topic} (Float64MultiArray)\n'
            f'  输出TF: {self.odom_frame} -> {self.base_frame}\n'
            f'  输出话题: {output_topic}\n'
            f'  定位源: MuJoCo ground truth (零漂移)'
            + (
                f'\n  cmd_vel relay: {cmd_vel_input} -> {cmd_vel_output}'
                f' (max_ax={self._rate_limiter.max_ax},'
                f' max_ay={self._rate_limiter.max_ay},'
                f' max_az={self._rate_limiter.max_az})'
                if self._enable_cmd_vel_relay else '\n  cmd_vel relay: DISABLED'
            )
        )

    def _cmd_vel_relay_cb(self, msg: Twist):
        """cmd_vel 中继回调：对 Nav2 输出施加加速度限幅后转发到 /cmd_vel_limiter"""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        vx, vy, wz = self._rate_limiter.limit(
            msg.linear.x, msg.linear.y, msg.angular.z, now_sec
        )

        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.linear.z = msg.linear.z   # passthrough (恒为 0)
        out.angular.x = msg.angular.x  # passthrough (恒为 0)
        out.angular.y = msg.angular.y  # passthrough (恒为 0)
        out.angular.z = wz
        self._cmd_vel_pub.publish(out)

    def gt_callback(self, msg: Float64MultiArray):
        """解析 MuJoCo ground truth，发布 odom->base_footprint TF + /odom 话题

        GT 数据格式: [sim_t, x, y, z, roll, pitch, yaw, rtf, collisions, cum_dist]
        相对原点法：首帧位置作为 odom 原点，消除绝对坐标偏移，
        保持 odom 坐标系语义与原 FastLIO2 一致（原点=机器人初始位置）。
        """
        data = msg.data
        if len(data) < 7:
            return

        sim_t = data[0]
        gt_x = data[1]
        gt_y = data[2]
        # data[3] = z (pelvis高度, 站立≈0.6m)
        # data[4] = roll, data[5] = pitch
        yaw = data[6]

        # 相对原点：首帧记录
        if not self._origin_set:
            self._origin_x = gt_x
            self._origin_y = gt_y
            self._origin_set = True
            self.get_logger().info(
                f'GT odom 原点设定: ({gt_x:.3f}, {gt_y:.3f})'
            )

        odom_x = gt_x - self._origin_x
        odom_y = gt_y - self._origin_y

        # 速度差分（用 GT sim_time，精确）
        vx = 0.0
        wz = 0.0
        if self._prev_sim_t is not None:
            dt = sim_t - self._prev_sim_t
            if dt > 1e-6:
                vx = (odom_x - self._prev_odom_x) / dt
                dyaw = yaw - self._prev_yaw
                # 角度环绕处理
                if dyaw > math.pi:
                    dyaw -= 2 * math.pi
                elif dyaw < -math.pi:
                    dyaw += 2 * math.pi
                wz = dyaw / dt
        self._prev_sim_t = sim_t
        self._prev_odom_x = odom_x
        self._prev_odom_y = odom_y
        self._prev_yaw = yaw

        # EMA 平滑速度（轻微去噪，GT 差分本身已较准）
        self._filt_vx = self.alpha_v * vx + (1 - self.alpha_v) * self._filt_vx
        self._filt_wz = self.alpha_v * wz + (1 - self.alpha_v) * self._filt_wz

        # base_footprint: 仅保留 Yaw（抹平 Pitch/Roll 以满足 Nav2 平面代价地图）
        flat_quat = Rotation.from_euler('xyz', [0, 0, yaw]).as_quat()

        # 使用当前仿真时间作为 TF 时间戳
        stamp = self.get_clock().now().to_msg()

        # ========== 1. 广播 TF: odom -> base_footprint ==========
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = odom_x
        t.transform.translation.y = odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = flat_quat[0]
        t.transform.rotation.y = flat_quat[1]
        t.transform.rotation.z = flat_quat[2]
        t.transform.rotation.w = flat_quat[3]
        self.tf_broadcaster.sendTransform(t)

        # ========== 2. 发布 /odom 话题（含速度信息，MPPI 必需）==========
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = odom_x
        odom_msg.pose.pose.position.y = odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = flat_quat[0]
        odom_msg.pose.pose.orientation.y = flat_quat[1]
        odom_msg.pose.pose.orientation.z = flat_quat[2]
        odom_msg.pose.pose.orientation.w = flat_quat[3]

        odom_msg.twist.twist.linear.x = self._filt_vx
        odom_msg.twist.twist.angular.z = self._filt_wz

        self.odom_pub.publish(odom_msg)


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
