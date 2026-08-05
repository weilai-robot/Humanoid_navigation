#!/usr/bin/env python3
"""
TF + Odometry + cmd_vel 桥接节点：
  1. 将 FastLIO2 的 camera_init->body 位姿转换为 Nav2 标准的 odom->base_footprint TF
  2. 发布标准 nav_msgs/Odometry 话题到 /odom（携带速度信息，MPPI 控制器必需）
  3. 中继 Nav2 /cmd_vel → aimrt_main /cmd_vel_limiter，附加加速度限幅（防速度跳变致 RL 摔倒）

工作原理：
  FastLIO2 发布 /Odometry (frame: camera_init, child: body)
  本节点：
    - 广播 TF: odom -> base_footprint（Costmap、规划器等依赖）
    - 发布 /odom 话题：包含位姿 + 速度（MPPI 等局部规划器依赖 twist 做轨迹预测）
    - 订阅 /cmd_vel (Nav2 MPPI 输出)，经 VelocityRateLimiter 限幅后发布 /cmd_vel_limiter

TF 树最终结构：
  map ──(static)──> odom ──(本节点)──> base_footprint ──(URDF)──> base_link ──> lidar_link / imu_link
                     │
  map ──(static)──> camera_init ──(FastLIO2)──> body   （FastLIO2 自身的分支，OctoMap 使用）

cmd_vel 链路：
  Nav2 /cmd_vel ──(VelocityRateLimiter)──> /cmd_vel_limiter ──> aimrt_main ControlModule
  限幅参数与 MPPI 的 ax_max/az_max 对齐（见 nav2_mujoco.yaml），不干扰正常规划，仅截断异常跳变。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
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
        """对 (vx, vy, wz) 施加加速度限幅，返回限幅后的值。

        Args:
            vx, vy, wz: 目标速度 (m/s, m/s, rad/s)
            now_sec: 当前时间戳 (秒)，应来自 ROS clock (sim time)
        Returns:
            (vx_limited, vy_limited, wz_limited)
        """
        if self._last_time is None:
            # 首帧：记录状态，直接透传
            self._last_vx = vx
            self._last_vy = vy
            self._last_wz = wz
            self._last_time = now_sec
            return vx, vy, wz

        dt = now_sec - self._last_time
        if dt < 1e-6:
            # 时间戳未前进（同一周期多帧或时钟抖动），用上一帧结果
            return self._last_vx, self._last_vy, self._last_wz

        # 计算实际加速度并 clamp
        ax = (vx - self._last_vx) / dt
        ay = (vy - self._last_vy) / dt
        az = (wz - self._last_wz) / dt

        ax = max(-self.max_ax, min(self.max_ax, ax))
        ay = max(-self.max_ay, min(self.max_ay, ay))
        az = max(-self.max_az, min(self.max_az, az))

        # 积分回速度
        self._last_vx += ax * dt
        self._last_vy += ay * dt
        self._last_wz += az * dt
        self._last_time = now_sec

        return self._last_vx, self._last_vy, self._last_wz


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        # --- 参数 ---
        self.declare_parameter('body_to_footprint_z', -1.31)  # body 到 base_footprint 的 Z 偏移
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('input_topic', '/Odometry')     # FastLIO2 的里程计话题
        self.declare_parameter('output_topic', '/odom')        # 输出的标准里程计话题

        # --- cmd_vel relay 参数 ---
        self.declare_parameter('enable_cmd_vel_relay', True)
        self.declare_parameter('cmd_vel_input_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_output_topic', '/cmd_vel_limiter')
        self.declare_parameter('max_ax', 1.5)    # m/s²，匹配 MPPI ax_max
        self.declare_parameter('max_ay', 0.5)    # m/s²，侧步保守值
        self.declare_parameter('max_az', 2.0)    # rad/s²，略大于 MPPI az_max

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
            f'OdomBridge 启动:\n'
            f'  输入: {input_topic}\n'
            f'  输出TF: {self.odom_frame} -> {self.base_frame}\n'
            f'  输出话题: {output_topic}\n'
            f'  Z偏移: {self.body_to_footprint_z}m'
            + (
                f'\n  cmd_vel relay: {cmd_vel_input} -> {cmd_vel_output}'
                f' (max_ax={self._rate_limiter.max_ax},'
                f' max_ay={self._rate_limiter.max_ay},'
                f' max_az={self._rate_limiter.max_az})'
                if self._enable_cmd_vel_relay else '\n  cmd_vel relay: DISABLED'
            )
        )

    def _cmd_vel_relay_cb(self, msg: Twist):
        """cmd_vel 中继回调：对 Nav2 输出施加加速度限幅后转发到 /cmd_vel_limiter

        限幅范围与 nav2_mujoco.yaml 中 MPPI 的 ax_max/az_max 对齐：
          max_ax=1.5 m/s²  (MPPI ax_max=1.5)
          max_az=2.0 rad/s² (MPPI az_max=1.0，此处更宽松仅截断异常跳变)

        当输入突然归零（如 nav_state_manager 安全停止）时，
        限幅器以 max_ax 的减速度平滑到零，约 0.33s (0.5→0 @1.5m/s²)。
        但此时机器人已在 stand 模式（nav_state_manager 先发 stand_mode 再停 cmd_vel），
        RL 步态已退出，故平滑归零不会引发失稳。
        """
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

        # 使用当前仿真时间而非 FastLIO2 消息时间作为 TF 时间戳
        # FastLIO2 处理+传输有 ~20-30ms 延迟，若用 msg.header.stamp 会导致 Nav2 控制器
        # 查询"当前时刻"的 TF 时出现 ExtrapolationError，进而触发重规划风暴
        stamp = self.get_clock().now().to_msg()

        
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
