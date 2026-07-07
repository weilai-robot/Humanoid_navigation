#!/usr/bin/env python3
"""
leg_odom_node.py — 腿运动学前馈里程计 (方案 A: 独立外挂节点)

原理:
  1. 用 MuJoCo 正运动学计算脚底板相对骨盆的位置 (精度 = MuJoCo 精度)
  2. 检测支撑脚 (Z 最低的脚)
  3. 支撑脚在世界系不动 → 骨盆位移 = -(支撑脚在骨盆系的位移)
  4. 用 IMU yaw 积分得到骨盆朝向
  5. 输出 /leg_odom (nav_msgs/Odometry)

用法:
  ros2 run humanoid_sim leg_odom_node.py
  python3 leg_odom_node.py --ros-args -p model_path:='/path/to/xyber_x1_nav.xml'

输出:
  /leg_odom (nav_msgs/Odometry) — 腿里程计估计 (frame: odom_leg → base_link)
"""

import os
import sys
import time
import math
import numpy as np

import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time as TimeMsg


# ─── 运动学链常量 ───

LEG_JOINT_NAMES = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_pitch_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_pitch_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
]

# 脚底接触点在 ankle_roll body 局部坐标系中的近似中心
FOOT_CONTACT_OFFSET = np.array([0.0, -0.04, 0.0])

# 支撑脚切换的 Z 阈值 (骨盆系中，两脚 Z 差小于此值视为双足支撑)
SUPPORT_FOOT_Z_THRESHOLD = 0.02  # m


def stamp_from_sec(sec: float) -> TimeMsg:
    s = TimeMsg()
    s.sec = int(sec)
    s.nanosec = int((sec - s.sec) * 1e9)
    return s


class LegOdomNode(Node):
    """腿运动学前馈里程计节点"""

    def __init__(self):
        super().__init__('leg_odom_node')

        # ─── 参数 ───
        self.declare_parameter('model_path', '')
        self.declare_parameter('publish_tf', False)  # 默认不发布 TF (避免与 odom_bridge 冲突)
        self.declare_parameter('odom_frame', 'odom_leg')
        self.declare_parameter('base_frame', 'base_link')

        model_path = self.get_parameter('model_path').value
        if not model_path:
            self.get_logger().error('model_path 参数未设置!')
            raise RuntimeError('model_path required')

        self._publish_tf = self.get_parameter('publish_tf').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # ─── 加载 MuJoCo 模型 (仅用于 FK, 不做物理) ───
        self.get_logger().info(f'Loading MuJoCo model for FK: {model_path}')
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 查找 free joint 的 qpos 地址 (用于设置骨盆位姿)
        self._free_joint_adr = -1
        for i in range(self.model.njnt):
            if self.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                self._free_joint_adr = self.model.jnt_qposadr[i]
                break
        if self._free_joint_adr < 0:
            self.get_logger().warn('未找到 free joint, FK 可能有误')

        # 查找关节的 qpos 地址
        self._joint_qpos_adr = {}
        for name in LEG_JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self._joint_qpos_adr[name] = self.model.jnt_qposadr[jid]
            else:
                self.get_logger().warn(f'关节 {name} 未在模型中找到')

        # 查找 body ID
        self._body_ids = {}
        for name in ['x1-body', 'link_left_ankle_roll', 'link_right_ankle_roll',
                      'imu', 'link_left_hip_pitch', 'link_right_hip_pitch']:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid >= 0:
                self._body_ids[name] = bid

        # 查找 site ID (imu site)
        self._imu_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'imu')

        # ─── 状态 ───
        self._initialized = False
        self._prev_foot_left = None    # 骨盆系中左脚位置 (上一帧)
        self._prev_foot_right = None   # 骨盆系中右脚位置 (上一帧)
        self._support_foot = 'none'    # 当前支撑脚: 'left', 'right', 'both'
        self._prev_support_foot = 'none'

        # 累积位姿 (odom_leg 系)
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._pos_z = 0.0
        self._yaw = 0.0                # 从 IMU 获取

        # 速度估计
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._last_update_time = 0.0
        self._prev_yaw = 0.0

        # ─── ROS2 接口 ───
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=50,
        )
        self.create_subscription(JointState, '/joint_states', self._joint_cb, sensor_qos)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, sensor_qos)

        self._odom_pub = self.create_publisher(Odometry, '/leg_odom', 10)

        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'LegOdomNode 启动:\n'
            f'  模型: {model_path}\n'
            f'  关节数: {len(self._joint_qpos_adr)}\n'
            f'  输出: /leg_odom ({self._odom_frame} → {self._base_frame})\n'
            f'  发布 TF: {self._publish_tf}'
        )

    def _imu_cb(self, msg: Imu):
        """从骨盆 IMU 获取 yaw"""
        q = msg.orientation
        # 四元数 (x, y, z, w) → yaw
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._yaw = yaw

        # 角速度
        self._wz = msg.angular_velocity.z

    def _compute_fk(self, joint_angles: dict):
        """
        用 MuJoCo 正运动学计算脚底板在骨盆系中的位置。
        设置 free joint = 原点 (无旋转无平移)，这样 body xpos 就是相对骨盆的坐标。
        """
        # 设置 free joint 为单位变换 (骨盆在原点)
        if self._free_joint_adr >= 0:
            self.data.qpos[self._free_joint_adr + 0] = 0.0  # x
            self.data.qpos[self._free_joint_adr + 1] = 0.0  # y
            self.data.qpos[self._free_joint_adr + 2] = 0.0  # z
            self.data.qpos[self._free_joint_adr + 3] = 1.0  # qw
            self.data.qpos[self._free_joint_adr + 4] = 0.0  # qx
            self.data.qpos[self._free_joint_adr + 5] = 0.0  # qy
            self.data.qpos[self._free_joint_adr + 6] = 0.0  # qz

        # 设置关节角度
        for name, adr in self._joint_qpos_adr.items():
            if name in joint_angles:
                self.data.qpos[adr] = joint_angles[name]

        # 正运动学
        mujoco.mj_forward(self.model, self.data)

        # 读取脚底板位置 (骨盆系 = 世界系, 因为 free joint 设为原点)
        foot_left = None
        foot_right = None

        if 'link_left_ankle_roll' in self._body_ids:
            bid = self._body_ids['link_left_ankle_roll']
            pos = self.data.xpos[bid].copy()
            # 加上脚底接触点偏移 (脚底中心)
            rot = self.data.xmat[bid].reshape(3, 3)
            foot_left = pos + rot @ FOOT_CONTACT_OFFSET

        if 'link_right_ankle_roll' in self._body_ids:
            bid = self._body_ids['link_right_ankle_roll']
            pos = self.data.xpos[bid].copy()
            rot = self.data.xmat[bid].reshape(3, 3)
            foot_right = pos + rot @ FOOT_CONTACT_OFFSET

        return foot_left, foot_right

    def _detect_support_foot(self, foot_left, foot_right):
        """
        检测当前支撑脚。
        在骨盆系中, Z 最小 (最低) 的脚是支撑脚。
        """
        if foot_left is None or foot_right is None:
            return 'none'

        z_diff = foot_left[2] - foot_right[2]

        if abs(z_diff) < SUPPORT_FOOT_Z_THRESHOLD:
            return 'both'  # 双足支撑
        elif z_diff < 0:
            return 'left'  # 左脚更低 → 左脚支撑
        else:
            return 'right'  # 右脚更低 → 右脚支撑

    def _joint_cb(self, msg: JointState):
        """关节状态回调: FK + 支撑脚检测 + 位移积分"""
        # 构建关节角度字典
        joint_angles = {}
        for i, name in enumerate(msg.name):
            if name in self._joint_qpos_adr and i < len(msg.position):
                joint_angles[name] = msg.position[i]

        # 检查是否有足够的关节数据
        if len(joint_angles) < 12:
            return

        # FK 计算脚底板位置 (骨盆系)
        foot_left, foot_right = self._compute_fk(joint_angles)
        if foot_left is None or foot_right is None:
            return

        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if now == 0:
            now = time.monotonic()

        # ─── 初始化 ───
        if not self._initialized:
            self._prev_foot_left = foot_left.copy()
            self._prev_foot_right = foot_right.copy()
            self._support_foot = self._detect_support_foot(foot_left, foot_right)
            self._prev_support_foot = self._support_foot
            self._last_update_time = now
            self._initialized = True
            self.get_logger().info(
                f'Leg odom 初始化:\n'
                f'  左脚 (骨盆系): ({foot_left[0]:.3f}, {foot_left[1]:.3f}, {foot_left[2]:.3f})\n'
                f'  右脚 (骨盆系): ({foot_right[0]:.3f}, {foot_right[1]:.3f}, {foot_right[2]:.3f})\n'
                f'  支撑脚: {self._support_foot}'
            )
            return

        # ─── 检测支撑脚 ───
        support = self._detect_support_foot(foot_left, foot_right)

        # ─── 计算骨盆位移 ───
        # 支撑脚在世界系不动 → 骨盆位移 = -(支撑脚在骨盆系中的位移)
        dx = 0.0
        dy = 0.0

        if support == 'left' or support == 'both':
            # 左脚支撑: 骨盆移动了 -(左脚在骨盆系中的变化)
            dx_l = -(foot_left[0] - self._prev_foot_left[0])
            dy_l = -(foot_left[1] - self._prev_foot_left[1])
            dx, dy = dx_l, dy_l

        if support == 'right':
            dx_r = -(foot_right[0] - self._prev_foot_right[0])
            dy_r = -(foot_right[1] - self._prev_foot_right[1])
            dx, dy = dx_r, dy_r

        if support == 'both':
            # 双足支撑: 取两脚位移的平均
            dx_r = -(foot_right[0] - self._prev_foot_right[0])
            dy_r = -(foot_right[1] - self._prev_foot_right[1])
            dx = (dx_l + dx_r) / 2.0
            dy = (dy_l + dy_r) / 2.0

        # 位移需要从骨盆当前朝向旋转到世界系
        # 骨盆朝向 = self._yaw (从 IMU 获取)
        cos_yaw = math.cos(self._yaw)
        sin_yaw = math.sin(self._yaw)
        world_dx = cos_yaw * dx - sin_yaw * dy
        world_dy = sin_yaw * dx + cos_yaw * dy

        # 积分
        self._pos_x += world_dx
        self._pos_y += world_dy
        self._pos_z = 0.0  # 2D 平面假设

        # ─── 速度估计 ───
        dt = now - self._last_update_time
        if dt > 1e-6:
            # 用世界系位移算速度
            self._vx = world_dx / dt
            self._vy = world_dy / dt
            self._wz = (self._yaw - self._prev_yaw) / dt

        self._prev_yaw = self._yaw
        self._last_update_time = now

        # 支撑脚切换日志
        if support != self._prev_support_foot:
            self.get_logger().info(
                f'支撑脚切换: {self._prev_support_foot} → {support}'
            )
            self._prev_support_foot = support

        # 更新上一帧脚位置
        self._prev_foot_left = foot_left.copy()
        self._prev_foot_right = foot_right.copy()

        # ─── 发布 /leg_odom ───
        self._publish_odom(now)

    def _publish_odom(self, sim_time: float):
        """发布腿里程计"""
        odom = Odometry()
        odom.header.stamp = stamp_from_sec(sim_time)
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        # 位姿
        odom.pose.pose.position.x = self._pos_x
        odom.pose.pose.position.y = self._pos_y
        odom.pose.pose.position.z = self._pos_z

        # yaw → quaternion
        half = self._yaw * 0.5
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)

        # 协方差 (腿里程计的 x/y 比较准, yaw 来自 IMU 也可信)
        odom.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,    # x
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,    # y
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,    # z (不使用)
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,    # roll (不使用)
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,    # pitch (不使用)
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05,   # yaw
        ]

        # 速度
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._wz

        odom.twist.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1,
        ]

        self._odom_pub.publish(odom)

        # 可选: 发布 TF
        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._pos_x
            tf.transform.translation.y = self._pos_y
            tf.transform.translation.z = self._pos_z
            tf.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = LegOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
