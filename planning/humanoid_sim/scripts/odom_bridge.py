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
  本限幅器是链路中唯一的加速度权威（nav2 侧 MPPI 加速度死键已于 2026-11-08 删除，
  勿再按「与 MPPI 对齐」选参）：
  - P6a 加速向对称夹 max_ax、减速向非对称夹 max_decel（双足急停温柔化，减速更慢）；
  - P6b 输入沉默时由 20Hz 定时器接管：末目标≈0（取消/门控停的零帧收尾）走快车道
    立即续刹到零；末目标≠0（上游崩溃形状）保沉默 T 秒后强制归零 ramp（死人开关）。
    详见 _silence_timer_cb。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation

# ---- P6 沉默期接管常量（节点内部，不进 launch）----
SILENCE_TIMER_PERIOD = 0.05   # 20Hz 巡检周期，与 nav2 指令帧率一致
SILENCE_MARGIN = 0.1          # 沉默判定余量：> 帧周期 0.05 且 << T；帧流正常时回调负责，定时器不双积分
SILENCE_DT_CAP = 0.1          # 定时器单步 dt 上限：sim /clock 前跳最多当 0.1s 用，防前跳虚假放大单步衰减（审核 F4）


class VelocityRateLimiter:
    """速度变化率限制器（P6a 非对称加速度限幅）

    将 Nav2 输出的速度指令平滑到 RL 步态可跟踪的加速度范围内，
    防止速度跳变冲击 RL 策略导致摔倒。加速向与减速向非对称：
    急停（减速向）比加速更温柔——7500 步行模型 settle-fall 族对急停敏感。

    参数选择依据（本限幅器是链路唯一加速度权威；sim 侧值见 tf_bridge.launch.py）：
      max_ax       = 1.5 m/s²   — 加速向上界（对称夹）
      max_ay       = 0.5 m/s²   — 侧步保守值 (当前 DiffDrive vy=0，不触发，双向对称)
      max_az       = 2.0 rad/s² — wz 加速向上界（对称夹）
      max_decel    = 0.6 m/s²   — 减速向下界（P6a 新增）；None/≤0 → =max_ax（旧对称行为）
      max_decel_wz = 2.0 rad/s² — wz 衰减向下界（P6a 新增，默认=max_az 行为不变；
                                  None/≤0 → =max_az。收紧前须先跑 wz 版刹车需求分布，
                                  且收紧方向待证——减速更慢=转弯持续更久未必利好）
    """

    def __init__(self, max_ax=1.5, max_ay=0.5, max_az=2.0,
                 max_decel=None, max_decel_wz=None):
        self.max_ax = max_ax
        self.max_ay = max_ay
        self.max_az = max_az
        # P6a：减速向夹板，缺省/非法值回落到对称（旧行为）
        self.max_decel = max_decel if (max_decel and max_decel > 0) else max_ax
        self.max_decel_wz = max_decel_wz if (max_decel_wz and max_decel_wz > 0) else max_az
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0
        self._last_time = None  # None 表示首帧，直接透传不限幅
        # P6b：末目标记忆 = 沉默期收敛对象（每次收帧顺带记录；定时器据此走快车道/超时归零）
        self._tgt = (0.0, 0.0, 0.0)

    def limit(self, vx, vy, wz, now_sec):
        """对 (vx, vy, wz) 施加加速度限幅，返回限幅后的值（帧流路径，回调调用）。

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
            self._tgt = (vx, vy, wz)
            return vx, vy, wz

        dt = now_sec - self._last_time
        if dt < 1e-6:
            # 时间戳未前进（同一周期多帧或时钟抖动），用上一帧结果
            return self._last_vx, self._last_vy, self._last_wz

        self._tgt = (vx, vy, wz)
        return self._integrate(self._tgt, dt, now_sec)

    def _integrate(self, tgt, dt, now_sec):
        """向目标速度单步收敛（限幅器唯一积分点，帧流回调与沉默定时器共用）。

        加速向夹 max_ax/max_ay/max_az，减速向夹 max_decel/max_ay/max_decel_wz。
        积分后更新 _last_* 状态；调用方保证 0 < dt（limit 的守卫在本方法之外）。
        """
        ax = (tgt[0] - self._last_vx) / dt
        ay = (tgt[1] - self._last_vy) / dt
        az = (tgt[2] - self._last_wz) / dt

        ax = max(-self.max_decel, min(self.max_ax, ax))          # P6a：减速向非对称
        ay = max(-self.max_ay, min(self.max_ay, ay))
        az = max(-self.max_decel_wz, min(self.max_az, az))       # P6a：wz 衰减夹板（默认=max_az 行为不变）

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
        self.declare_parameter('mount_rpy', [0.0, 0.0, 0.0])   # 雷达/IMU 相对 base_link 的安装 RPY(度)，纠正 mount 朝向偏移；0=不修正

        # --- cmd_vel relay 参数 ---
        self.declare_parameter('enable_cmd_vel_relay', True)
        self.declare_parameter('cmd_vel_input_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_output_topic', '/cmd_vel_limiter')
        self.declare_parameter('max_ax', 1.5)    # m/s²，加速向上界（对称夹）
        self.declare_parameter('max_ay', 0.5)    # m/s²，侧步保守值（双向对称）
        self.declare_parameter('max_az', 2.0)    # rad/s²，wz 加速向上界
        # P6a：减速向非对称夹板（双足急停温柔化）。0 = 对称沿用 max_ax/max_az（旧行为）。
        # sim 0.6 / real 0.5（real 无含 /cmd_vel 的 bag，取保守初值，复核挂 T1 清单）
        self.declare_parameter('max_decel', 0.0)
        self.declare_parameter('max_decel_wz', 0.0)   # rad/s²，wz 衰减夹板；0 = 对称沿用 max_az（行为不变）
        # P6b：/cmd_vel 沉默期非零末目标强制归零的超时 T（秒）；两侧同值 0.5
        # （nav2 20Hz 两侧同速率；巡航帧间隙 p99=0.053s，9.4× 裕量）
        self.declare_parameter('cmd_silence_timeout_s', 0.5)
        # true: 转发时对 angular.z/linear.y 取负（linear.x 不动）。
        # 适用 native 倒装图（map≡camera_init z-down）：base_footprint z 物理朝下，
        # nav2 +wz = 绕下向 z 轴 CCW = 物理右转，与底盘标准约定（+wz=左转）反号；
        # base ŷ = 物理右，底盘 +y = 左，同理。不取负则 yaw 闭环负增益 ⇒ 原地转圈。
        # 注意：仅命令路取负；/odom twist 反馈走 R_mount⁻¹ 同一 z-down 口径，不取负。
        self.declare_parameter('invert_cmd_vel_yz', False)

        self.body_to_footprint_z = self.get_parameter('body_to_footprint_z').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # mount 旋转：body(IMU) 相对 base 的安装 RPY；base = body ⊗ R_mount⁻¹
        _mount_rpy = self.get_parameter('mount_rpy').value
        self._R_mount_inv = Rotation.from_euler('xyz', _mount_rpy, degrees=True).inv()
        self._mount_active = any(abs(float(a)) > 1e-6 for a in _mount_rpy)

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
            self._rate_limiter = VelocityRateLimiter(
                max_ax, max_ay, max_az,
                max_decel=self.get_parameter('max_decel').value,
                max_decel_wz=self.get_parameter('max_decel_wz').value)
            self._cmd_silence_timeout = self.get_parameter('cmd_silence_timeout_s').value
            # P6b 沉默期状态（rclpy.spin 单线程：定时器与订阅回调天然串行，无锁）
            self._last_rx = None       # 最近一次 /cmd_vel 收帧时刻（秒，ROS clock）
            self._zero_flushed = False # 沉默期归零完成后是否已补发零帧
            self._last_published = None  # 最近一次发布 (vx, vy, wz)，定时器「值不变不发布」用
            self._cmd_vel_sub = self.create_subscription(
                Twist, cmd_vel_input, self._cmd_vel_relay_cb, 10
            )
            self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_output, 10)
            self._silence_timer = self.create_timer(
                SILENCE_TIMER_PERIOD, self._silence_timer_cb)

        self.get_logger().info(
            f'OdomBridge 启动:\n'
            f'  输入: {input_topic}\n'
            f'  输出TF: {self.odom_frame} -> {self.base_frame}\n'
            f'  输出话题: {output_topic}\n'
            f'  Z偏移: {self.body_to_footprint_z}m\n'
            f'  mount_rpy: {_mount_rpy}(deg){" [ACTIVE]" if self._mount_active else " [identity]"}'
            + (
                f'\n  cmd_vel relay: {cmd_vel_input} -> {cmd_vel_output}'
                f' (max_ax={self._rate_limiter.max_ax},'
                f' max_ay={self._rate_limiter.max_ay},'
                f' max_az={self._rate_limiter.max_az},'
                f' max_decel={self._rate_limiter.max_decel},'
                f' max_decel_wz={self._rate_limiter.max_decel_wz},'
                f' cmd_silence_timeout_s={self._cmd_silence_timeout})'
                f' invert_yz={self.get_parameter("invert_cmd_vel_yz").value}'
                f'{" [z-down 反号 ACTIVE]" if self.get_parameter("invert_cmd_vel_yz").value else ""}'
                if self._enable_cmd_vel_relay else '\n  cmd_vel relay: DISABLED'
            )
        )

    def _cmd_vel_relay_cb(self, msg: Twist):
        """cmd_vel 中继回调（帧流路径）：限幅后转发到 /cmd_vel_limiter

        本限幅器是链路唯一加速度权威（nav2 侧 MPPI 加速度死键已于 2026-11-08
        删除，勿再按「与 MPPI 对齐」选参）：加速向对称夹 max_ax/max_ay/max_az，
        减速向非对称夹 max_decel/max_decel_wz（P6a）。

        invert_cmd_vel_yz（真机 z-down 图设 true，实时可改，便于现场 A/B）：
          native 倒装图下 base_footprint z 物理朝下，nav2 的 +angular.z
          （绕下向 z 轴 CCW）= 物理右转，底盘标准约定 +wz = 左转 → 需取负；
          linear.y 同理（base ŷ=物理右、底盘 +y=左）；linear.x 不取负
          （mount −90 修正后 base x̂ = 机头）。取负在限幅之前（入口处先换到
          底盘约定，限幅器与 P6b 沉默 ramp 全程工作在底盘口径，出口不再过 invert）。

        沉默语义（P6b，取代旧「纯 pass-through 不合成零」契约）：
          帧流正常时平滑只随输入走（MPPI 收尾递减，限幅器跟帧）。输入沉默时
          本回调不再被调用，由 _silence_timer_cb 接管（快车道/超时归零 ramp）；
          本回调只同步沉默状态（_last_rx / _zero_flushed）。NSM 看门狗仍按
          /cmd_vel_limiter 心跳与 3s/90s 规则兜底 bridge 自身死亡。
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._last_rx = now_sec
        self._zero_flushed = False
        vx_in, vy_in, wz_in = msg.linear.x, msg.linear.y, msg.angular.z
        if self.get_parameter('invert_cmd_vel_yz').value:
            vy_in, wz_in = -vy_in, -wz_in
        vx, vy, wz = self._rate_limiter.limit(vx_in, vy_in, wz_in, now_sec)
        self._publish_limited(vx, vy, wz, src_msg=msg)

    def _publish_limited(self, vx, vy, wz, src_msg=None):
        """发布限幅后指令并记录 _last_published（沉默定时器「值不变不发布」用）。"""
        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.linear.z = src_msg.linear.z if src_msg is not None else 0.0   # passthrough (恒为 0)
        out.angular.x = src_msg.angular.x if src_msg is not None else 0.0  # passthrough (恒为 0)
        out.angular.y = src_msg.angular.y if src_msg is not None else 0.0  # passthrough (恒为 0)
        out.angular.z = wz
        self._cmd_vel_pub.publish(out)
        self._last_published = (float(vx), float(vy), float(wz))

    def _silence_timer_cb(self):
        """P6b 沉默期接管（20Hz 定时器；rclpy.spin 单线程与回调串行，无锁）。

        帧流正常（now-_last_rx < SILENCE_MARGIN）时本函数只读不写、直接返回，
        积分由 _cmd_vel_relay_cb 独占——不双积分。帧流沉默后：

        - 末目标≈0（取消/门控停正常形状：零帧收尾）→ 快车道：立即向零续刹，
          不等 T。落回 §4.1 数学（0.3→0 @0.6 = 0.5s / 7.5cm）；
        - 末目标≠0（真崩溃形状：沉默时末命令=巡航全速）→ T 之前保沉默不发布
          （AimRT 保末帧=原行为；NSM「末帧非零+沉默>3s→stand」backstop 原样
          保留）；T 到 → 末目标强制归零，转 ramp 按 max_decel/max_decel_wz 衰减；
        - 全分量 ≤0.01 → 补发 2 帧零、停发回沉默态（NSM「末帧零→90s」契约）；
        - ramp 期间收到新 /cmd_vel → 回调路径从衰减态无缝续积分（加速夹板接管）；
        - 值不变不发布，不污染 NSM 心跳语义。

        ⚠️ ramp 发布衰减态直接出口、不再过 invert：invert_cmd_vel_yz 在入口
        （_cmd_vel_relay_cb 限幅之前）已换到底盘口径，限幅器/ramp 全程底盘口径。
        若日后把 invert 挪到出口，此处中间帧 vy/wz 会反号（P6 审核 §3 实现注记）。
        """
        if self._last_rx is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_rx < SILENCE_MARGIN:
            return  # 帧流正常：cb 负责，勿双积分

        lim = self._rate_limiter
        # F4 守卫：sim /clock 前跳最多当 0.1s 用，防前跳虚假放大单步衰减
        dt = min(now - lim._last_time, SILENCE_DT_CAP)
        if dt < 1e-6:
            return
        if any(abs(t) > 1e-3 for t in lim._tgt):
            # 末目标非零 = 真崩溃形状：T 之前保沉默（AimRT 保末帧；NSM 3s backstop 原样）
            if now - self._last_rx < self._cmd_silence_timeout:
                return
            lim._tgt = (0.0, 0.0, 0.0)  # T 到 → 目标强制归零，转 ramp（死人开关）
            self._zero_flushed = False
        # 末目标≈0：续刹快车道（不等 T）——取消/门控停落回 §4.1 数学
        vx, vy, wz = lim._integrate(lim._tgt, dt, now)
        if all(abs(v) <= 0.01 for v in (vx, vy, wz)):
            if not self._zero_flushed:
                # 收敛完成补 2 帧零回沉默（NSM「末帧零→90s」契约；AimRT 保帧=零）
                self._publish_limited(0.0, 0.0, 0.0)
                self._publish_limited(0.0, 0.0, 0.0)
                self._zero_flushed = True
            return
        if self._last_published != (vx, vy, wz):
            self._publish_limited(vx, vy, wz)  # 值不变不发布：不污染 NSM 心跳语义

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

        # body(IMU) 在 camera_init 下的旋转
        body_rot = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
        # body→base 偏移：base 帧下 [0,0,z] 转到 body 帧 = R_mount⁻¹·[0,0,z]，再转世界
        local_offset = self._R_mount_inv.apply(
            np.array([0.0, 0.0, self.body_to_footprint_z]))
        world_offset = body_rot.apply(local_offset)

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

        
        # body(IMU)→base 朝向：扣除 mount 旋转 (base = body ⊗ R_mount⁻¹)，再 flatten 到 yaw
        # (Nav2 平面代价地图要求 base_footprint 只保留 Yaw)
        base_rot = body_rot * self._R_mount_inv
        euler = base_rot.as_euler('xyz', degrees=False)
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

        # 速度：FastLIO2 的 twist 在 body 系；转到 base 系 = R_mount⁻¹·twist_body，再 EMA
        # (不转的话 mount 有 yaw 偏移时 MPPI 会拿到横向速度)
        raw_tw = msg.twist.twist
        lin_base = self._R_mount_inv.apply(
            np.array([raw_tw.linear.x, raw_tw.linear.y, raw_tw.linear.z]))
        ang_base = self._R_mount_inv.apply(
            np.array([raw_tw.angular.x, raw_tw.angular.y, raw_tw.angular.z]))

        # EMA 平滑滤波 (过滤高频抖动)
        self.filtered_twist_linear_x = self.alpha_v * lin_base[0] + (1 - self.alpha_v) * self.filtered_twist_linear_x
        self.filtered_twist_linear_y = self.alpha_v * lin_base[1] + (1 - self.alpha_v) * self.filtered_twist_linear_y
        self.filtered_twist_linear_z = self.alpha_v * lin_base[2] + (1 - self.alpha_v) * self.filtered_twist_linear_z
        self.filtered_twist_angular_x = self.alpha_v * ang_base[0] + (1 - self.alpha_v) * self.filtered_twist_angular_x
        self.filtered_twist_angular_y = self.alpha_v * ang_base[1] + (1 - self.alpha_v) * self.filtered_twist_angular_y
        self.filtered_twist_angular_z = self.alpha_v * ang_base[2] + (1 - self.alpha_v) * self.filtered_twist_angular_z

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
