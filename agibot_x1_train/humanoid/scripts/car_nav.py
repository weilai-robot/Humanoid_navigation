"""
car_nav.py  —  小车 mid360 LiDAR MuJoCo仿真 + ROS2 导航桥接

运行方式:
    python car_nav.py

订阅:
    /cmd_vel  (geometry_msgs/Twist)  —— 导航速度命令 (teleop 或 Nav2)

发布:
    /livox/lidar  (livox_ros_driver2/CustomMsg)  —— FAST_LIO2 建图/定位
    /imu/data     (sensor_msgs/Imu)              —— FAST_LIO2 建图/定位
    /clock        (rosgraph_msgs/Clock)          —— 仿真时间源 (use_sim_time=True)
    /tf           odom→base_link (动态), base_link→lidar_link (静态)

依赖:
    pip install -e <workspace>/src/MuJoCo-LiDAR/
    colcon build (livox_ros_driver2)
"""

import os
import queue as _queue
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

# --- MuJoCo-LiDAR 路径 (与 sim2sim_nav.py 同级 src 目录) ---
from humanoid import LEGGED_GYM_ROOT_DIR   # agibot_x1_train/humanoid/

_MUJOCO_LIDAR_SRC = os.path.normpath(
    os.path.join(LEGGED_GYM_ROOT_DIR, '..', 'MuJoCo-LiDAR', 'src')
)
if _MUJOCO_LIDAR_SRC not in sys.path:
    sys.path.insert(0, _MUJOCO_LIDAR_SRC)
from mujoco_lidar import MjLidarWrapper, scan_gen

# --- 小车 MJCF 路径 ---
_CAR_XML = os.path.normpath(
    os.path.join(LEGGED_GYM_ROOT_DIR, 'resources', 'car', 'mjcf', 'car_nav.xml')
)

# --- Livox CustomMsg（优先），回退到 PointCloud2 ---
try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    _USE_CUSTOM_MSG = True
    print("[INFO] 使用 livox_ros_driver2/CustomMsg → FAST_LIO2 lidar_type: 1")
except ImportError:
    from sensor_msgs.msg import PointCloud2, PointField
    _USE_CUSTOM_MSG = False
    print("[WARN] livox_ros_driver2 不可用，回退 PointCloud2 → FAST_LIO2 lidar_type: 2")

# --- 仿真时间戳辅助 ---
def _mj_to_stamp(t: float):
    sec     = int(t)
    nanosec = int((t - sec) * 1_000_000_000)
    from builtin_interfaces.msg import Time as _T
    s = _T(); s.sec = sec; s.nanosec = nanosec
    return s

# --- 全局速度命令（由 /cmd_vel 回调写入）---
x_vel_cmd, y_vel_cmd, yaw_vel_cmd = 0.0, 0.0, 0.0

# --- 仿真参数 ---
_LIDAR_HZ          = 10           # 与 car_mid360.yaml scan_rate 一致
_IMU_HZ            = 200
_LIDAR_DOWNSAMPLE  = 10           # 24000 / 10 = 2400 pts/frame
_LIDAR_FRAME_NS    = 1_000_000    # 1ms：瞬时采样，不做畸变矫正
_SIM_DURATION      = 3600.0       # 最大仿真时长 (s)

# lidar_site 相对于 car body 中心的 Z 偏移 (m)，用于 base_link→lidar_link 静态 TF
# car body 中心 z=0.10m，lidar 绝对高度 0.20m → 相对 0.10m
_LIDAR_HEIGHT_REL  = 0.10


# ═══════════════════════════════════════════════════════════
#  ROS2 节点
# ═══════════════════════════════════════════════════════════
class CarNavNode(Node):
    """订阅 /cmd_vel，发布 LiDAR、IMU、Clock，广播 TF"""

    def __init__(self):
        super().__init__("car_mujoco_nav")

        self.tf_broadcaster     = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.clock_pub = self.create_publisher(Clock, "/clock",    10)
        self.imu_pub   = self.create_publisher(Imu,   "/imu/data", 10)

        if _USE_CUSTOM_MSG:
            self.lidar_pub = self.create_publisher(CustomMsg,   "/livox/lidar", 10)
        else:
            self.lidar_pub = self.create_publisher(PointCloud2, "/livox/lidar", 10)

        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self._prev_vel_world = np.zeros(3)  # 上一步世界系线速度，用于计算 a = dv/dt
        self._prev_imu_time  = -1.0          # 上次 IMU 发布的仿真时间，-1 表示首次

    # ── /cmd_vel 回调 ──
    def _cmd_vel_cb(self, msg: Twist):
        global x_vel_cmd, y_vel_cmd, yaw_vel_cmd
        x_vel_cmd   = msg.linear.x
        y_vel_cmd   = msg.linear.y
        yaw_vel_cmd = msg.angular.z

    # ── 仿真时钟 ──
    def publish_clock(self, sim_time: float):
        msg = Clock()
        msg.clock = _mj_to_stamp(sim_time)
        self.clock_pub.publish(msg)

    # ── 静态 TF: base_link → lidar_link (正装, identity 旋转) ──
    def publish_static_tf(self):
        stamp = self.get_clock().now().to_msg()
        stf = TransformStamped()
        stf.header.stamp    = stamp
        stf.header.frame_id = "base_link"
        stf.child_frame_id  = "lidar_link"
        stf.transform.translation.x = 0.0
        stf.transform.translation.y = 0.0
        stf.transform.translation.z = _LIDAR_HEIGHT_REL   # 0.10 m
        stf.transform.rotation.x    = 0.0
        stf.transform.rotation.y    = 0.0
        stf.transform.rotation.z    = 0.0
        stf.transform.rotation.w    = 1.0
        self.static_broadcaster.sendTransform([stf])

    # ── 动态 TF: odom → base_link (地面真实位姿, 从 freejoint 读取) ──
    def publish_odom_tf(self, mj_data: mujoco.MjData, sim_time: float):
        stamp = _mj_to_stamp(sim_time)
        # freejoint qpos: [x, y, z, qw, qx, qy, qz]
        tf = TransformStamped()
        tf.header.stamp        = stamp
        tf.header.frame_id     = "odom"
        tf.child_frame_id      = "base_link"
        tf.transform.translation.x = float(mj_data.qpos[0])
        tf.transform.translation.y = float(mj_data.qpos[1])
        tf.transform.translation.z = float(mj_data.qpos[2])
        tf.transform.rotation.w    = float(mj_data.qpos[3])
        tf.transform.rotation.x    = float(mj_data.qpos[4])
        tf.transform.rotation.y    = float(mj_data.qpos[5])
        tf.transform.rotation.z    = float(mj_data.qpos[6])
        self.tf_broadcaster.sendTransform(tf)

    # ── IMU 发布 ──
    def publish_imu(self, mj_data: mujoco.MjData, sim_time: float, dt: float):
        stamp     = _mj_to_stamp(sim_time)
        quat_wxyz = mj_data.sensor("lidar-orientation").data.copy()
        omega     = mj_data.sensor("lidar-angular-velocity").data.copy()

        # ── 运动学加速度覆盖 ──
        # 直接注入 qvel 时 MuJoCo 物理力为零，accelerometer 传感器读不到水平加速度。
        # 正确做法：用 dv/dt 计算真实加速度，旋转到 IMU site 坐标系，再加重力补偿。
        # MuJoCo accelerometer 约定: a_measured = a_site_local - g_local
        # → a_measured = R_site^T * (a_true_world - g_world)
        # 注意：_prev_vel_world 每次 IMU 发布才更新，时间间隔为 imu_period*dt，
        # 必须用实际仿真时间差分，否则加速度虚高 imu_period 倍。
        vel_world = mj_data.qvel[0:3].copy()
        if self._prev_imu_time < 0.0:
            actual_dt = dt
        else:
            actual_dt = sim_time - self._prev_imu_time
            if actual_dt <= 0.0:
                actual_dt = dt
        self._prev_imu_time = sim_time
        a_true_world = (vel_world - self._prev_vel_world) / actual_dt
        self._prev_vel_world[:] = vel_world

        R_site  = mj_data.site("lidar_imu").xmat.reshape(3, 3)
        g_world = np.array([0.0, 0.0, -9.81])
        accel   = R_site.T @ (a_true_world - g_world)   # site-frame IMU reading

        msg = Imu()
        msg.header.stamp    = stamp
        msg.header.frame_id = "lidar_link"
        msg.orientation.w   = float(quat_wxyz[0])
        msg.orientation.x   = float(quat_wxyz[1])
        msg.orientation.y   = float(quat_wxyz[2])
        msg.orientation.z   = float(quat_wxyz[3])
        msg.angular_velocity.x    = float(omega[0])
        msg.angular_velocity.y    = float(omega[1])
        msg.angular_velocity.z    = float(omega[2])
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        self.imu_pub.publish(msg)

    # ── LiDAR 发布 (CustomMsg) ──
    def _publish_lidar_custom(self, points_local: np.ndarray, sim_time: float):
        valid_mask = np.linalg.norm(points_local, axis=1) > 0.01
        pts = points_local[valid_mask]
        n   = len(pts)
        if n == 0:
            return

        stamp       = _mj_to_stamp(sim_time)
        timebase_ns = stamp.sec * int(1e9) + stamp.nanosec

        msg = CustomMsg()
        msg.header.stamp    = stamp
        msg.header.frame_id = "lidar_link"
        msg.timebase        = timebase_ns
        msg.point_num       = n
        msg.lidar_id        = 0
        for i in range(n):
            cp              = CustomPoint()
            cp.offset_time  = int(i / n * _LIDAR_FRAME_NS)
            cp.x            = float(pts[i, 0])
            cp.y            = float(pts[i, 1])
            cp.z            = float(pts[i, 2])
            cp.reflectivity = 100
            cp.tag          = 0
            cp.line         = 0
            msg.points.append(cp)
        self.lidar_pub.publish(msg)

    # ── LiDAR 发布 (PointCloud2 回退) ──
    def _publish_lidar_pc2(self, points_local: np.ndarray, sim_time: float):
        from sensor_msgs.msg import PointCloud2, PointField
        stamp  = _mj_to_stamp(sim_time)
        fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg = PointCloud2()
        msg.header.stamp    = stamp
        msg.header.frame_id = "lidar_link"
        msg.fields          = fields
        msg.is_bigendian    = False
        msg.point_step      = 12
        msg.height          = 1
        msg.width           = len(points_local)
        msg.row_step        = msg.point_step * msg.width
        msg.is_dense        = True
        msg.data = np.ascontiguousarray(points_local, dtype=np.float32).tobytes()
        self.lidar_pub.publish(msg)

    def publish_lidar(self, points_local: np.ndarray, sim_time: float):
        if _USE_CUSTOM_MSG:
            self._publish_lidar_custom(points_local, sim_time)
        else:
            self._publish_lidar_pc2(points_local, sim_time)


# ═══════════════════════════════════════════════════════════
#  主仿真循环
# ═══════════════════════════════════════════════════════════
def run_car_sim(ros_node: CarNavNode):
    print(f"[INFO] 加载 MuJoCo XML: {_CAR_XML}")
    model = mujoco.MjModel.from_xml_path(_CAR_XML)
    data  = mujoco.MjData(model)
    mujoco.mj_step(model, data)   # 初始化接触

    # ── LiDAR 初始化 ──
    # 车体现在对 LiDAR 可见（真实环境同理）
    # 车体点云距传感器 < 0.2m，由 FAST_LIO2 的 blind=0.3m 自动过滤
    geomgroup = np.ones((mujoco.mjNGROUP,), dtype=np.ubyte)
    lidar = MjLidarWrapper(
        model,
        site_name="lidar_site",
        backend="cpu",
        cutoff_dist=25.0,
        args={"geomgroup": geomgroup},
    )
    livox_gen = scan_gen.LivoxGenerator("mid360")

    # ── 后台 LiDAR 线程 ──
    _lidar_q    = _queue.Queue(maxsize=1)
    _lidar_stop = threading.Event()

    def _lidar_worker():
        while not _lidar_stop.is_set():
            try:
                snap = _lidar_q.get(timeout=0.5)
            except _queue.Empty:
                continue
            rays_theta, rays_phi = livox_gen.sample_ray_angles(
                downsample=_LIDAR_DOWNSAMPLE
            )
            lidar.trace_rays(snap, rays_theta, rays_phi)
            pts = lidar.get_hit_points()
            if len(pts) > 0:
                ros_node.publish_lidar(pts, snap.time)

    lidar_thread = threading.Thread(target=_lidar_worker, daemon=True)
    lidar_thread.start()

    # ── 三缓冲区 snap ──
    _snap_pool = [mujoco.MjData(model) for _ in range(3)]
    _snap_idx  = [0]

    # ── 发布静态 TF ──
    ros_node.publish_static_tf()

    # ── 周期计算 ──
    dt            = model.opt.timestep

    # ── 实时限速：防止仿真跑得比实时快导致 FAST_LIO2 漂移 ──
    _wall_start = time.perf_counter()
    _sim_start  = 0.0  # data.time 初始值，循环中赋值

    lidar_period  = int(round(1.0 / (_LIDAR_HZ * dt)))
    imu_period    = int(round(1.0 / (_IMU_HZ   * dt)))
    viewer_period = int(round(1.0 / (30.0       * dt)))
    total_steps   = int(_SIM_DURATION / dt)

    print("[INFO] 小车仿真启动。")
    print("[INFO]   /cmd_vel → 速度命令 (或 teleop_twist_keyboard)")
    print("[INFO]   FAST_LIO2 → car_mid360.yaml")
    print(f"[INFO]   XML: {_CAR_XML}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type        = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = model.body("car").id

        _sim_start  = data.time
        _wall_start = time.perf_counter()

        for step in range(total_steps):
            if not viewer.is_running():
                break

            # ── 速度控制：读取当前 Yaw，将体系速度转到世界系 ──
            # freejoint qpos: [x, y, z, qw, qx, qy, qz]
            qw = float(data.qpos[3])
            qx = float(data.qpos[4])
            qy = float(data.qpos[5])
            qz = float(data.qpos[6])
            yaw = np.arctan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))

            vx_w = x_vel_cmd * np.cos(yaw) - y_vel_cmd * np.sin(yaw)
            vy_w = x_vel_cmd * np.sin(yaw) + y_vel_cmd * np.cos(yaw)

            # ── 直接赋値命令速度：IMU 加速度已由 actual_dt 正确计算，无需每步限幅 ──
            data.qvel[0] = vx_w          # 世界系 X 线速度
            data.qvel[1] = vy_w          # 世界系 Y 线速度
            data.qvel[2] = 0.0           # 无垂直运动
            data.qvel[3] = 0.0           # 无 Roll
            data.qvel[4] = 0.0           # 无 Pitch
            data.qvel[5] = yaw_vel_cmd   # 体系 Yaw 角速度

            mujoco.mj_step(model, data)

            # ── 实时限速：仿真时间不超过挂钟时间 ──
            sim_elapsed  = data.time - _sim_start
            wall_elapsed = time.perf_counter() - _wall_start
            if sim_elapsed > wall_elapsed:
                time.sleep(sim_elapsed - wall_elapsed)

            if step % viewer_period == 0:
                viewer.sync()

            # ── 时钟 + IMU + odom TF ──
            ros_node.publish_clock(data.time)
            if step % imu_period == 0:
                ros_node.publish_imu(data, data.time, dt)
                ros_node.publish_odom_tf(data, data.time)

            # ── LiDAR 触发 ──
            if step % lidar_period == 0 and _lidar_q.empty():
                wi = _snap_idx[0]
                _snap_idx[0] = (wi + 1) % 3
                snap = _snap_pool[wi]
                snap.geom_xpos[:] = data.geom_xpos
                snap.geom_xmat[:] = data.geom_xmat
                snap.site_xpos[:] = data.site_xpos
                snap.site_xmat[:] = data.site_xmat
                snap.time         = data.time
                _lidar_q.put_nowait(snap)

    _lidar_stop.set()
    lidar_thread.join(timeout=2.0)


# ═══════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════
if __name__ == '__main__':
    rclpy.init()
    ros_node    = CarNavNode()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    spin_thread.start()

    try:
        run_car_sim(ros_node)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
