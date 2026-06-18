"""
sim2sim_nav.py  —  X1 RL行走 + mid360 LiDAR MuJoCo仿真 + ROS2导航桥接

运行方式:
    python sim2sim_nav.py --task x1_dh_stand

订阅:
    /cmd_vel  (geometry_msgs/Twist)  —— Nav2 导航速度命令

发布:
    /livox/lidar  (livox_ros_driver2/CustomMsg)  —— FAST_LIO2 建图/定位
    /imu/data     (sensor_msgs/Imu)              —— FAST_LIO2 建图/定位
    /clock        (rosgraph_msgs/Clock)          —— 仿真时间源，供所有 use_sim_time=True 节点使用
    /tf           odom → base_link (动态), base_link → lidar_link (静态)

依赖:
    pip install -e <workspace>/src/MuJoCo-LiDAR/
    colcon build (livox_ros_driver2)
"""

import importlib.util
import math
import os
import queue as _queue
import sys
import threading
import types

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import tf2_ros
import torch
from collections import deque
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

from humanoid import LEGGED_GYM_ROOT_DIR

# 用 importlib 直接加载配置文件，绕过 envs/__init__.py 的训练依赖链
# （避免 isaacgym / wandb / tensorboard 等训练包的 ImportError）
def _load_cfg_module():
    root = LEGGED_GYM_ROOT_DIR
    for _pkg in ('humanoid.envs', 'humanoid.envs.base', 'humanoid.envs.x1'):
        if _pkg not in sys.modules:
            sys.modules[_pkg] = types.ModuleType(_pkg)

    def _load(mod_name, rel_path):
        fp = os.path.join(root, rel_path)
        spec = importlib.util.spec_from_file_location(mod_name, fp)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[mod_name] = mod
        spec.loader.exec_module(mod)
        return mod

    _load('humanoid.envs.base.base_config',
          'humanoid/envs/base/base_config.py')
    _load('humanoid.envs.base.legged_robot_config',
          'humanoid/envs/base/legged_robot_config.py')
    cfg_mod = _load('humanoid.envs.x1.x1_dh_stand_config',
                    'humanoid/envs/x1/x1_dh_stand_config.py')
    return cfg_mod

_cfg_mod = _load_cfg_module()
X1DHStandCfg = _cfg_mod.X1DHStandCfg

# --- MuJoCo-LiDAR 路径 ---
_MUJOCO_LIDAR_SRC = os.path.normpath(
    os.path.join(LEGGED_GYM_ROOT_DIR, '..', 'MuJoCo-LiDAR', 'src')
)
if _MUJOCO_LIDAR_SRC not in sys.path:
    sys.path.insert(0, _MUJOCO_LIDAR_SRC)
from mujoco_lidar import MjLidarWrapper, scan_gen

# --- Livox CustomMsg（优先），回退到 PointCloud2 ---
try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    _USE_CUSTOM_MSG = True
    print("[INFO] 使用 livox_ros_driver2/CustomMsg 发布点云 → FAST_LIO2 lidar_type: 1")
except ImportError:
    from sensor_msgs.msg import PointCloud2, PointField
    _USE_CUSTOM_MSG = False
    print("[WARN] livox_ros_driver2 不可用，回退 PointCloud2 → FAST_LIO2 lidar_type: 2")

# --- 仿真时间戳辅助 ---
def _mj_to_stamp(t: float):
    """MuJoCo 仿真时间(秒) → ROS2 Time msg"""
    sec = int(t)
    nanosec = int((t - sec) * 1_000_000_000)
    from builtin_interfaces.msg import Time as _T
    s = _T()
    s.sec = sec
    s.nanosec = nanosec
    return s

# --- 速度命令（由 /cmd_vel 回调更新）---
x_vel_cmd, y_vel_cmd, yaw_vel_cmd = 0.0, 0.0, 0.0

_LIDAR_HZ              = 10          # 20Hz：缩短帧间间隔，减少转弯时相邻帧旋转量 ！！！
_IMU_HZ                = 200
_LIDAR_DOWNSAMPLE      = 10         # 24000/10 = 2400 pts/frame，平衡速度与精度
_LIDAR_POINTS_PER_FRAME = 24000 // _LIDAR_DOWNSAMPLE
_LIDAR_FRAME_NS        = 1_000_000   # 1ms：点云瞬时采样，不做畸变矫正


# ═══════════════════════════════════════════════════════════
#  ROS2 节点
# ═══════════════════════════════════════════════════════════
class X1NavNode(Node):
    """订阅 /cmd_vel，发布 LiDAR、IMU，广播 odom→base_link TF"""

    def __init__(self, mj_model: mujoco.MjModel):
        super().__init__("x1_mujoco_nav")
        self.mj_model = mj_model

        self.tf_broadcaster     = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._static_tf_sent    = False

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)

        if _USE_CUSTOM_MSG:
            self.lidar_pub = self.create_publisher(CustomMsg, "/livox/lidar", 10)
        else:
            self.lidar_pub = self.create_publisher(PointCloud2, "/livox/lidar", 10)

        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

    def _cmd_vel_cb(self, msg: Twist):
        global x_vel_cmd, y_vel_cmd, yaw_vel_cmd
        x_vel_cmd   = msg.linear.x
        y_vel_cmd   = msg.linear.y
        yaw_vel_cmd = msg.angular.z

    # ── 仿真时钟发布 ──
    def publish_clock(self, sim_time: float):
        msg = Clock()
        msg.clock = _mj_to_stamp(sim_time)
        self.clock_pub.publish(msg)

    # ── 静态 TF: base_footprint → base_link  &&  base_link → lidar_link ──
    # odom_bridge.py 已发布 odom→base_footprint，本函数补全后两跳：
    #   odom → base_footprint → base_link → lidar_link
    # base_footprint 与 base_link 共位（identity），lidar 在地面以上 1.31m 处
    def publish_static_tf(self, mj_data: mujoco.MjData):
        stamp = self.get_clock().now().to_msg()

        # base_footprint → base_link (identity)
        stf_bf2bl = TransformStamped()
        stf_bf2bl.header.stamp    = stamp
        stf_bf2bl.header.frame_id = "base_footprint"
        stf_bf2bl.child_frame_id  = "base_link"
        stf_bf2bl.transform.translation.x = 0.0
        stf_bf2bl.transform.translation.y = 0.0
        stf_bf2bl.transform.translation.z = 0.0
        stf_bf2bl.transform.rotation.x    = 0.0
        stf_bf2bl.transform.rotation.y    = 0.0
        stf_bf2bl.transform.rotation.z    = 0.0
        stf_bf2bl.transform.rotation.w    = 1.0

        # base_link → lidar_link
        # odom_bridge 将 base_link 置于 LiDAR 高度（map z≈0），z 无需再偏移
        # 倒装安装 Ry(180°): quat (x,y,z,w) = (0, 1, 0, 0)
        stf_bl2ll = TransformStamped()
        stf_bl2ll.header.stamp    = stamp
        stf_bl2ll.header.frame_id = "base_link"
        stf_bl2ll.child_frame_id  = "lidar_link"
        stf_bl2ll.transform.translation.x = 0.05
        stf_bl2ll.transform.translation.y = 0.0
        stf_bl2ll.transform.translation.z = 0.0
        stf_bl2ll.transform.rotation.x    = 0.0
        stf_bl2ll.transform.rotation.y    = 1.0
        stf_bl2ll.transform.rotation.z    = 0.0
        stf_bl2ll.transform.rotation.w    = 0.0

        self.static_broadcaster.sendTransform([stf_bf2bl, stf_bl2ll])
        self._static_tf_sent = True

    # ── 动态 TF: odom → base_link ──
    def _publish_odom_tf(self, mj_data: mujoco.MjData, stamp):
        pos  = mj_data.site("imu").xpos.copy()
        mat  = mj_data.site("imu").xmat.reshape(3, 3).copy()
        quat = R.from_matrix(mat).as_quat()  # [x,y,z,w]

        tf = TransformStamped()
        tf.header.stamp        = stamp
        tf.header.frame_id     = "odom"
        tf.child_frame_id      = "base_link"
        tf.transform.translation.x = float(pos[0])
        tf.transform.translation.y = float(pos[1])
        tf.transform.translation.z = float(pos[2])
        tf.transform.rotation.x    = float(quat[0])
        tf.transform.rotation.y    = float(quat[1])
        tf.transform.rotation.z    = float(quat[2])
        tf.transform.rotation.w    = float(quat[3])
        self.tf_broadcaster.sendTransform(tf)

    # ── IMU 发布（使用 mid360 内置 lidar_imu 传感器，独立于 RL 控制的 body-* 传感器）──
    def publish_imu(self, mj_data: mujoco.MjData, sim_time: float):
        stamp     = _mj_to_stamp(sim_time)
        quat_wxyz = mj_data.sensor("lidar-orientation").data.copy()
        omega     = mj_data.sensor("lidar-angular-velocity").data.copy()
        accel     = mj_data.sensor("lidar-linear-acceleration").data.copy()

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
        # odom→base_link 由 FAST_LIO2 body 链推导，不在此处发布

    # ── LiDAR 发布（CustomMsg，含逐点 offset_time）──
    def _publish_lidar_custom(self, points_local: np.ndarray, sim_time: float):
        valid_mask  = np.linalg.norm(points_local, axis=1) > 0.01
        pts         = points_local[valid_mask]
        n           = len(pts)
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

    # ── LiDAR 发布（PointCloud2 回退）──
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
        msg.fields      = fields
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.height       = 1
        msg.width        = len(points_local)
        msg.row_step     = msg.point_step * msg.width
        msg.is_dense     = True
        msg.data         = np.ascontiguousarray(points_local, dtype=np.float32).tobytes()
        self.lidar_pub.publish(msg)

    def publish_lidar(self, points_local: np.ndarray, sim_time: float):
        if _USE_CUSTOM_MSG:
            self._publish_lidar_custom(points_local, sim_time)
        else:
            self._publish_lidar_pc2(points_local, sim_time)


# ═══════════════════════════════════════════════════════════
#  工具函数（保留自 sim2sim.py）
# ═══════════════════════════════════════════════════════════
def quaternion_to_euler_array(quat):
    x, y, z, w = quat
    roll_x  = np.arctan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    pitch_y = np.arcsin(np.clip(2.0*(w*y - z*x), -1.0, 1.0))
    yaw_z   = np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    return np.array([roll_x, pitch_y, yaw_z])


def pd_control(target_q, q, kp, target_dq, dq, kd, default_dof_pos):
    return (target_q + default_dof_pos - q) * kp - dq * kd


# ═══════════════════════════════════════════════════════════
#  主仿真循环
# ═══════════════════════════════════════════════════════════
def run_mujoco_nav(policy, cfg, env_cfg, ros_node: X1NavNode):
    print("Load mujoco xml from:", cfg.sim_config.mujoco_model_path)
    model = mujoco.MjModel.from_xml_path(cfg.sim_config.mujoco_model_path)
    model.opt.timestep = cfg.sim_config.dt
    data = mujoco.MjData(model)

    num_actions = env_cfg.env.num_actions
    data.qpos[-num_actions:] = cfg.robot_config.default_dof_pos
    mujoco.mj_step(model, data)

    # ── LiDAR 初始化（CPU后端，无需GPU）──
    geomgroup = np.ones((mujoco.mjNGROUP,), dtype=np.ubyte)
    geomgroup[3] = 0   # 排除碰撞几何组，避免与视觉网格重复检测
    lidar = MjLidarWrapper(
        model,
        site_name="lidar_site",
        backend="cpu",
        cutoff_dist=30.0,
        args={"geomgroup": geomgroup},
    )
    livox_gen = scan_gen.LivoxGenerator("mid360")

    # ── 后台 LiDAR 线程：异步射线追踪，不阻塞主仿真循环 ──
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
                hits = int(np.sum(np.linalg.norm(pts, axis=1) > 0.3))
                if hits < 10:
                    print(f"[LIDAR WARN] valid hits={hits}/{len(pts)}")
                ros_node.publish_lidar(pts, snap.time)

    _lidar_thread = threading.Thread(target=_lidar_worker, daemon=True)
    _lidar_thread.start()

    # ── 三缓冲区 snap，避免线程使用期间主循环覆写 ──
    _snap_pool = [mujoco.MjData(model) for _ in range(3)]
    _snap_write_idx = [0]

    # ── 观测历史队列 ──
    hist_obs = deque()
    for _ in range(env_cfg.env.frame_stack):
        hist_obs.append(np.zeros([1, env_cfg.env.num_single_obs], dtype=np.double))

    target_q = np.zeros(num_actions, dtype=np.double)
    action   = np.zeros(num_actions, dtype=np.double)
    count_lowlevel = 1

    lidar_period  = int(1.0 / (_LIDAR_HZ * cfg.sim_config.dt))
    imu_period    = int(1.0 / (_IMU_HZ   * cfg.sim_config.dt))
    viewer_period = int(1.0 / (30.0     * cfg.sim_config.dt))  # 30Hz 显示帧率

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type       = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = model.body("x1-body").id

        ros_node.publish_static_tf(data)
        print("[INFO] 仿真启动。通过 /cmd_vel 发送导航命令。")

        for step in range(int(cfg.sim_config.sim_duration / cfg.sim_config.dt)):
            if not viewer.is_running():
                break

            # ── RL 策略推理 (100Hz) ──
            if count_lowlevel % cfg.sim_config.decimation == 0:
                if hasattr(env_cfg.commands, 'sw_switch'):
                    vel_norm = math.sqrt(x_vel_cmd**2 + y_vel_cmd**2 + yaw_vel_cmd**2)
                    if env_cfg.commands.sw_switch and vel_norm <= env_cfg.commands.stand_com_threshold:
                        count_lowlevel = 0

                q  = data.qpos[-num_actions:].astype(np.double)
                dq = data.qvel[-num_actions:].astype(np.double)

                quat   = data.sensor("body-orientation").data[[1, 2, 3, 0]].astype(np.double)
                omega  = data.sensor("body-angular-velocity").data.astype(np.double)
                eu_ang = quaternion_to_euler_array(quat)
                eu_ang[eu_ang > math.pi] -= 2 * math.pi

                obs = np.zeros([1, env_cfg.env.num_single_obs], dtype=np.float32)
                nc  = env_cfg.env.num_commands
                na  = num_actions

                if nc == 5:
                    t = count_lowlevel * cfg.sim_config.dt
                    obs[0, 0] = math.sin(2*math.pi*t / env_cfg.rewards.cycle_time)
                    obs[0, 1] = math.cos(2*math.pi*t / env_cfg.rewards.cycle_time)
                    obs[0, 2] = x_vel_cmd   * env_cfg.normalization.obs_scales.lin_vel
                    obs[0, 3] = y_vel_cmd   * env_cfg.normalization.obs_scales.lin_vel
                    obs[0, 4] = yaw_vel_cmd * env_cfg.normalization.obs_scales.ang_vel
                elif nc == 3:
                    obs[0, 0] = x_vel_cmd   * env_cfg.normalization.obs_scales.lin_vel
                    obs[0, 1] = y_vel_cmd   * env_cfg.normalization.obs_scales.lin_vel
                    obs[0, 2] = yaw_vel_cmd * env_cfg.normalization.obs_scales.ang_vel

                obs[0, nc:nc+na]             = (q - cfg.robot_config.default_dof_pos) * env_cfg.normalization.obs_scales.dof_pos
                obs[0, nc+na:nc+2*na]        = dq * env_cfg.normalization.obs_scales.dof_vel
                obs[0, nc+2*na:nc+3*na]      = action
                obs[0, nc+3*na:nc+3*na+3]    = omega
                obs[0, nc+3*na+3:nc+3*na+6]  = eu_ang

                if getattr(env_cfg.env, 'add_stand_bool', False):
                    vel_norm = math.sqrt(x_vel_cmd**2 + y_vel_cmd**2 + yaw_vel_cmd**2)
                    obs[0, -1] = float(vel_norm <= env_cfg.commands.stand_com_threshold)

                obs = np.clip(obs, -env_cfg.normalization.clip_observations,
                              env_cfg.normalization.clip_observations)
                hist_obs.append(obs)
                hist_obs.popleft()

                policy_input = np.zeros([1, env_cfg.env.num_observations], dtype=np.float32)
                for i in range(env_cfg.env.frame_stack):
                    s = i * env_cfg.env.num_single_obs
                    policy_input[0, s:s+env_cfg.env.num_single_obs] = hist_obs[i][0, :]

                action[:] = policy(torch.tensor(policy_input))[0].detach().numpy()
                action = np.clip(action, -env_cfg.normalization.clip_actions,
                                 env_cfg.normalization.clip_actions)
                target_q = action * env_cfg.control.action_scale

            # ── PD 力矩控制 ──
            q  = data.qpos[-num_actions:].astype(np.double)
            dq = data.qvel[-num_actions:].astype(np.double)
            tau = pd_control(target_q, q, cfg.robot_config.kps,
                             np.zeros(num_actions), dq, cfg.robot_config.kds,
                             cfg.robot_config.default_dof_pos)
            tau = np.clip(tau, -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)
            data.ctrl[:] = tau

            mujoco.mj_step(model, data)
            if step % viewer_period == 0:
                viewer.sync()

            # ── 仿真时钟发布 (每步) + IMU 发布 (200Hz) ──
            ros_node.publish_clock(data.time)
            if step % imu_period == 0:
                ros_node.publish_imu(data, data.time)

            # ── LiDAR 触发 (10Hz)：三缓冲区交替使用，避免线程读取时主循环覆写 ──
            if step % lidar_period == 0 and _lidar_q.empty():
                wi = _snap_write_idx[0]
                _snap_write_idx[0] = (wi + 1) % 3
                snap = _snap_pool[wi]
                snap.geom_xpos[:] = data.geom_xpos
                snap.geom_xmat[:] = data.geom_xmat
                snap.site_xpos[:] = data.site_xpos
                snap.site_xmat[:] = data.site_xmat
                snap.time         = data.time      # 记录仿真时间供 LiDAR 时间戳使用
                _lidar_q.put_nowait(snap)

            count_lowlevel += 1


# ═══════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='X1 MuJoCo 导航仿真')
    parser.add_argument('--task', type=str, required=True,
                        help='任务名称，例如 x1_dh_stand')
    parser.add_argument('--model_path', type=str, required=True,
                        help='TorchScript 策略文件路径（.jit）')
    args = parser.parse_args()

    env_cfg = X1DHStandCfg()

    class Sim2simNavCfg:
        class sim_config:
            mujoco_model_path = os.path.join(
                LEGGED_GYM_ROOT_DIR,
                'resources', 'robots', 'x1', 'mjcf', 'xyber_x1_nav.xml'
            )
            sim_duration = 3600.0
            dt           = 0.001
            decimation   = 10

        class robot_config:
            kps = np.array(
                [env_cfg.control.stiffness[j] for j in env_cfg.control.stiffness] * 2,
                dtype=np.double
            )
            kds = np.array(
                [env_cfg.control.damping[j] for j in env_cfg.control.damping] * 2,
                dtype=np.double
            )
            tau_limit       = 500.0 * np.ones(env_cfg.env.num_actions, dtype=np.double)
            default_dof_pos = np.array(list(env_cfg.init_state.default_joint_angles.values()))

    # ── 加载 TorchScript 策略 ──
    policy = torch.jit.load(args.model_path)
    print("Load model from:", args.model_path)

    # ── 初始化 ROS2 ──
    rclpy.init()
    mj_model_tmp = mujoco.MjModel.from_xml_path(Sim2simNavCfg.sim_config.mujoco_model_path)
    ros_node = X1NavNode(mj_model_tmp)
    spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    spin_thread.start()

    try:
        run_mujoco_nav(policy, Sim2simNavCfg(), env_cfg, ros_node)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
