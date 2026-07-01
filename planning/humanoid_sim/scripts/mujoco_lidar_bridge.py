#!/usr/bin/env python3
"""
mujoco_lidar_bridge.py — MuJoCo LiDAR 射线追踪桥接节点 (事件驱动)

架构:
  sim_module (C++/aimrt_main) 做物理仿真 + RL 控制 (真机一致)
  本节点 只做 LiDAR 射线追踪 (不做物理)

工作流 (事件驱动, 无定时器轮询):
  1. 订阅 /mujoco/base_pose (1000Hz, sim_module 发布的 ground truth 位姿)
  2. 每次 base_pose 到达:
     a. 立即发布 /clock (1000Hz, 供所有 use_sim_time 节点)
     b. 时间节流检查: 距上次 LiDAR 帧 ≥ lidar_period (100ms@10Hz) 则立即射线追踪
     c. 设置 free joint qpos → mj_forward() → 射线追踪 → 发布 /livox/lidar

运行:
  ros2 run humanoid_sim mujoco_lidar_bridge.py
  或:  python3 mujoco_lidar_bridge.py --model <path_to_xyber_x1_nav.xml>

依赖:
  pip install -e navigation/MuJoCo-LiDAR
  colcon build livox_ros_driver2
"""

import os
import sys
import numpy as np

import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField

# --- MuJoCo-LiDAR ---
_MUJOCO_LIDAR_SRC = os.environ.get(
    "MUJOCO_LIDAR_SRC",
    os.path.expanduser("~/code/F1/navigation/MuJoCo-LiDAR/src"),
)
if _MUJOCO_LIDAR_SRC not in sys.path:
    sys.path.insert(0, _MUJOCO_LIDAR_SRC)
from mujoco_lidar import MjLidarWrapper, scan_gen

# --- Livox CustomMsg ---
try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    _HAS_CUSTOM_MSG = True
except ImportError:
    _HAS_CUSTOM_MSG = False
    print("[WARN] livox_ros_driver2 不可用，回退 PointCloud2")

# --- 默认参数 ---
_LIDAR_HZ = 10
_LIDAR_DOWNSAMPLE = 10
_LIDAR_FRAME_NS = 1_000_000  # 1ms


class LidarBridgeNode(Node):
    """LiDAR 射线追踪桥接节点 (事件驱动)"""

    def __init__(self):
        super().__init__("mujoco_lidar_bridge")

        # --- 参数 ---
        self.declare_parameter("model_path", "")
        self.declare_parameter("lidar_hz", _LIDAR_HZ)
        self.declare_parameter("downsample", _LIDAR_DOWNSAMPLE)
        self.declare_parameter("output_type", "pointcloud2")

        model_path = self.get_parameter("model_path").value
        if not model_path:
            self.get_logger().error("model_path 参数未设置!")
            raise RuntimeError("model_path required")

        self._lidar_hz = self.get_parameter("lidar_hz").value
        self._downsample = self.get_parameter("downsample").value
        output_type = str(self.get_parameter("output_type").value).lower()
        if output_type not in ("pointcloud2", "custom", "auto"):
            raise RuntimeError("output_type must be one of: pointcloud2, custom, auto")
        self._use_custom_msg = (output_type == "custom") or (
            output_type == "auto" and _HAS_CUSTOM_MSG
        )
        if self._use_custom_msg and not _HAS_CUSTOM_MSG:
            raise RuntimeError("output_type=custom requires livox_ros_driver2")
        self._lidar_period_sec = 1.0 / self._lidar_hz

        # --- 加载 MuJoCo 场景 (仅用于几何体 + 射线追踪, 不做物理) ---
        self.get_logger().info(f"Loading MuJoCo scene: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # The lidar bridge owns a separate MuJoCo model used only for ray casts.
        # Move every robot geom into group 3 so geomgroup[3]=0 removes self hits.
        robot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "x1-body")
        excluded_geom_count = 0
        if robot_body_id >= 0:
            robot_bodies = {robot_body_id}
            changed = True
            while changed:
                changed = False
                for body_id in range(self.model.nbody):
                    if body_id not in robot_bodies and self.model.body_parentid[body_id] in robot_bodies:
                        robot_bodies.add(body_id)
                        changed = True
            for geom_id in range(self.model.ngeom):
                if self.model.geom_bodyid[geom_id] in robot_bodies:
                    self.model.geom_group[geom_id] = 3
                    excluded_geom_count += 1

        # 初始化 LiDAR wrapper
        geomgroup = np.ones((mujoco.mjNGROUP,), dtype=np.ubyte)
        geomgroup[3] = 0  # 排除机器人自身与碰撞几何组
        self.lidar = MjLidarWrapper(
            self.model,
            site_name="lidar_site",
            backend="cpu",
            cutoff_dist=30.0,
            args={"geomgroup": geomgroup},
        )
        self.get_logger().info(
            f"LiDAR raycast excludes {excluded_geom_count} robot geoms via group 3"
        )
        self.livox_gen = scan_gen.LivoxGenerator("mid360")

        # 查找 free joint 的 qpos 地址
        free_joint_id = -1
        for i in range(self.model.njnt):
            if self.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                free_joint_id = i
                break
        if free_joint_id < 0:
            self.get_logger().error("模型中没有 free joint!")
            raise RuntimeError("No free joint found")
        self.qpos_adr = self.model.jnt_qposadr[free_joint_id]
        self.get_logger().info(f"Free joint qpos_adr={self.qpos_adr}")

        # --- 事件驱动状态 ---
        self._last_lidar_sim_time = -1.0  # 上次 LiDAR 帧的仿真时间
        self._frame_count = 0

        # --- ROS2 接口 ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(PoseStamped, "/mujoco/base_pose", self._pose_cb, qos)

        if self._use_custom_msg:
            self.lidar_pub = self.create_publisher(CustomMsg, "/livox/lidar", 10)
        else:
            self.lidar_pub = self.create_publisher(PointCloud2, "/livox/lidar", 10)

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)

        self.get_logger().info(
            f"LidarBridge 启动 (事件驱动): model={model_path}\n"
            f"  lidar_hz={self._lidar_hz}, downsample={self._downsample}\n"
            f"  output_type={'custom' if self._use_custom_msg else 'pointcloud2'}\n"
            f"  base_pose 驱动: /clock (1000Hz) + /livox/lidar ({self._lidar_hz}Hz)\n"
        )

    def _pose_cb(self, msg: PoseStamped):
        """事件驱动回调: 发布 /clock (每次) + 射线追踪 (节流到 lidar_hz)"""
        # 1. 提取仿真时间 + 发布 /clock (每次都发, 供 use_sim_time)
        sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        clock_msg = Clock()
        clock_msg.clock = msg.header.stamp
        self.clock_pub.publish(clock_msg)

        # 2. 节流检查: 距上次 LiDAR 帧的时间差
        if self._last_lidar_sim_time >= 0:
            elapsed = sim_time - self._last_lidar_sim_time
            if elapsed < self._lidar_period_sec:
                return  # 未到下一帧周期, 跳过

        self._last_lidar_sim_time = sim_time

        # 3. 设置 free joint 位姿 → mj_forward 更新几何体位置
        self.data.qpos[self.qpos_adr + 0] = msg.pose.position.x
        self.data.qpos[self.qpos_adr + 1] = msg.pose.position.y
        self.data.qpos[self.qpos_adr + 2] = msg.pose.position.z
        self.data.qpos[self.qpos_adr + 3] = msg.pose.orientation.w
        self.data.qpos[self.qpos_adr + 4] = msg.pose.orientation.x
        self.data.qpos[self.qpos_adr + 5] = msg.pose.orientation.y
        self.data.qpos[self.qpos_adr + 6] = msg.pose.orientation.z
        mujoco.mj_forward(self.model, self.data)

        # 4. 射线追踪
        rays_theta, rays_phi = self.livox_gen.sample_ray_angles(
            downsample=self._downsample
        )
        self.lidar.trace_rays(self.data, rays_theta, rays_phi)
        pts = self.lidar.get_hit_points()

        if len(pts) == 0:
            return

        # 5. 过滤近场噪点
        valid_mask = np.linalg.norm(pts, axis=1) > 0.01
        pts = pts[valid_mask]
        if len(pts) < 10:
            self.get_logger().warn(
                f"valid hits={len(pts)}", throttle_duration_sec=5.0
            )
            return
        if self._frame_count % 20 == 0:
            self.get_logger().info(
                f"MID360 hits={len(pts)}, downsample={self._downsample}",
                throttle_duration_sec=2.0,
            )

        # 6. 发布点云
        stamp_sec = msg.header.stamp.sec
        stamp_nsec = msg.header.stamp.nanosec

        if self._use_custom_msg:
            self._publish_custom(pts, stamp_sec, stamp_nsec)
        else:
            self._publish_pc2(pts, stamp_sec, stamp_nsec)

        self._frame_count += 1

    def _publish_custom(self, pts: np.ndarray, sec: int, nsec: int):
        msg = CustomMsg()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = "lidar_link"
        msg.timebase = sec * int(1e9) + nsec
        msg.point_num = len(pts)
        msg.lidar_id = 0

        for i in range(len(pts)):
            cp = CustomPoint()
            cp.offset_time = int(i / len(pts) * _LIDAR_FRAME_NS)
            cp.x = float(pts[i, 0])
            cp.y = float(pts[i, 1])
            cp.z = float(pts[i, 2])
            cp.reflectivity = 100
            cp.tag = 0
            cp.line = 0
            msg.points.append(cp)

        self.lidar_pub.publish(msg)

    def _publish_pc2(self, pts: np.ndarray, sec: int, nsec: int):
        dtype = np.dtype([
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("intensity", "<f4"),
            ("tag", "u1"),
            ("line", "u1"),
        ])
        fields = [
            PointField(name="x", offset=dtype.fields["x"][1], datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=dtype.fields["y"][1], datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=dtype.fields["z"][1], datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=dtype.fields["intensity"][1], datatype=PointField.FLOAT32, count=1),
            PointField(name="tag", offset=dtype.fields["tag"][1], datatype=PointField.UINT8, count=1),
            PointField(name="line", offset=dtype.fields["line"][1], datatype=PointField.UINT8, count=1),
        ]
        pts_i = np.empty(len(pts), dtype=dtype)
        pts_i["x"] = pts[:, 0].astype(np.float32, copy=False)
        pts_i["y"] = pts[:, 1].astype(np.float32, copy=False)
        pts_i["z"] = pts[:, 2].astype(np.float32, copy=False)
        pts_i["intensity"] = 100.0
        pts_i["tag"] = 0
        pts_i["line"] = (np.arange(len(pts), dtype=np.uint32) % 4).astype(np.uint8)

        msg = PointCloud2()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = "lidar_link"
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = dtype.itemsize
        msg.height = 1
        msg.width = len(pts)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = pts_i.tobytes()
        self.lidar_pub.publish(msg)


def main(args=None):
    import argparse

    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="MuJoCo LiDAR Bridge")
    parser.add_argument("--model", type=str, default="",
                        help="Path to MuJoCo scene XML (xyber_x1_nav.xml)")
    cli_args, _ = parser.parse_known_args()

    node = LidarBridgeNode()

    if cli_args.model:
        node.get_logger().warn("--model 应通过 ROS2 参数 model_path 传入")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
