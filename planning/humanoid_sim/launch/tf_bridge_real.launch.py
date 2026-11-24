"""
TF 桥接 Launch 文件 — 实机版 (X1 + Mid360 倒装)
启动内容：
  1. odom_bridge — FastLIO2 /Odometry → odom→base_footprint
                 + /cmd_vel → /cmd_vel_limiter（AimRT 只订 limiter）
  2. 静态 TF（非精确标定，仅接通 TF 树）:
       base_footprint → base_link (z=-0.65)
       base_link → lidar_link (z=-0.637, Rx≈180° 倒装)
  注意：
  - map→odom / map→camera_init 由 open3d_loc 动态发布，勿再发静态
  - use_sim_time: False

z-down 口径（2026-11-24 离线 bag A/B 定值，真机三臂 /tmp/realbag_z_ab.py）：
  native 倒装图 robot_lab_20261030 下 body z 轴物理朝下（bag zcomp≈+0.97~1.0 实证），
  地面锚 map z=+1.287。btfz=+1.287 ⇒ odom→base_footprint z≈+1.29±0.03 落地锚成立
  （旧值 −1.25 在新代码下 = footprint 悬在传感器上方 1.25m，z 为负）。
  静态枝取负 = base_link 在 z-down footprint 系中比地面高 0.65（z 反向即上）；
  lidar_link z=-0.637=1.287-0.65 由锚链推得（取代旧估计 +0.66），Rx180 保留
  （S0 正值臂对账 map→base_link +1.94 / map→lidar +2.60，判死；S1 负值臂
  bag 20261119 map→base_link +0.645 / map→lidar +0.011 ≈ 传感器本体高度，胜）。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---- odom_bridge: Odometry → odom→base_footprint + cmd_vel 限幅中继 ----
        Node(
            package='humanoid_sim',
            executable='odom_bridge.py',
            name='odom_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                # z-down 地面锚：body z 轴物理朝下（倒装），地图地面 z=+1.287。
                # 旧值 -1.25 是 z-up 时代口径；新代码（cancel-hack 已删）下必须为正，
                # 否则 footprint 落在传感器上方（bag A/B: deployed odom→fp z≈-1.2）
                'body_to_footprint_z': 1.287,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'input_topic': '/Odometry',
                'output_topic': '/odom',
                'mount_rpy': [0.0, 0.0, -90.0],  # 2026-11-03 终审：Fixed Frame=base_footprint 点物理前方物体 x<0 ⇒ 朝向相对 10-14 有未记录变动（疑 10-30 吊架期头部转回 180°），[0,0,90]→[0,0,-90]；验法=base_footprint 系点击 ±x（屏幕箭头判据镜像不可辨，作废）；详见 docs/sim2real/真机倒装建图工作记录-robot_lab_20261030-2026-11-02.md
                # 显式写死：AimRT x1_cfg 只订 /cmd_vel_limiter，不可靠默认值
                'enable_cmd_vel_relay': True,
                'cmd_vel_input_topic': '/cmd_vel',
                'cmd_vel_output_topic': '/cmd_vel_limiter',
                # 本限幅器是链路唯一加速度权威（nav2 侧加速度键已删）
                'max_ax': 1.0,
                'max_ay': 0.5,
                'max_az': 1.0,
                # P6a 非对称减速限幅（急停温柔化）。real 无含 /cmd_vel 的 bag，刹车需求
                # 未验证 → 取比 sim(0.6) 更保守的 0.5 明示初值；用首个 real bag 复核（T1 挂账）
                'max_decel': 0.5,
                # wz 衰减夹板：0 = 沿用 max_az（行为不变）；收紧方向待证，勿擅改
                'max_decel_wz': 0.0,
                # P6b 死人开关：/cmd_vel 沉默期非零末目标强制归零超时（s）；两侧同值
                # （nav2 20Hz 两侧同速率）
                'cmd_silence_timeout_s': 0.5,
                # 2026-11-03 G2 兑现（11-02 起预注册）：native z-down 图 base_footprint 物理朝下，
                # nav2 +angular.z=绕下向 z 轴 CCW=物理右转，底盘标准 +wz=左转 ⇒ 反号；不取负则
                # yaw 闭环负增益 ⇒ 原地转圈（11-03 真机实测 ±90 mount 同样转圈，与「mount 只改
                # 偏航不改 z 手性」的预判一致）。linear.y 同理取负，linear.x 不动（mount −90 后
                # base x̂=机头）；仅命令路取负，/odom twist 反馈不取负（与 TF 同口径）。
                # 旧图时代链内有 ICP 种子 roll=180 隐性翻手性，relay 直通即自洽——此参数为
                # z-down 图专属，仿真/旧正装图链须保持 false。
                'invert_cmd_vel_yz': True,
            }]
        ),

        # ---- 静态 TF: base_footprint → base_link ----
        # 非精确标定，仅接通 TF 树（避免 base_link 孤儿）
        # z=-0.65: z-down footprint 系中 base_link 在地面（z=0）之上 0.65
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_footprint_to_base_link',
            parameters=[{'use_sim_time': False}],
            arguments=['0', '0', '-0.65', '0', '0', '0', 'base_footprint', 'base_link']
        ),

        # ---- 静态 TF: base_link → lidar_link ----
        # 非精确标定；qx=1 近似倒装 Mid360（与仿真 tf_bridge 一致）
        # z=-0.637 = 地面锚 1.287 − 0.65（锚链推得，非实测标定；bag A/B S1 臂对账
        # map→lidar z≈+0.01≈传感器本体高度成立）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_link_to_lidar_link',
            parameters=[{'use_sim_time': False}],
            arguments=['0', '0', '-0.637', '1', '0', '0', '0', 'base_link', 'lidar_link']
        ),
    ])
