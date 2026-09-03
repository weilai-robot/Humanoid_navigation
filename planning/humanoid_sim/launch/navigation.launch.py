import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_humanoid = get_package_share_directory('humanoid_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # 指向刚才保存的地图和新建的参数文件
    # lab_env_map: main 自带 generate_map_from_xml.py 生成的完整世界图 (13/13 关键点位验证)
    # mujoco_lab.pgm 为 FastLIO 单次实建图: 缺失 par_s1 墙段 (通道A被静态图视为 2.4m 宽)
    # + 北墙大面积缺失 + 噪点岛屿 — 与仿真世界不一致, 不可用于严格 CI 判定
    map_file = os.path.join(pkg_humanoid, 'maps', 'lab_env_map.yaml')
    params_file = os.path.join(pkg_humanoid, 'config', 'nav2_mujoco.yaml')
    # 保守行为树: Recovery 仅 [清代价地图+等待], 无 Spin/BackUp
    # (默认 BT 的 spin 0.798 rad/s 曾致人形摔倒 → FastLIO 发散, 2026-07-14 CI 事故)
    bt_xml_file = os.path.join(pkg_humanoid, 'behavior_trees', 'humanoid_navigate_w_replanning.xml')
    if not os.path.isfile(bt_xml_file):
        # 增量构建可能未安装 behavior_trees 目录 — 回退默认 BT 而非让
        # bt_navigator 因文件缺失崩溃 (默认 BT 的 spin 已被 behavior_server
        # 限速 0.3 rad/s + velocity_smoother deadband=0 兜底)
        print(f'[navigation.launch] WARNING: BT 文件缺失: {bt_xml_file}, '
              f'回退 Nav2 默认 BT (须重建 humanoid_sim 安装 behavior_trees)')
        bt_xml_file = 'default'

    # 用 RewrittenYaml 注入运行期参数 (yaml 不支持包路径替换, 须在此改写)。
    # 注意: 必须把 RewrittenYaml 对象【直接】作为 launch argument 传给
    # bringup_launch.py (nav2 官方模式; 它是 Substitution, perform 时返回
    # 改写后的临时 yaml 路径)。不能包 ParameterFile — 那是给
    # Node(parameters=[...]) 用的, 作为 launch argument 传递会以
    # "'ParameterFile' object is not iterable" 崩溃 (CI run 33030080681)。
    #   - default_nav_to_pose_bt_xml: 自定义保守 BT 的绝对路径
    #     (RewrittenYaml 按 key 改写 nav2_mujoco.yaml 中的占位值 "default")
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml_file,
        },
        convert_types=True)

    # ====== 1. 包含 tf_bridge (发布 odom 和 map->camera_init 静态 TF) ======
    tf_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_humanoid, 'launch', 'tf_bridge.launch.py')
        )
    )

    # ====== 2. 包含 pc2scan (将 3D 点云转换为 2D 激光给 AMCL) ======
    # pc2scan_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_humanoid, 'launch', 'pc2scan.launch.py')
    #     )
    # )

    # ====== 3. Nav2 核心: 纯导航栈 + 显式 map_server (不用 bringup/AMCL) ======
    # 关键: bringup_launch.py 会启动 AMCL, 其发布动态 map→odom 与 open3d_loc
    # 的静态 camera_init→odom 形成 odom 双父 → TF 树间歇撕裂
    # ("two or more unconnected trees", CI run 33042878544: A-D 场景机器人
    # 静止, nav2 284 次 TF 超时; E/F 才恢复行走)。
    # 仿真中地图与世界轴对齐, map→camera_init→odom 静态链即正确定位,
    # AMCL 完全多余。navigation_launch.py 只含 controller/planner/behavior/
    # bt_navigator/velocity_smoother 等, 不含 AMCL; map_server 单独启动。
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'yaml_filename': map_file}],
    )
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'autostart': True,
                     'node_names': ['map_server']}],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': configured_params,
            'use_sim_time': 'True',  # true
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        tf_bridge_launch,
        map_server_node,
        lifecycle_manager_map,
        nav2_launch
    ])
