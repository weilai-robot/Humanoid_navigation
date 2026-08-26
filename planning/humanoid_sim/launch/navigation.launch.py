import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_humanoid = get_package_share_directory('humanoid_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # 指向刚才保存的地图和新建的参数文件
    map_file = os.path.join(pkg_humanoid, 'maps', 'mujoco_lab.yaml')  # f1_test1 school_room school_room2
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

    # 用 RewrittenYaml 注入运行期参数 (yaml 不支持包路径替换, 须在此改写):
    #   - default_nav_to_pose_bt_xml: 自定义保守 BT 的绝对路径
    #     (RewrittenYaml 按 key 改写 nav2_mujoco.yaml 中的占位值 "default")
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'default_nav_to_pose_bt_xml': bt_xml_file,
            },
            convert_types=True),
        allow_substs=True)

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

    # ====== 3. 包含 Nav2 核心 (带 AMCL 等) ======
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': configured_params,
            'use_sim_time': 'True',  # true
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        tf_bridge_launch,
        # pc2scan_launch,
        nav2_launch
    ])
