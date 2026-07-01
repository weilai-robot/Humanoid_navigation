import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'humanoid_sim'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. 处理 URDF 文件
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_xml = robot_description_config.toxml()

    #传入360雷达csv
    livox_simulation_path = get_package_share_directory('ros2_livox_simulation')
    csv_file_path = os.path.join(livox_simulation_path, 'scan_mode', 'mid360.csv')

    # 处理 xacro 时传入参数
    robot_description_config = xacro.process_file(
        xacro_file, 
        mappings={'csv_path': csv_file_path}
    )

    # --- 定义 World 文件路径 ---
    # test_room 最开始的简单环境
    # test 加了一些高墙
    # school_room 套在一个屋子里面
    world_path = os.path.join(pkg_share, 'worlds', 'school_room.world')    

    # 2. 启动 Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_xml, 'use_sim_time': True}]
    )

    # 3. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        # 添加下面这一行，告诉 Gazebo 不要加载空地图，而是加载我们的文件
        # launch_arguments={'world': world_path}.items() # <--- 3. 传入 world 参数

        # 无头模式：关闭 Gazebo GUI 渲染，只运行物理引擎，提升 RTF
        launch_arguments={
            'world': world_path,
            # 'gui': 'false',       # 关闭 gzclient（3D渲染窗口），gzserver 正常运行
            # 'verbose': 'true',    # 输出详细日志，方便调试
        }.items()
    )

    # 4. 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'f1_humanoid',
                   '-z', '0.05','-timeout', '600'],
        output='screen'
    )

    return LaunchDescription([
        # 强制设置断网环境变量，防止 Gazebo 启动卡死 (推荐加上)
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),
        # 强制使用 Mesa 软件渲染，确保 gzserver 在虚拟显示下能初始化渲染引擎（雷达传感器需要）
        # SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
