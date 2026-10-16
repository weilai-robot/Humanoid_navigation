cmake_minimum_required(VERSION 3.8)
project(humanoid_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 安装 Python 脚本为可执行节点
# nav_state_manager.py 必须在此列表：nav_state_manager.launch.py 用
# Node(executable='nav_state_manager.py') 找它，Humble 只在
# <prefix>/lib/humanoid_sim/ 下解析可执行文件，缺装=launch 直接
# SubstitutionFailure（P0 审查 2026-10-16 发现的历史缺口）
install(PROGRAMS
  scripts/odom_bridge.py
  scripts/nav_state_manager.py
  scripts/mujoco_lidar_bridge.py
  scripts/livox_imu_throttle.py
  scripts/leg_odom_node.py
  scripts/generate_map_from_xml.py
  scripts/generate_nav_map.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  worlds
  config
  maps
  rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
