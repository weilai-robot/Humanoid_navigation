# tools/ — 导航通用工具（栈的客户端与离线分析）

## 归属判据（与 F1 集成仓 scripts/ 的分工）

- **本目录**：对 nav2 栈的客户端/诊断/离线分析——不组装、不驱动整套栈（现场用还是离线用均可）。
- **F1/scripts/**：组装编排——launch/tmux/session/env、跨子仓接线、写死 F1 运行拓扑假设的（run_*/record_*/archive_*/nav_test_runner/r2s_chain）。

## 现役

| 工具 | 用途 | 用法 |
|---|---|---|
| drift_check.py | 发 NavigateToPose 目标并现场监 odom drift（rclpy，需栈活着） | `python3 tools/drift_check.py [X Y] [timeout] [--no-goal]` |
| bag_analyze.py | rosbag2 离线导航指标 + 命令-响应对照（零 ROS 依赖，WSL/81/minipc 均可跑） | `python3 tools/bag_analyze.py BAG [BAG ...] [--goal X Y YAW] [--csv\|--json\|--dump-timeline]` |
| d1_judge.py | 七维度 D1 判定（命令-响应批判别，吃 S1/S2/... 场景采集目录） | `python3 tools/d1_judge.py RUNS... [-o OUT]` |

三机可得：本仓 dual remote（origin github + gitlab），81/minipc `git pull` 即用；真机侧离线分析无需 F1 集成仓同步。

## 遗留（冻结，勿删）

`sim2sim_test.sh` + `nav_test_runner.py`：sim2sim 测试配对单元（yumx 开发），2026-08 起冻结；nav_test_runner **现役版在 F1/scripts/**（尾录5s+锁终帧），本份仅供 sim2sim_test.sh 内部调用。
