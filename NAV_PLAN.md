# X1 人形机器人 MuJoCo 导航仿真 — 详细实施计划

> **目标**：在 MuJoCo 中运行 X1 机器人的 RL 行走策略，仿真 mid360 LiDAR，通过 FAST_LIO2 建图，最终由 Nav2 实现自主点到点导航。
>
> **原则**：不破坏 Gazebo 已有导航框架，逐阶段推进，每阶段有明确验收标准后再进入下一阶段。

---

## 总体架构

```
┌─────────────────────────────────────────────────────────┐
│                    MuJoCo 仿真进程                        │
│  RL 策略(TorchScript) → PD控制 → X1 行走                 │
│  mujoco_lidar → mid360 点云(24000pts/帧, 10Hz)           │
│  MuJoCo sensors → IMU 数据(200Hz)                        │
└──────────────┬──────────────────────────────────────────┘
               │ ROS2 话题
       ┌───────┴────────┐
       │                │
  /livox/lidar      /imu/data          /tf
  (CustomMsg)       (Imu, 200Hz)   odom→base_link
       │                │
  ┌────▼────────────────▼────┐
  │       FAST_LIO2           │
  │  激光惯性里程计 + 建图      │
  └────────────┬─────────────┘
               │
        /Odometry  /cloud_registered
               │
  ┌────────────▼─────────────┐
  │          Nav2             │
  │  全局规划 + MPPI 局部控制  │
  └────────────┬─────────────┘
               │
           /cmd_vel
               │
  ┌────────────▼─────────────┐
  │   sim2sim_nav.py 订阅     │
  │   更新 x_vel / yaw_vel    │
  └──────────────────────────┘
```

---

## 已完成工作（代码已写入）

| 文件 | 内容 |
|------|------|
| `agibot_x1_train/resources/robots/x1/mjcf/robot/xyber_x1/xyber_x1_serial.xml` | 新增 `lidar_site`（pelvis 上 0.35m、前 0.05m） |
| `agibot_x1_train/resources/robots/x1/mjcf/environment/nav_room.xml` | 8×8m 导航测试房间（墙/走廊/柱子/箱子） |
| `agibot_x1_train/resources/robots/x1/mjcf/xyber_x1_nav.xml` | 导航场景入口（include robot + nav_room） |
| `agibot_x1_train/humanoid/scripts/sim2sim_nav.py` | 核心脚本：RL + mid360 + IMU + TF + /cmd_vel |
| `fast_lio2/config/mujoco_x1_mid360.yaml` | FAST_LIO2 专用配置（外参已填，lidar_type:1） |

---

## 阶段 1 — 基础仿真验证

**目标**：确认 MuJoCo 环境、mujoco_lidar 包和 livox_ros_driver2 能正常工作。

### 1.1 安装 mujoco_lidar

```bash
cd ~/humanoid_ws/src/MuJoCo-LiDAR
pip install -e ".[cpu]"

# 验证安装
python -c "from mujoco_lidar import MjLidarWrapper, scan_gen; print('OK')"
```

**预期输出**：`OK`，无 ImportError。

---

### 1.2 构建 livox_ros_driver2

```bash
cd ~/humanoid_ws
colcon build --packages-select livox_ros_driver2
source install/setup.bash

# 验证消息类型存在
ros2 interface show livox_ros_driver2/msg/CustomMsg
```

**预期输出**：显示 CustomMsg 字段定义（header, timebase, point_num, points...）。

---

### 1.3 验证 X1 在 nav_room 中行走

用原始 `sim2sim.py` 先验证场景文件本身无问题（不涉及 ROS2）。
临时方式：直接修改 `Sim2simCfg.sim_config.mujoco_model_path` 指向 `xyber_x1_nav.xml`，或：

```bash
cd ~/humanoid_ws/src/agibot_x1_train
python humanoid/scripts/sim2sim.py --task x1_dh_stand
# 确认 MuJoCo viewer 弹出，X1 在有墙的房间中站立行走
```

> ⚠️ `sim2sim.py` 默认用 `xyber_x1_flat.xml`。若想验证 nav_room，
> 可临时在命令行中加参数，或直接跳到 1.4。

---

### 1.4 启动 sim2sim_nav.py（纯仿真，不看 ROS2）

```bash
cd ~/humanoid_ws/src/agibot_x1_train
source ~/humanoid_ws/install/setup.bash
python humanoid/scripts/sim2sim_nav.py --task x1_dh_stand
```

**✅ 阶段1 验收标准**
- [ ] MuJoCo viewer 正常弹出
- [ ] X1 在 nav_room 中站立/行走，不穿墙、不倒地
- [ ] 终端无 ImportError / MJCF 解析报错

---

## 阶段 2 — ROS2 传感器数据验证

**目标**：确认 `/livox/lidar`、`/imu/data`、`/tf` 以正确频率和格式发布。

### 2.1 检查话题

```bash
# 终端A：运行仿真（保持运行）
python humanoid/scripts/sim2sim_nav.py --task x1_dh_stand

# 终端B：检查话题列表和频率
ros2 topic list
ros2 topic hz /livox/lidar     # 期望 ~10 Hz
ros2 topic hz /imu/data        # 期望 ~200 Hz
ros2 topic echo /tf --once     # 期望看到 odom→base_link

# 检查点云消息内容
ros2 topic echo /livox/lidar --once
```

**期望**：
- `/livox/lidar`：类型 `livox_ros_driver2/msg/CustomMsg`，`point_num` ≈ 24000
- `/imu/data`：类型 `sensor_msgs/msg/Imu`，有合理的加速度/角速度值
- `/tf`：`odom → base_link` 动态变换 + `base_link → lidar_link` 静态变换

---

### 2.2 RViz2 可视化

```bash
rviz2
```

RViz2 配置：
- Fixed Frame：`odom`
- 添加 `PointCloud2` → Topic：`/livox/lidar`（注：CustomMsg 需要 FAST_LIO2 转发后才能在 RViz2 看到标准点云）
- 添加 `TF` 显示，查看 `lidar_link` 是否在机器人胸部位置

> 💡 若想在 RViz2 直接看到原始点云，可临时把 `_USE_CUSTOM_MSG` 强制改为 `False`，
> 使用 PointCloud2 格式，验证完再改回来。

**✅ 阶段2 验收标准**
- [ ] 三个话题以正确频率发布
- [ ] `lidar_link` TF 位于机器人胸部区域（约 1m 高度）
- [ ] 点云形状符合房间几何（可见墙壁、柱子轮廓）

---

## 阶段 3 — FAST_LIO2 建图验证

**目标**：FAST_LIO2 成功订阅仿真数据，实时构建点云地图，机器人行走时地图无明显漂移。

### 3.1 启动 FAST_LIO2

```bash
# 终端A：仿真（保持）
python humanoid/scripts/sim2sim_nav.py --task x1_dh_stand

# 终端B：FAST_LIO2
cd ~/humanoid_ws
source install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_path:=$(pwd)/src/fast_lio2/config/mujoco_x1_mid360.yaml
```

**观察 FAST_LIO2 输出**：
```
[ INFO] Scan matched ratio: 0.xx    # >0.9 说明匹配良好
[ INFO] Residual: x.xxxe-xx         # 残差小说明收敛
```

---

### 3.2 驱动机器人行走建图

```bash
# 终端C：发速度命令（前进 0.3m/s）
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10

# 让机器人走一圈后停止
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
```

### 3.3 RViz2 观察建图效果

RViz2 添加：
- `/cloud_registered`（白色点云地图）
- `/Odometry`（里程计轨迹）
- `/path`（历史路径）

**常见问题处理**：

| 症状 | 原因 | 解决 |
|------|------|------|
| FAST_LIO2 立即崩溃 | lidar_type 不匹配 | 检查是否用 CustomMsg，对应改 `lidar_type: 1` |
| 地图严重漂移 | 外参不准 | 调整 `mujoco_x1_mid360.yaml` 中 `extrinsic_T` |
| 点云全为空 | 射线全未命中 | 检查 `lidar_site` 位置是否在房间内部 |
| IMU 数据异常 | 坐标系不对 | 检查 `body-orientation` sensor 的 frame |

**✅ 阶段3 验收标准**
- [ ] FAST_LIO2 启动无崩溃
- [ ] 机器人静止时地图稳定（里程计漂移 < 2cm/10s）
- [ ] 机器人行走一圈后房间轮廓清晰可见
- [ ] `/Odometry` 轨迹与实际行走路径基本一致

---

## 阶段 4 — Nav2 接入

**目标**：Nav2 接收 FAST_LIO2 的里程计和地图，输出 `/cmd_vel` 驱动 X1 自主导航。

> 📝 **此阶段代码由 Cascade 协助生成**，包括：
> - Nav2 的 `params.yaml`（适配 X1 足迹尺寸）
> - FAST_LIO2 → Nav2 适配层（odom 坐标系对齐）
> - 导航 launch 文件

### 4.1 需要配置的关键参数

**X1 机器人足迹（用于 costmap）**：
- 半径：约 0.25m（保守值，X1 肩宽约 0.5m）
- 高度：约 1.65m

**FAST_LIO2 输出对接 Nav2**：
- FAST_LIO2 发布 `/Odometry`（frame: `camera_init` → `body`）
- Nav2 需要 `odom` → `base_link`
- 需要确认 frame_id 是否一致，必要时加 frame 重映射

### 4.2 预计生成的文件

```
humanoid_ws/src/
├── x1_nav2/                       # 新建 Nav2 配置包（不影响现有代码）
│   ├── config/
│   │   ├── nav2_params.yaml       # Nav2 参数（costmap/planner/controller）
│   │   └── rviz_nav.rviz          # 导航可视化配置
│   ├── launch/
│   │   └── x1_nav2.launch.py      # 一键启动 Nav2
│   └── package.xml
```

**✅ 阶段4 验收标准**
- [ ] Nav2 所有节点正常启动（无 `Waiting for...` 超时）
- [ ] RViz2 中 costmap 正确显示障碍物
- [ ] 设置 2D Goal Pose 后，机器人开始移动
- [ ] X1 能绕过柱子/箱子到达目标点

---

## 阶段 5 — 闭环评估

**目标**：长时间连续导航测试，量化评估 RL 步态对建图精度的影响。

### 5.1 测试项目

| 测试 | 指标 | 对比基准 |
|------|------|---------|
| 静止建图精度 | 里程计漂移(m/min) | Gazebo 圆柱体 |
| 行走建图精度 | 回环误差(m) | Gazebo 圆柱体 |
| 步态扰动量化 | IMU 振动功率谱 | 理想刚体运动 |
| 导航成功率 | 到达率/耗时 | Gazebo 圆柱体 |

### 5.2 数据采集

```bash
# 录制 rosbag 用于离线分析
ros2 bag record /livox/lidar /imu/data /Odometry /path /cmd_vel \
  -o ~/humanoid_ws/bags/x1_nav_test_$(date +%Y%m%d_%H%M%S)
```

**✅ 阶段5 验收标准**
- [ ] 5分钟内建图轨迹回环误差 < 0.2m
- [ ] X1 RL 步态引起的 IMU 振动已被 FAST_LIO2 有效补偿
- [ ] 自主导航成功率 > 80%

---

## 依赖版本记录

| 包 | 版本要求 | 备注 |
|----|---------|------|
| mujoco | ≥ 3.0 | 已有 |
| mujoco_lidar | latest | CPU 后端 |
| ROS2 | Humble | 已有 |
| FAST_LIO2 | 已有 | 用新配置文件 |
| Nav2 | Humble | 已有（Gazebo 环境中） |
| livox_ros_driver2 | 已有源码 | 需 colcon build |
| PyTorch | ≥ 1.10 | 已有（RL 策略） |

---

## 快速参考：完整启动命令序列

```bash
# === 终端1：MuJoCo + RL + LiDAR ===
cd ~/humanoid_ws/src/agibot_x1_train
source ~/humanoid_ws/install/setup.bash
python humanoid/scripts/sim2sim_nav.py --task x1_dh_stand

# === 终端2：FAST_LIO2 建图 ===
cd ~/humanoid_ws && source install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_path:=$(pwd)/src/fast_lio2/config/mujoco_x1_mid360.yaml

# === 终端3：Nav2（阶段4后使用）===
ros2 launch x1_nav2 x1_nav2.launch.py

# === 终端4：RViz2 可视化 ===
rviz2 -d ~/humanoid_ws/src/x1_nav2/config/rviz_nav.rviz
```

---

## 当前状态

- **阶段1** 🔄 进行中 — 等待安装 mujoco_lidar 并验证仿真启动
- **阶段2-5** ⏳ 待执行

> 每完成一个阶段，在对应的 `[ ]` 中填 `[x]` 并告知 Cascade，继续推进下一阶段。
