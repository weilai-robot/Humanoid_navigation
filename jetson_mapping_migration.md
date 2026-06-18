# Jetson Orin Nano 建图迁移配置指南

> **平台**: Jetson Orin Nano + Ubuntu 22.04 + ROS2 Humble  
> **传感器**: Livox Mid360 雷达  
> **算法**: Fast-LIO2 (3D SLAM) + OctoMap (3D/2D 地图)

---

## 关键差异：仿真 vs 实机

| 参数 | 仿真值 | 实机值 | 位置 |
|------|--------|--------|------|
| `use_sim_time` | `true` | `false` | launch 文件 + yaml |
| `timestamp_unit` | `2` (ms/ROS时间) | `0` (us/硬件时间) | `car_30_mid360_real.yaml` |
| `imu_topic` | `/imu/data` | `/livox/imu` | `car_30_mid360_real.yaml` |
| `rviz` | `true` | `false` | `mapping_real.launch.py` |
| `config_file` | `simulation_mid360.yaml` | `car_30_mid360_real.yaml` | launch 文件 |

---

## 第一步：安装系统依赖

`apt install` 是幂等的，已安装的包会自动跳过，直接执行即可。

```bash
sudo apt update && sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-octomap \
  ros-humble-octomap-ros \
  ros-humble-octomap-server \
  libpcl-dev \
  libeigen3-dev \
  libgflags-dev \
  python3-colcon-common-extensions
```

如需检查哪些已安装：

```bash
dpkg -l ros-humble-pcl-ros ros-humble-octomap ros-humble-octomap-ros \
         ros-humble-octomap-server libpcl-dev libeigen3-dev \
         libgflags-dev python3-colcon-common-extensions 2>/dev/null | \
  awk '/^ii/{print $2, "已安装"} /^un/{print $2, "未安装"}'
```

> **注意**：`ros-humble-octomap-server` 在 ROS2 Humble 中不一定默认安装，是最关键的依赖，务必确认。

---

## 第二步：安装 Livox-SDK2

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4          # Jetson Orin Nano 建议 j4，避免OOM
sudo make install
```

---

## 第三步：安装 livox_ros_driver2（ROS2 版）

```bash
cd ~/humanoid_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

编辑 Mid360 驱动配置文件：

```bash
nano ~/humanoid_ws/src/livox_ros_driver2/config/MID360_config.json
```

修改以下字段（其余保持默认）：

```json
{
  "MID360": {
    "host_net_info": {
      "cmd_data_ip":   "192.168.1.50",
      "push_msg_ip":   "192.168.1.50",
      "point_data_ip": "192.168.1.50",
      "imu_data_ip":   "192.168.1.50",
      "log_data_ip":   "192.168.1.50"
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.1xx"
    }
  ]
}
```

> - `192.168.1.50` 改为 Jetson 实际 IP  
> - `192.168.1.1xx` 改为 Mid360 背面标签上的实际 IP

---

## 第四步：配置 Jetson 网络（连接 Mid360）

Mid360 通过有线以太网连接，固定 IP 网段 `192.168.1.x`。

**临时配置（重启后失效）：**

```bash
# 查找网卡名
ip link show

# 替换 eth0 为实际网卡名
sudo ip addr flush dev eth0
sudo ip addr add 192.168.1.50/24 dev eth0
sudo ip link set eth0 up
```

**永久配置（写入 netplan）：**

```bash
sudo nano /etc/netplan/01-network-manager-all.yaml
```

```yaml
network:
  version: 2
  ethernets:
    eth0:                        # 替换为实际网卡名
      dhcp4: false
      addresses: [192.168.1.50/24]
```

```bash
sudo netplan apply

# 验证连通性
ping 192.168.1.1xx              # Mid360 的 IP
```

---

## 第五步：编译工作空间

```bash
cd ~/humanoid_ws

# 1. 编译 Livox 驱动
colcon build --packages-select livox_ros_driver2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2. 编译 Fast-LIO2
colcon build --packages-select fast_lio \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. 编译 humanoid_sim（OctoMap launch）
colcon build --packages-select humanoid_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

---

## 第六步：实机运行

分别打开三个终端：

**终端 1** — 启动 Mid360 驱动：

```bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**终端 2** — 启动 Fast-LIO2：

```bash
source ~/humanoid_ws/install/setup.bash
ros2 launch fast_lio mapping_real.launch.py
```

**终端 3** — 启动 OctoMap：

```bash
source ~/humanoid_ws/install/setup.bash
ros2 launch humanoid_sim octomap_real.launch.py
```

```
sudo nmcli connection add type ethernet ifname eth1 con-name "LiDAR" ipv4.method manual ipv4.addresses 192.168.1.50/24
sudo nmcli connection up "LiDAR"
colcon build --symlink-install --packages-select fast_lio humanoid_sim
colcon build --symlink-install --packages-select humanoid_sim
colcon build --packages-select open3d_loc --symlink-install
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCOMPUTE_PLATFORM=ARM

# 建议使用的终极编译指令
colcon build --symlink-install --packages-select fast_lio open3d_loc humanoid_sim --cmake-args -DCMAKE_BUILD_TYPE=Release


```

---

## 第七步：验证话题

```bash
# 确认 Mid360 话题正常发布
ros2 topic hz /livox/lidar              # 期望: ~10 Hz
ros2 topic hz /livox/imu               # 期望: ~200 Hz

# 确认 Fast-LIO2 输出
ros2 topic hz /cloud_registered_body   # OctoMap 订阅的话题
ros2 topic echo /Odometry --once       # 位姿输出

# 确认 OctoMap 输出
ros2 topic hz /octomap_full            # 3D 地图
ros2 topic hz /projected_map           # 2D 投影地图
```

---

## Jetson Orin Nano 性能优化

```bash
# 开启最大性能模式
sudo nvpmodel -m 0
sudo jetson_clocks
```

`car_30_mid360_real.yaml` 中可按需调整：

| 参数 | 默认值 | 省算力值 | 说明 |
|------|--------|----------|------|
| `point_filter_num` | `3` | `4` | 减少处理点数 |
| `filter_size_surf` | `0.5` | `0.6` | 降低精度换帧率 |
| `frame_skip` (OctoMap) | `1` | `2`~`3` | 跳帧处理 |

> **建议**：在 Jetson 本机**不启动 RViz**，通过 `ros2 bag record` 录包后在 PC 端可视化。

---

## timestamp_unit 说明

> 若 Fast-LIO2 报时间戳异常，尝试按以下顺序排查：
> 
> | 值 | 含义 |
> |---|---|
> | `0` | 微秒 (us) |
> | `1` | 10微秒 |
> | `2` | 毫秒 (ms) / 仿真用 |
> | `3` | 秒 (s) |
> 
> 官方 `mid360.yaml` 默认用 `3`，实机建议先用 `3`，报错再改为 `0`。

---

## 新增文件索引

| 文件 | 说明 |
|------|------|
| `src/fast_lio2/config/car_30_mid360_real.yaml` | 实机 Fast-LIO2 参数配置 |
| `src/fast_lio2/launch/mapping_real.launch.py` | 实机 Fast-LIO2 启动文件 |
| `src/humanoid_sim/launch/octomap_real.launch.py` | 实机 OctoMap 启动文件 |
