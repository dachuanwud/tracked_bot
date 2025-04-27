# tracked_bot
基于ros2的多功能履带车机器人

# 履带式机器人 ROS 2 Gazebo + Nav2 仿真完整路线图（超详细版）

> **目标**：在台式 / 笔记本电脑上，用 *Gazebo Classic* + *ROS 2 Humble* 完成履带车从建模 → 定位 → SLAM/导航 → 避障 → 手动/自动切换的全流程仿真，为真机部署奠基。

## 项目进度跟踪

| 步骤 | 内容 | 状态 | 完成日期 |
|-----|------|------|---------|
| Step 0 | 环境与工具链准备 | ⬜ 未开始 | |
| Step 1 | 机器人描述（URDF/Xacro） | ✅ 已完成 | 2025-04-25 |
| Step 2 | Gazebo模型与驱动 | ⬜ 未开始 | |
| Step 3 | 手柄遥控 | ⬜ 未开始 | |
| Step 4 | 里程计+IMU融合 | ⬜ 未开始 | |
| Step 5 | SLAM与静态定位 | ⬜ 未开始 | |
| Step 6 | Nav2骨架 | ⬜ 未开始 | |
| Step 7 | 局部避障&速度整形 | ⬜ 未开始 | |
| Step 8 | twist_mux仲裁 | ⬜ 未开始 | |
| Step 9 | 真机过渡 | ⬜ 未开始 | |

状态说明：
- ⬜ 未开始
- 🟡 进行中
- ✅ 已完成
- ❌ 遇到问题

---

## Step 0 · 环境与工具链准备 （一次性完成）

| # | 动作 | 参考命令 / 文件 |
|---|------|----------------|
| 0‑1 | 安装 Ubuntu 22.04 Desktop | [下载 ISO](https://releases.ubuntu.com/) |
| 0‑2 | 系统更新 | `sudo apt update && sudo apt upgrade -y` |
| 0‑3 | 安装 ROS 2 Humble 桌面完整版 | ```bash
sudo locale-gen en_US en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install ros-humble-desktop
``` |
| 0‑4 | 配置环境变量 | `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc` |
| 0‑5 | 安装 Gazebo Classic | `sudo apt install gazebo` (ROS Desktop 已含) |
| 0‑6 | 创建工作空间 | ```bash
mkdir -p ~/tracked_bot_ws/src
cd ~/tracked_bot_ws
colcon build
``` |
| 0‑7 | 安装常用依赖 | `sudo apt install ros-humble-{nav2-*,slam-toolbox,robot-localization,teleop-twist-joy,joy,ros2-control,ros2-controllers}` |

---

## Step 1 · 机器人描述（`tracked_bot_description`）

1. **初始化包**  
   ```bash
   cd ~/tracked_bot_ws/src
   ros2 pkg create --build-type ament_cmake tracked_bot_description
   ```

2. **撰写 Xacro**  
   * `urdf/tracked_bot.urdf.xacro`  
   * 关键标签：`<transmission>` + `<gazebo reference="left_wheel_link">`

3. **生成 RViz 配置**  
   ```bash
   rviz2 -d $(ros2 pkg prefix tracked_bot_description)/rviz/tracked_bot.rviz
   ```

4. **Launch 可视化**  
   新建 `launch/view_robot.launch.py`：  
   ```python
   from launch_ros.actions import Node
   # ... load robot_state_publisher & rviz2
   ```

5. **验证**  
   `ros2 launch tracked_bot_description view_robot.launch.py`  
   检查 `/tf_static` 与 `/robot_description`

---

## Step 2 · Gazebo 模型与驱动（`tracked_bot_gazebo`）

### 2‑1 创建包并复制 meshes

```bash
ros2 pkg create --build-type ament_cmake tracked_bot_gazebo
mkdir -p tracked_bot_gazebo/models/tracked_bot/meshes
```

### 2‑2 写 SDF 插件

```xml
<!-- diff_drive 插件 -->
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.45</wheel_separation>
  <wheel_radius>0.09</wheel_radius>
  <command_topic>/cmd_vel</command_topic>
</plugin>
```

### 2‑3 Launch 世界

`launch/empty_world.launch.py`  
```python
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare('gazebo_ros'), '/launch/gzserver.launch.py']),
    launch_arguments={'world': world_file}.items())
```

### 2‑4 测试

```bash
ros2 launch tracked_bot_gazebo empty_world.launch.py
rostopic pub /cmd_vel geometry_msgs/Twist "{linear:{x:0.5},angular:{z:0.0}}"
```

---

## Step 3 · 手柄遥控（`tracked_bot_teleop`）

1. 安装内核驱动 `sudo apt install joystick`  
2. `config/joy_params_h12.yaml`（示例）  
   ```yaml
   axis_linear: 1
   scale_linear: 1.0
   enable_button: 4
   ```
3. `launch/teleop_joy.launch.py` 同时启 `joy_linux` 与 `teleop_twist_joy`.
4. **判据**：摇杆前推 → Gazebo 车辆前进。

---

## Step 4 · 里程计 + IMU 融合（`tracked_bot_localization`）

| 子步骤 | 说明 |
|-------|------|
| 4‑1 | `odom_pub` 插件已在 Step 2 产生 `/wheel_odom` |
| 4‑2 | 创建包 `tracked_bot_localization`，复制 `ekf.yaml` |
| 4‑3 | 关键参数：`world_frame: map`・`odom0: wheel_odom`・`imu0: imu/data` |
| 4‑4 | `launch/ekf.launch.py` 启动 `ekf_node`. |
| 4‑5 | **检查**：`ros2 topic echo /odom | grep -m1 pose`，看连贯性。 |

---

## Step 5 · SLAM 与静态定位

### 5A 在线建图

```bash
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=${pkg_path}/config/mapper_params_online_sync.yaml
```

*保存地图*  
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: 'campus_map'}"
```

### 5B AMCL 定位

1. `map_server` 加载 `campus_map.yaml`  
2. `amcl` 参数：`use_map_topic: true`；`alpha1: 0.2` 等。  
3. 判据：RViz `PoseArray` 收敛于真位姿。

---

## Step 6 · Nav2 骨架

1. 复制官方示例 `tb3_nav_params.yaml`，删掉多余字段。  
2. 设 `planner_server.plugin: nav2_smac_planner/SmacPlannerHybrid`。  
3. `launch/nav2_bringup.launch.py` 包含：`map_server`, `amcl`, `planner_server`, `controller_server`, `bt_navigator`.  
4. **测试**  
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ... 
   ```

---

## Step 7 · 局部避障 & 速度整形

| 动作 | 参数点 |
|------|--------|
| 开启 `inflation_layer` | `inflation_radius: 0.4` |
| 开启 `voxel_layer` | `z_voxels: 10` `z_resolution: 0.2` |
| 调 `DWB` critic | `PathAlign.scale: 32.0` |
| 启 `nav2_velocity_smoother` | `max_velocity: 0.6` `decel_limit: 1.5` |
| 判据 | 撒盒子模型，机器人平滑绕行 |

---

## Step 8 · twist_mux 仲裁

1. `config/twist_mux.yaml`  
   ```yaml
   topics:
     - topic: cmd_vel_joy
       timeout: 0.1
       priority: 2
     - topic: cmd_vel_nav
       timeout: 0.5
       priority: 1
   ```
2. `launch/twist_mux.launch.py`  
3. **测试**：按住 Enable → 手动，松开 → Nav2 接管。

---

## Step 9 · 真机过渡

| # | 任务 | 完成标志 |
|---|------|----------|
| 9‑1 | STM32 + CANopen 402 从站 | PDO 能汇报轮速 |
| 9‑2 | `socketcan_bridge` 验证 | `ros2 topic echo /from_can_bus` |
| 9‑3 | 替换 Gazebo diff_drive → 真机 `tracked_bot_motor_driver` | 地面实跑 NavigateToPose 成功 |

---

### 推荐日程

| Day | 目标 | 检查点 |
|-----|------|-------|
| 1‑2 | Step 1 | TF 树正确 |
| 3‑4 | Step 2 | Gazebo 可驱动 |
| 5 | Step 3 | 手柄控制 OK |
| 6‑7 | Step 4 | /odom 稳定 |
| 8‑9 | Step 5 | 保存地图 |
| 10 | Step 6 | 第一次自动导航 |
| 11‑12 | Step 7 | 避障顺滑 |
| 13 | Step 8 | 快速切换 |
| 14+ | Step 9 | 上车实测 |

---

> **故障排查小抄**  
> * Gazebo 无法连接 ROS：确认 `GAZEBO_MASTER_URI` 与 `gzserver` 端口。  
> * Nav2 "controller failed": 检查 `/cmd_vel` 是否被 twist_mux 阻塞。  
> * AMCL 漂移：调低 `laser_z_hit`，增大 `transform_tolerance`.

---

完成以上全部后，你的仿真环境即可 1:1 替换实车驱动层，实现"软件零改动"下线部署。祝开发顺利！
