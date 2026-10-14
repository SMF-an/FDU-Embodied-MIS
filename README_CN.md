# FDU-Embodied-MIS

面向 RealMan RM65-6FB 机械臂、RealSense 相机、RCM 约束运动和具身微创手术（MIS）任务实验的 ROS 2 Humble 工作空间。项目基于 Ubuntu 22.04 开发。英文版请见 [README.md](README.md).

> **复现范围：** 本仓库包含 ROS 2 驱动、Gazebo/MoveIt 仿真、双臂 RCM 遥操作、RGB-D 感知和任务状态机。没有实体机械臂、相机或夹爪时，可以先体验仿真、感知和网页界面。实机实验需要相应硬件、设备专属标定、对应的 phantom/材料，并确保硬件急停触手可及。仓库未提供可直接下载的训练模型或完整实验数据集。

## 0. 环境要求

- Ubuntu 22.04。原生 Ubuntu 环境最容易复现；WSL2 需要额外配置网络和 USB 透传，见下方链接的说明文档。
- ROS 2 Humble、MoveIt 2、ROS Gazebo/Harmonic 相关软件包、`colcon` 和 `rosdep`。
- 实机实验至少需要一台 RealMan RM65-6FB；双臂流程需要两台。相机为 Intel RealSense D405/D4xx 兼容设备。Feetech SCS215 夹爪为可选设备。
- ROS 2 节点应使用系统 Python。不要在 Conda 环境中运行 ROS 节点。安装夹爪 SDK 时使用 `/usr/bin/python3`。
- 命令中的 `/absolute/path/to/FDU-Embodied-MIS` 要替换为克隆仓库后的绝对路径。除非特别说明，工作空间命令均从仓库根目录执行。

仿真可以跳过实体机械臂、网络、USB 和 SCS215 配置，但仍需正确安装 ROS 2、MoveIt、Gazebo 并成功编译工作空间。

## 1. 获取源码

```bash
git clone https://github.com/SMF-an/FDU-Embodied-MIS.git
cd FDU-Embodied-MIS
```

## 2. 配置环境

先安装并加载 ROS 2 Humble，再安装项目依赖。仓库内的 RealMan 安装脚本会修改 apt 软件源、配置 pip 镜像，并尝试为 `%sudo` 组设置免密 sudo，不适合直接用于已有或共享机器。建议按照 [ROS 2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 安装 ROS，再确认实际需要哪些 RealMan 安装步骤。如果决定运行仓库脚本，请先检查其系统修改，并确认系统为 Ubuntu 22.04：

```bash
source /opt/ros/humble/setup.bash
sudo bash src/ros2_rm_robot/rm_install/scripts/ros2_install.sh
sudo bash src/ros2_rm_robot/rm_install/scripts/moveit2_install.sh
```

安装 RealMan C/C++ 库时，必须从脚本所在目录执行。脚本通过相对路径查找附带的 `.so` 文件：

```bash
cd src/ros2_rm_robot/rm_driver/lib
sudo bash ./lib_install.sh
cd /absolute/path/to/FDU-Embodied-MIS
```

安装项目使用的其他 ROS 软件包：

```bash
sudo apt update
sudo apt install \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions \
  python3-rosdep
```

首次编译前安装仓库依赖。如果系统使用 `rosdepc`，将下面命令中的 `rosdep` 替换为 `rosdepc`：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

先编译 ROS 接口，再编译整个工作空间：

```bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
```

如果只想编译感知、任务执行或遥操作与 bringup 相关包：

```bash
colcon build --packages-up-to mis_perception
colcon build --packages-up-to mis_task_manager
colcon build --packages-select rm_rcm_web_teleop rm_bringup
```

每个新终端都要加载 ROS 和工作空间环境（将示例路径替换为仓库实际位置）：

```bash
cd /absolute/path/to/FDU-Embodied-MIS
source /opt/ros/humble/setup.bash
source install/setup.bash
```

修改源码后使用 `colcon build` 重新编译，也可以使用 `colcon build --packages-select <package>` 单独编译指定包。WSL2 硬件配置请参考[机械臂网络配置](docs/机械臂与电脑网络通信配置说明.md)、[RealSense USB 配置](docs/深度相机与电脑通信配置说明.md)和 [SCS215 串口配置](docs/SCS215舵机与电脑连接配置说明.md)。

### 配置文件与运行时文件

首次使用前请检查以下文件：

| 文件 | 用途 | 是否需要修改 |
| --- | --- | --- |
| `src/ros2_rm_robot/rm_driver/config/rm_65_config.yaml` | 单臂 IP、UDP 回报地址、TCP 和工具参数 | 实机使用时按当前网络配置 |
| `src/ros2_rm_robot/rm_driver/config/rm_65_left_config.yaml`、`rm_65_right_config.yaml` | 双臂 IP、关节名和 UDP 参数 | 使用双臂实机前核对 |
| `src/ros2_rm_robot/rm_rcm_web_teleop/config/scs215.yaml` | 夹爪串口、ID、波特率、开合位置 | 使用夹爪时需要配置 |
| `data/calibration/dual_rcm_points.json` | `world` 坐标系下的双臂 RCM 点，单位为米 | 实机需要重新测量或验证 |
| `data/calibration/robot_checkerboard.json` | 相机到 `world` 坐标系的外参 | 没有有效 TF 且感知需要世界坐标时需要 |

`data/calibration/`、`data/teachings/` 和视频输出目录初始时可能不存在，程序需要时会创建。也可以用 `mkdir -p data/calibration data/teachings` 创建标定和教学目录。标定文件和教学文件都与具体设备有关；未经验证不要直接使用其他设备的文件。

## 3. 启动硬件

每个组件在独立终端中运行，并确保已经加载 ROS 和工作空间环境。连接实机前，检查 `src/ros2_rm_robot/rm_driver/config/` 中的 IP、`udp_ip`、`udp_port`，并确认机械臂处于可安全使能的状态。

### 机械臂驱动

单台 RM65-6FB 实机，启动 MoveIt 和 RViz：

```bash
ros2 launch rm_bringup rm_65_6fb_bringup.launch.py
```

单臂 RCM 遥操作需要在 MoveIt 的 `world` 坐标系中提供经过标定的固定点：

```bash
ros2 launch rm_bringup rm_65_6fb_rcm_real.launch.py \
  derive_rcm_from_current:=false rcm_x:=<x_m> rcm_y:=<y_m> rcm_z:=<z_m>
```

启动双臂驱动、MoveIt 和 RCM 节点：

```bash
ros2 launch rm_bringup rm_65_dual_6fb_rcm_real.launch.py
```

双臂 RCM 启动文件会在可用时从 `data/calibration/dual_rcm_points.json` 加载 `world` 坐标系下的标定点。没有有效文件时，它会根据初始工具轴推算 RCM 点；此值仅用于软件启动，并非实测的 trocar 位置。

仿真请参阅 [Gazebo 和 MoveIt 文档](src/ros2_rm_robot/rm_gazebo/README_CN.md)。RM65 RCM 仿真的入口命令为：

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch rm_bringup rm_65_6fb_rcm_gazebo.launch.py
```

### RealSense 相机

安装前文所述 ROS 软件包并连接相机，然后启动 RGB 和深度图像流：

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  align_depth.enable:=true \
  depth_module.color_profile:=480x270x5 \
  depth_module.depth_profile:=480x270x5
```

双臂遥操作和任务执行默认使用 `/camera/camera/color/image_raw`。根据设备调整相机 profile 或 topic 参数。WSL2 USB 透传和已测试的低带宽 profile 见[相机配置说明](docs/深度相机与电脑通信配置说明.md)。

在另一个终端检查相机话题：

```bash
ros2 topic list | grep -E 'camera/camera/(color|aligned_depth_to_color)|joint_states'
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo --once /camera/camera/color/camera_info
```

如果没有图像流，不要启动感知或启用机械臂运动。先按 WSL2/USB 说明检查当前系统能否识别设备。

### SCS215 夹爪

双臂网页遥操作程序通过 USB 直接控制 SCS215 舵机，不会启动舵机 ROS 节点，也不会发布舵机 topic。安装 SDK：

```bash
/usr/bin/python3 -m pip install --user ftservo-python-sdk
```

在 `src/ros2_rm_robot/rm_rcm_web_teleop/config/scs215.yaml` 中设置各舵机的端口、ID、波特率和开合位置。启用扭矩前确认端口对应关系并清空夹爪周围区域。夹爪使用位置控制，没有力反馈或 ROS 状态话题。

## 4. 标定

### 工具 TCP

工具长度变化时，需要更新驱动 TCP 偏移、`SurgicalToolTip`，以及 `rm_65.urdf.xacro` 和 `rm_65_gazebo.urdf.xacro` 中的工具几何；必要时还要更新负载参数。单位为米。重新编译 `rm_driver`、`rm_description`、`rm_65_config` 和 `rm_gazebo`，然后在 RViz 或 Gazebo 中验证工具尖端。TCP 长度与 RCM 距离是不同的参数。当前模型假设工具沿工具坐标系局部 `+Z` 方向延伸；其他偏移需要完整 TCP 标定。

### 双臂 RCM 点

启动两侧实机 RCM 链。标定前停止网页遥操作，并确认没有其他控制器正在移动机械臂。编译后运行：

```bash
ros2 run rm_rcm_web_teleop dual_rcm_calibrate
```

输入 `s left` 或 `s right` 启用手动拖动。将工具尖端对准 trocar 中心，输入 `r left` 或 `r right` 记录；保持尖端在该位置，再输入 `a left` 或 `a right` 验证并保存。工具轴误差必须小于 2 mm。输入 `x left` 或 `x right` 取消对应侧，输入 `q` 退出。

标定结果以米为单位，保存在 MoveIt 的 `world` 坐标系中，路径为 `data/calibration/dual_rcm_points.json`。文件必须采用当前的 `rm_dual_rcm_points_v2` 格式。重新启动双臂 RCM 后确认文件已加载且点位正确。详情见[网页遥操作指南](src/ros2_rm_robot/rm_rcm_web_teleop/README.md)。

### 相机到世界坐标系的外参

进行 eye-to-hand 标定时，启动相机和双臂 RCM。保持相机和棋盘格固定，然后运行：

```bash
ros2 run rm_rcm_web_teleop robot_checkerboard_calibrate \
  --image-topic /camera/camera/color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --base-frame world \
  --tool-frame SurgicalToolTip \
  --square-size-mm 20 \
  --output data/calibration/robot_checkerboard.json
```

按 `1` 选择左臂，按 `2` 选择右臂。点击一个棋盘格内角点，用工具尖端轻触该点，机械臂静止时按 `c` 采样。每侧至少采集 8 个分布均匀的点，然后按 `s` 拟合。使用标定结果前检查输出的残差和 RMS 误差。结果保存到 `data/calibration/robot_checkerboard.json`。

## 5. 使用机器人功能

### RCM 遥操作

先启动 RCM 机械臂节点，再在另一个已加载环境的终端启动网页界面。

单臂仿真：

```bash
ros2 launch rm_rcm_web_teleop web_teleop.launch.py
```

单臂实机：

```bash
ros2 launch rm_rcm_web_teleop web_teleop.launch.py use_sim_time:=false
```

双臂实机（相机和 SCS215 配置已就绪）：

```bash
ros2 launch rm_rcm_web_teleop dual_web_teleop.launch.py
```

打开 `http://127.0.0.1:8765`。界面支持倾转、插入/回撤、双臂模式下的轴向旋转、夹爪控制和示教录制。双臂模式还支持 RGB 视频流和录制。只有在可信网络中才使用 `host:=0.0.0.0` 开放局域网访问。网页停止按钮发出的是软件停止请求，不能替代控制器急停。步长、标定、示教和回放细节见[遥操作指南](src/ros2_rm_robot/rm_rcm_web_teleop/README.md)。

### Codex Agent 模式

仓库根目录的 `AGENTS.md` 是 Codex 执行任务时使用的说明文件，定义观察循环、动作格式、任务限制和停止条件。Agent 模式使用在线图像伺服：Codex 读取最新 RGB 图像和机械臂状态，发送一个小幅归一化 RCM 动作，等待完成后再次观察。它不使用传统 `mis_perception` 任务流程，控制环也不需要相机到世界坐标系的外参；但仍需要已标定的 RCM 点。

先启动相机、两侧 RCM 节点和双臂遥操作程序。检查系统时保持 Agent 控制禁用：

```bash
ros2 launch rm_rcm_web_teleop dual_web_teleop.launch.py \
  agent_enabled:=false
```

检查工作空间、RCM 点、相机画面、夹爪映射和实体急停后，再显式启用 Agent 接口：

```bash
ros2 launch rm_rcm_web_teleop dual_web_teleop.launch.py \
  agent_enabled:=true agent_max_action_hz:=2.0
```

在同一 Codex 会话中输入：`开始任务，并严格遵循仓库根目录 AGENTS.md。`

Agent 使用以下本机 allowlist 接口：

```text
GET  http://127.0.0.1:8765/api/agent/observe
POST http://127.0.0.1:8765/api/agent/action
POST http://127.0.0.1:8765/api/stop
```

每个动作只包含一侧机械臂和一个小幅增量。运动和夹爪命令需要分别发送：

```json
{
  "arm": "left",
  "tilt_x": 0.0,
  "tilt_y": -0.25,
  "insertion": 0.0,
  "roll": 0.0,
  "gripper": "hold",
  "confidence": 0.86
}
```

运动值是 `[-1, 1]` 范围内的归一化增量，由本地控制器进行频率限制和检查。如果图像、目标、工具尖端、RCM 状态或动作结果不明确，应停止任务。软件停止不能代替实体急停。仓库说明仅提供行为约束，不是操作系统安全边界；如果需要限制 Agent 读取其他文件，应使用受限工作空间或容器。

任务开始时可能因插入不足而看不到器械尖端。如果所选机械臂状态报告 `rcm_initializing: true`，先用小幅轴向插入/回撤动作；初始化期间倾转和旋转会被拒绝。当前控制器把“尖端距 RCM 点 20 mm 以内”定义为初始化状态。这与 RCM 标定时工具轴误差小于 2 mm 的要求不同。

### 双臂运动示例

也可以单独启动双臂驱动：

```bash
ros2 launch rm_bringup rm_65_dual_bringup.launch.py
```

`rm_example` 提供 `rm_65_dual_movej` 关节目标示例和 `rm_65_dual_pose` 笛卡尔位姿示例。发送运动命令前确认目标值、单位和碰撞间隙。这些命令分别向两侧发送目标，不保证同步运动。

### 感知和任务执行

先启动相机和双臂 RCM，再通过以下命令一起启动感知、任务执行和面板：

```bash
ros2 launch mis_task_manager task_execution.launch.py
```

默认参数为 `motion_enabled:=false` 和 `dry_run:=true`。检查感知、标定、坐标系和工作空间后，如需实机执行，再显式开启运动：

```bash
ros2 launch mis_task_manager task_execution.launch.py \
  motion_enabled:=true dry_run:=false \
  target_frame:=world \
  camera_extrinsics_file:=/absolute/path/to/FDU-Embodied-MIS/data/calibration/robot_checkerboard.json \
  camera_extrinsics_arm:=left \
  lesion_profile:=painted
```

如果已经有有效的相机到 `world` TF，可省略外参参数。打开 `http://127.0.0.1:8780` 查看系统就绪状态和任务状态。**Start** 按钮每个进程只启动一个任务。**Emergency Stop** 按钮发送软件停止命令，不能代替实体急停。

当前任务执行器会执行短程、受 RCM 约束的视觉伺服动作，移动到相对于物体的若干路径点。它尚未实现组织暴露、沿边界切割、抓取或病灶取出。启用运动前请阅读[任务管理器说明](src/ros2_rm_robot/mis_task_manager/README.md)和[感知说明](src/ros2_rm_robot/mis_perception/README.md)。

### 故障排查

- `Package 'rm_bringup' not found`：当前终端可能没有加载工作空间，或编译失败。在仓库根目录运行 `source /opt/ros/humble/setup.bash` 和 `source install/setup.bash`，然后检查 `colcon build` 的第一个错误。
- 驱动无法连接机械臂：先执行 `ping <arm_ip>`，然后检查 `arm_ip`、主机网卡、`udp_ip`/`udp_port` 和防火墙。IP 配置在 YAML 文件中，不仅由 launch 参数控制。
- 找不到 `libapi_cpp.so` 或 `libRM_Service.so`：确认从 `src/ros2_rm_robot/rm_driver/lib` 目录执行了库安装，并且 `sudo ldconfig` 已完成。可用 `ldconfig -p | grep -E 'libapi_cpp|libRM_Service'` 检查。
- 相机节点已启动但没有图像：检查 USB 3 连接、`lsusb`/`rs-enumerate-devices` 结果，以及是否有其他程序正在占用相机。尝试降低分辨率或帧率。
- 感知发布了 mask 但没有世界坐标：检查 `target_frame:=world`、相机光学坐标系的 TF，或提供通过 `--base-frame world` 标定的棋盘格外参。
- 任务面板的 Start 按钮不可用：检查 `/perception/scene` 是否新鲜并包含有效目标、两侧机械臂是否 RCM ready，以及 MoveIt 是否确认了当前障碍物版本。先用默认 `motion_enabled:=false dry_run:=true` 模式验证。
- 网页可以打开但夹爪不动：检查 `/dev/serial/by-id/`、`scs215.yaml` 中的端口/ID/波特率/电源，并确认 SDK 安装在 `/usr/bin/python3` 对应的 Python 环境中。夹爪没有力反馈。

### 推荐复现顺序

1. 启动 RM65 Gazebo RCM 仿真，确认 RViz、MoveIt 和网页遥操作可用。
2. 启动 RealSense（或等效的录制/回放图像源），运行 `mis_perception`，检查 `/perception/overlay`、mask 和 `/perception/scene`。
3. 使用 `task_execution.launch.py` 的默认 dry-run 参数检查状态机和面板，然后再检查标定、碰撞盒和 RCM 状态。
4. 最后连接实体机械臂。低速检查单臂 RCM、双臂 RCM、相机外参和夹爪。进行任何实机运动前，确保实体急停触手可及。

### 仿真和 TCP 参考

支持的机械臂型号和 Gazebo/MoveIt 通用操作见 [Gazebo 文档](src/ros2_rm_robot/rm_gazebo/README_CN.md)。RM65 TCP 模型文件及其更新步骤见上方标定章节。
