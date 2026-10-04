# FDU-Embodied-MIS

本项目基于 **Ubuntu 22.04 + ROS 2 Humble** 开发，并使用睿尔曼（RealMan）机械臂的 ROS 2 功能包进行 Gazebo/MoveIt 2 仿真。

`src/ros2_rm_robot` 是本项目源码目录，与其他项目文件一并由本仓库管理。

## 获取源码

克隆项目：

```bash
git clone https://github.com/SMF-an/FDU-Embodied-MIS.git
cd FDU-Embodied-MIS
```

## 安装环境与依赖

首次配置请使用 Ubuntu 22.04。`src/ros2_rm_robot` 中包含 ROS 2 Humble、MoveIt 2 及机械臂功能包所需运行库的安装脚本；在项目根目录依次执行：

```bash
sudo bash src/ros2_rm_robot/rm_install/scripts/ros2_install.sh
sudo bash src/ros2_rm_robot/rm_install/scripts/moveit2_install.sh
sudo bash src/ros2_rm_robot/rm_driver/lib/lib_install.sh
```

安装完成后，编译本项目工作区。`rm_ros_interfaces` 必须先单独编译：

```bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
```

每次修改机械臂代码后，都需要在项目根目录重新编译：

```bash
colcon build
```

如果只改了一个功能包，可以只单独编译对应的功能包。比如：

```bash
colcon build --packages-select rm_driver
```

之后每次打开新终端开发或运行项目前，均需进入项目根目录并加载环境：

```bash
source install/setup.bash
```

## 双 RM65-6FB 机械臂

双臂驱动配置分别连接 `192.168.1.18`（左臂）和 `192.168.1.19`（右臂），并使用不同 UDP 端口。启动两台驱动：

```bash
ros2 launch rm_bringup rm_65_dual_bringup.launch.py
```

在另一个已加载工作区环境的终端中，分别给两臂传入 6 个关节目标。默认单位为弧度；下面通过 `joint_unit:=degrees` 显式使用角度。以下目标仅为命令格式示例，执行前应替换为已确认安全的目标，并确认两臂运动空间没有障碍物。运行后会向两台驱动分别发布一次 MoveJ 指令，并等待两臂执行结果（最多 120 秒）；这不保证两臂严格同步：

```bash
ros2 run rm_example rm_65_dual_movej --ros-args \
  -p left_joints:="[20.0, 18.0, -70.0, 0.0, 50.0, -20.0]" \
  -p right_joints:="[50.0, -30.0, 40.0, 0.0, -10.0, -40.0]" \
  -p joint_unit:=degrees \
  -p speed:=10
```

也可以指定左右臂的末端目标位姿。数组顺序为 `[x,y,z,qx,qy,qz,qw]`，位置单位是米，姿态使用四元数；`movej_p` 以关节路径到达目标，`movel` 让末端沿直线运动。目标相对于控制器当前工作坐标系；两臂分别下发，执行结果分别报告：

```bash
ros2 run rm_example rm_65_dual_pose --ros-args \
  -p left_pose:="[0.07, 0.05, 0.7, 0.0, 0.0, -1.0, 0.0]" \
  -p right_pose:="[0.07, 0.05, 0.7, 0.0, 0.0, -1.0, 0.0]" \
  -p motion:=movej_p \
  -p speed:=10
```

将示例中的中文占位符替换为实际数值后再运行。若要末端直线运动，将 `motion` 改为 `movel`。四元数会自动归一化；本示例不做两臂之间的碰撞检测，运行前请确认两个目标和运动路径安全。位姿位置单位为米，姿态使用四元数传入；控制器当前工作坐标系和接口定义见[官方 MoveJ_P/MoveL 说明](https://develop.realman-robotics.com/robot4th/apic/classes/movePlan/)。

注意：`rm_65_bringup.launch.py` 是单臂入口，不能替代上述双臂启动命令。启动进程正常不等于两台机械臂都已连接，应检查驱动连接日志和实际关节状态。Shell 续行符 `\` 后不能有空格。执行结果话题没有命令 ID，运行该示例时应避免其他节点同时发送 MoveJ 指令；等待超时也不代表机械臂已停止。

双臂配置启用 `verify_movej: true`：运动前必须读到真实模式（`mode=1`），阻塞 MoveJ 返回 0 后还要读取实际关节并核对目标（容差 1°）。查询失败、仿真模式或关节未到位都会报告失败；驱动不会自动切换控制器模式。出现 `rc=-2` 表示数据接收失败或控制器未及时回复，应先检查重复连接进程、控制器模式和网络通信。两臂配置中的 `udp_ip` 必须是控制器可以访问的本机网卡地址。

该入口针对两台 RM65-6FB，提供双驱动连接和 6 关节角直控。双臂 RCM 入口为每条手臂启动独立的单臂 MoveIt 会话，保留左右驱动的关节名前缀。它不建立包含两台手臂的联合碰撞模型，因此不会检测两臂之间的碰撞；运动前必须确认两臂工作空间分离且没有其他程序同时控制机械臂。

### 双臂真机 RCM 与 Web 遥控

双臂 RCM 启动要求左、右臂分别完成 RCM 标定。`left_rcm_x/y/z` 和 `right_rcm_x/y/z` 都是各自机械臂 `base_link` 坐标系下的实测坐标；默认值无效，未提供坐标时节点会拒绝启动。启动双臂驱动、两套 MoveIt/RCM 节点和关节状态桥接。将六个 RCM 参数替换为标定数值；启动参数 `max_rcm_rotation_deg` 和 `max_rcm_insertion_m` 限制单次目标的倾转与轴向位移，真机默认分别为 2° 和 2 mm：

```bash
ros2 launch rm_bringup rm_65_dual_6fb_rcm_real.launch.py \
  left_rcm_x:=LEFT_X_M left_rcm_y:=LEFT_Y_M left_rcm_z:=LEFT_Z_M \
  right_rcm_x:=RIGHT_X_M right_rcm_y:=RIGHT_Y_M right_rcm_z:=RIGHT_Z_M
```

在另一个已加载工作区环境的终端启动左右双栏 Web 遥控器：

```bash
ros2 launch rm_rcm_web_teleop dual_web_teleop.launch.py
```

浏览器打开 `http://127.0.0.1:8765`。左栏只发送到左臂，右栏只发送到右臂；Web 服务会阻止两侧同时发送步进目标。将浏览器放在另一台电脑时可用 `host:=0.0.0.0` 并通过 ROS 主机的局域网地址访问。Web 互斥不替代 MoveIt 双臂碰撞检测或硬件急停；当前双臂配置没有跨臂碰撞检测。

## RM65-6FB 单臂 RCM 约束运动仿真示例：

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch rm_bringup rm_65_6fb_rcm_gazebo.launch.py
```

示例以 `SurgicalToolTip` 为末端，启动后保持机械臂当前位姿，不再自动摆动。远心点默认设为当前器具轴线上距尖端 0.15 m 处。仿真机械臂从 `[50, -30, 40, 0, -10, -40]` 度的非奇异姿态启动，避免全零初始姿态导致笛卡尔逆解失败。RViz 中红点表示 RCM 点，蓝线表示器具轴线；红色箭头用于沿器具轴线插入/退出，绿色球用于倾转目标。松开鼠标后，路径会保持器具轴线穿过远心点，并经规划、校验后执行。若轴向路径完成度达到 85% 但未到达目标，会执行已校验的可达部分，之后可从当前位置继续拖动。若要使用规划坐标系（默认 `base_link`）中的固定远心点，可设置 `derive_rcm_from_current:=false` 并传入 `rcm_x`、`rcm_y`、`rcm_z`。该点必须位于启动时器具轴线上，否则节点会拒绝运动。

### 单臂真机 RCM 验证

真机入口复用 RM65-6FB 的驱动、`rm_control`、MoveIt 和 RViz bringup，并增加 RCM 约束节点；不会启动 Web 遥控器。先在 `src/ros2_rm_robot/rm_driver/config/rm_65_config.yaml` 核对机械臂 IP，并确认机械臂处于真实模式、运动区域清空且没有其他程序控制机械臂。编译并加载工作区后，只启动下面这个入口，它会一并启动驱动、MoveIt、RViz 和 RCM 节点：

```bash
ros2 launch rm_bringup rm_65_6fb_rcm_real.launch.py
```

RViz 中拖动 `SurgicalToolTip` 的交互目标，松开后节点会生成并验证保持器具轴线穿过 RCM 点的笛卡尔轨迹，再交给真机控制器执行。真机默认速度和加速度缩放均为 0.02；RViz 单次拖动默认不设 2° 倾转或 2 mm 轴向移动限幅。RCM 轨迹约束仍会验证并保持器具轴线穿过远心点，MoveIt 可达性和关节限制也仍然生效。可通过 `velocity_scaling`、`acceleration_scaling`、`max_rcm_rotation_deg`、`max_rcm_insertion_m` launch 参数调整；两个 `max_rcm_*` 参数设为正数时启用对应单次限幅，设为 `0` 时关闭。

真机 launch 默认仍以当前尖端轴线向后 0.15 m 推导 RCM 点，这只适合验证软件约束流程，并不代表已校准到实际穿刺孔/套管中心。要验证真实远心点，先在 `base_link` 坐标系中完成标定，再以 `derive_rcm_from_current:=false` 和 `rcm_x`、`rcm_y`、`rcm_z` 传入固定坐标。确认 RViz 中 RCM 点位于器具轴线上后，再做小幅运动测试。可用 RViz MotionPlanning 面板的 Stop 按钮停止当前轨迹；停止 launch 进程也会终止该节点。

### Web 遥控器

RCM 仿真或真机节点启动后，在另一个已加载工作区环境的终端启动 Web 遥控器。仿真使用默认参数：

```bash
ros2 launch rm_rcm_web_teleop web_teleop.launch.py
```

真机启动时关闭 Web 节点的仿真时钟：

```bash
ros2 launch rm_rcm_web_teleop web_teleop.launch.py use_sim_time:=false
```

在浏览器打开 `http://127.0.0.1:8765`。按住并偏转摇杆控制器具倾转，按住“向前插入 / 向后退出”按键控制轴向运动；每次只发送一个小步，完成后继续发送。停止按钮会请求 MoveIt 停止当前轨迹。若浏览器运行在另一台电脑，可加 `host:=0.0.0.0` 并通过 ROS 主机的局域网地址访问。Web 默认步长为 2° 和 2 mm；真机首次验证应在 `rm_65_6fb_rcm_real.launch.py` 中启用对应单步限幅、使用已标定的固定 RCM 坐标，并从低速开始。网页停止按钮是软件停止请求，不替代控制柜急停。详细说明见 [Web RCM 遥控器](src/ros2_rm_robot/rm_rcm_web_teleop/README.md)。

## 仿真说明

机械臂型号选择、Gazebo 启动命令和 MoveIt 2 控制方式请参考 [rm_gazebo 使用说明](src/ros2_rm_robot/rm_gazebo/README_CN.md)。该文档是仿真环境和运行方式的详细参考；本 README 保留项目源码获取、依赖安装和编译步骤。
