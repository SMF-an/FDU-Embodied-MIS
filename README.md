# FDU-Embodied-MIS

本项目基于 **Ubuntu 22.04 + ROS 2 Humble** 开发，并使用睿尔曼（RealMan）机械臂的 ROS 2 功能包进行 Gazebo/MoveIt 2 仿真。

`src/ros2_rm_robot` 是本项目的 Git 子模块，固定到已验证的上游版本。请勿在该目录中单独克隆或随意切换分支。

## 获取源码

克隆项目时一并初始化子模块：

```bash
git clone --recurse-submodules <本项目仓库地址>
cd FDU-Embodied-MIS
```

若项目已克隆但缺少 `src/ros2_rm_robot`，在项目根目录执行：

```bash
git submodule update --init --recursive
```

## 安装环境与依赖

首次配置请使用 Ubuntu 22.04。子模块中包含 ROS 2 Humble、MoveIt 2 及机械臂功能包所需运行库的安装脚本；在项目根目录依次执行：

```bash
sudo bash src/ros2_rm_robot/rm_install/scripts/ros2_install.sh
sudo bash src/ros2_rm_robot/rm_install/scripts/moveit2_install.sh
sudo bash src/ros2_rm_robot/rm_driver/lib/lib_install.sh
```

安装完成后，编译本项目工作区。`rm_ros_interfaces` 必须先单独编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
```

之后每次打开新终端开发或运行项目前，均需进入项目根目录并加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 仿真说明

机械臂型号选择、Gazebo 启动命令和 MoveIt 2 控制方式请参考子模块中的 [rm_gazebo 使用说明](src/ros2_rm_robot/rm_gazebo/README_CN.md)。该文档是仿真环境和运行方式的详细参考；本 README 仅保留项目复现所需的子模块获取、依赖安装和编译步骤。
