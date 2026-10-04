# RM65 RCM Web Teleop

独立的浏览器遥控组件。它不启动机器人或 Gazebo，依赖正在运行的 `rm65_rcm_demo` 节点。仿真中使用：

```bash
# 终端 1：启动 RM65 RCM Gazebo 仿真
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch rm_bringup rm_65_6fb_rcm_gazebo.launch.py

# 终端 2：启动独立 Web 遥控器
ros2 launch rm_rcm_web_teleop web_teleop.launch.py
```

打开 `http://127.0.0.1:8765`。按住并偏转圆形摇杆，每次完成一步器具倾转；按住“向前插入”或“向后退出”按钮，机械臂每次沿器具轴线移动一步。默认每步为 2 度或 2 mm，当前轨迹完成后才会发送下一步。停止按钮调用 MoveIt 的 `stop()` 中止当前规划执行轨迹。

Web 节点默认只监听 `127.0.0.1`，默认使用仿真时钟。页面的“仿真/真机”标签根据 `use_sim_time` 参数显示；仿真设为 `true`，真机设为 `false`。可在 launch 中覆盖 `host`、`port` 和 `use_sim_time`。浏览器服务通过以下 ROS 2 话题连接 RCM demo：

- 订阅 `/rcm_demo/current_tool_pose`、`/rcm_demo/rcm_point` 和 `/rcm_demo/motion_busy` 获取状态。
- 发布 `/rcm_demo/web_target` 发送一步目标，RCM demo 负责规划、约束校验和执行。
- 发布 `/rcm_demo/stop` 请求停止当前 MoveIt 轨迹。

## 真机使用

该组件可以连接 `rm65_rcm_demo` 真机执行链路。先按项目主 README 的“单臂真机 RCM 验证”启动并验证驱动、MoveIt 和 RCM 节点，然后在另一个加载同一工作区的终端启动 Web 节点：

```bash
ros2 launch rm_rcm_web_teleop web_teleop.launch.py use_sim_time:=false
```

浏览器与 ROS 2 主机不在同一台电脑时，将 `host` 设为 `0.0.0.0` 并使用主机的局域网地址访问，例如 `ros2 launch rm_rcm_web_teleop web_teleop.launch.py use_sim_time:=false host:=0.0.0.0`。请仅在可信局域网开放此端口。

真机启动 RCM 节点时必须使用已标定的固定远心点，不要使用默认的当前轴线推导值。例如将真机 launch 的 `derive_rcm_from_current` 设为 `false`，并传入以 `base_link` 为坐标系的标定值：

```bash
ros2 launch rm_bringup rm_65_6fb_rcm_real.launch.py \
  derive_rcm_from_current:=false rcm_x:=<x_m> rcm_y:=<y_m> rcm_z:=<z_m> \
  velocity_scaling:=0.02 acceleration_scaling:=0.02 \
  max_rcm_rotation_deg:=2.0 max_rcm_insertion_m:=0.002
```

将 `<x_m>`、`<y_m>`、`<z_m>` 替换为实测标定坐标。首次上机先在清空工作区、低速条件下验证状态反馈、RCM 位置和小幅单步动作。网页的停止按钮只是软件停止请求，不替代控制柜急停或现场安全措施。

## 双臂真机使用

双臂入口为左右臂分别启动 MoveIt 和 RCM 节点，再由双栏 Web 页面分别控制。启动时必须提供两臂各自在 `base_link` 坐标系下标定的 RCM 坐标；漏传参数时默认使用非有限值，RCM 节点会拒绝初始化。

```bash
ros2 launch rm_bringup rm_65_dual_6fb_rcm_real.launch.py \
  left_rcm_x:=LEFT_X_M left_rcm_y:=LEFT_Y_M left_rcm_z:=LEFT_Z_M \
  right_rcm_x:=RIGHT_X_M right_rcm_y:=RIGHT_Y_M right_rcm_z:=RIGHT_Z_M
```

将 `LEFT_X_M` 等占位符替换为实际标定值。之后在另一个终端启动双臂网页：

```bash
ros2 launch rm_rcm_web_teleop dual_web_teleop.launch.py
```

页面左侧控制左臂，右侧控制右臂。任一侧步进执行期间，另一侧会暂时锁定。两套单臂 MoveIt 会话各自在本臂模型内规划；目前没有双臂联合碰撞检测，操作前必须确认工作空间分离，并确保没有其他程序同时下发运动命令。双臂网页停止按钮只向对应手臂发送软件停止请求，不替代控制柜急停。
