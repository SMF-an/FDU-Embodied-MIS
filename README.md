# FDU-Embodied-MIS

ROS 2 Humble workspace for RealMan RM65-6FB arms, RealSense cameras, RCM-constrained motion, and embodied minimally invasive surgery (MIS) task experiments. The project is developed on Ubuntu 22.04. See the [Chinese version](README_CN.md).

> **Reproduction scope:** This repository includes ROS 2 drivers, Gazebo/MoveIt simulation, dual-arm RCM teleoperation, RGB-D perception, and a task state machine. Simulation, perception, and the web interface can be explored without the physical robot, camera, or grippers. Real-robot experiments require the hardware, device-specific calibration, the relevant phantom/material, and an accessible hardware emergency stop. The repository does not provide downloadable trained models or a complete experimental dataset.

## 0. Prerequisites

- Ubuntu 22.04. Native Ubuntu is the most straightforward setup. WSL2 requires additional network and USB passthrough configuration; see the guides linked below.
- ROS 2 Humble, MoveIt 2, the ROS Gazebo/Harmonic packages, `colcon`, and `rosdep`.
- Real runs require at least one RealMan RM65-6FB arm; dual-arm workflows require two. The camera is an Intel RealSense D405/D4xx-compatible device. Feetech SCS215 grippers are optional.
- Use the system Python installation used by ROS 2. Do not run ROS nodes from a Conda environment. Install the gripper SDK with `/usr/bin/python3`.
- Replace `/absolute/path/to/FDU-Embodied-MIS` in commands with the absolute path to your clone. Workspace commands assume the current directory is the repository root unless stated otherwise.

Simulation can skip real-arm, network, USB, and SCS215 setup, but still requires ROS 2, MoveIt, Gazebo, and a successful workspace build.

## 1. Get the Source

```bash
git clone https://github.com/SMF-an/FDU-Embodied-MIS.git
cd FDU-Embodied-MIS
```

## 2. Configure the Environment

Install and source ROS 2 Humble before installing dependencies. The bundled RealMan installer changes apt sources, configures a pip mirror, and attempts to enable passwordless sudo for the `%sudo` group. This is unsuitable for existing or shared machines. We recommend installing ROS using the [official ROS 2 Humble instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), then reviewing which RealMan installation steps are needed. If you choose to run the bundled scripts, inspect their system changes first and confirm that the system is Ubuntu 22.04:

```bash
source /opt/ros/humble/setup.bash
sudo bash src/ros2_rm_robot/rm_install/scripts/ros2_install.sh
sudo bash src/ros2_rm_robot/rm_install/scripts/moveit2_install.sh
```

Install the RealMan C/C++ libraries from the script directory. The script uses relative paths to locate the bundled `.so` files:

```bash
cd src/ros2_rm_robot/rm_driver/lib
sudo bash ./lib_install.sh
cd /absolute/path/to/FDU-Embodied-MIS
```

Install additional ROS packages used by this project:

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

Resolve repository dependencies before the first build. If your system uses `rosdepc`, substitute it for `rosdep` below:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the ROS interfaces first, then the workspace:

```bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
```

To build only perception, task execution, or teleoperation and bringup packages:

```bash
colcon build --packages-up-to mis_perception
colcon build --packages-up-to mis_task_manager
colcon build --packages-select rm_rcm_web_teleop rm_bringup
```

In every new terminal, source both ROS and the workspace (replace the example path with your clone location):

```bash
cd /absolute/path/to/FDU-Embodied-MIS
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Rebuild after source changes with `colcon build`, or build one package with `colcon build --packages-select <package>`. For WSL2 hardware setup, see [robot network setup](docs/机械臂与电脑网络通信配置说明.md), [RealSense USB setup](docs/深度相机与电脑通信配置说明.md), and [SCS215 serial setup](docs/SCS215舵机与电脑连接配置说明.md).

### Configuration and Runtime Files

Review these files before first use:

| File | Purpose | Must be changed? |
| --- | --- | --- |
| `src/ros2_rm_robot/rm_driver/config/rm_65_config.yaml` | Single-arm IP, UDP reporting address, TCP, and tool parameters | Set for the real network |
| `src/ros2_rm_robot/rm_driver/config/rm_65_left_config.yaml`, `rm_65_right_config.yaml` | Dual-arm IP addresses, joint names, and UDP parameters | Verify for both real arms |
| `src/ros2_rm_robot/rm_rcm_web_teleop/config/scs215.yaml` | Gripper serial port, ID, baud rate, and open/close positions | Required when using grippers |
| `data/calibration/dual_rcm_points.json` | Dual-arm RCM points in meters in the `world` frame | Measure or verify for real arms |
| `data/calibration/robot_checkerboard.json` | Camera-to-`world` extrinsics | Required for world-frame perception without a valid TF |

The `data/calibration/`, `data/teachings/`, and video output directories may not exist initially; programs create them when needed. You can also create the calibration and teaching directories with `mkdir -p data/calibration data/teachings`. Calibration and teaching files are device-specific and should not be copied from another setup without validation.

## 3. Start the Hardware

Run each component in its own terminal after sourcing ROS and the workspace. Before connecting real arms, check the IP, `udp_ip`, and `udp_port` values in `src/ros2_rm_robot/rm_driver/config/`, and confirm that the robot is in a safe state for enabling.

### Robot Drivers

For one real RM65-6FB arm with MoveIt and RViz:

```bash
ros2 launch rm_bringup rm_65_6fb_bringup.launch.py
```

For single-arm RCM teleoperation, supply a calibrated fixed point in the MoveIt `world` frame:

```bash
ros2 launch rm_bringup rm_65_6fb_rcm_real.launch.py \
  derive_rcm_from_current:=false rcm_x:=<x_m> rcm_y:=<y_m> rcm_z:=<z_m>
```

For both arms with drivers, MoveIt, and RCM nodes:

```bash
ros2 launch rm_bringup rm_65_dual_6fb_rcm_real.launch.py
```

When available, the dual-arm RCM launch loads `world`-frame calibration from `data/calibration/dual_rcm_points.json`. Without a valid file, it derives each point from the initial tool axis. This is a software setup value, not a measured trocar location.

For simulation, see the [Gazebo and MoveIt guide](src/ros2_rm_robot/rm_gazebo/README_CN.md). The RM65 RCM simulation entry point is:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch rm_bringup rm_65_6fb_rcm_gazebo.launch.py
```

### RealSense Camera

Install the ROS package above, connect the camera, then start RGB and depth streams:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  align_depth.enable:=true \
  depth_module.color_profile:=480x270x5 \
  depth_module.depth_profile:=480x270x5
```

Dual-arm teleoperation and task execution use `/camera/camera/color/image_raw` by default. Adjust camera profiles or topic arguments for your device. For WSL2 USB passthrough and tested low-bandwidth profiles, see the [camera setup guide](docs/深度相机与电脑通信配置说明.md).

Verify the camera topics from another terminal:

```bash
ros2 topic list | grep -E 'camera/camera/(color|aligned_depth_to_color)|joint_states'
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo --once /camera/camera/color/camera_info
```

If there is no image stream, do not start perception or enable robot motion. Check that the device is visible to the current system using the WSL2/USB guide.

### SCS215 Grippers

The dual-arm web teleoperator controls SCS215 servos directly over USB. It does not start a servo ROS node or publish servo topics. Install the SDK:

```bash
/usr/bin/python3 -m pip install --user ftservo-python-sdk
```

Set each port, servo ID, baud rate, and open/close position in `src/ros2_rm_robot/rm_rcm_web_teleop/config/scs215.yaml`. Verify port mapping and clear the gripper area before enabling torque. Control is position-based and has no force feedback or ROS state topic.

## 4. Calibrate

### Tool TCP

When tool length changes, update the driver TCP offset, `SurgicalToolTip`, and tool geometry in both `rm_65.urdf.xacro` and `rm_65_gazebo.urdf.xacro`, as well as payload values when needed. Values use meters. Rebuild `rm_driver`, `rm_description`, `rm_65_config`, and `rm_gazebo`, then verify the tip in RViz or Gazebo. TCP length is separate from the RCM distance. The current model assumes the tool extends along local `+Z`; other offsets require a full TCP calibration.

### Dual-Arm RCM Points

Start both real-arm RCM chains. Stop the web teleoperator and ensure no other controller is moving the arms. Build and run:

```bash
ros2 run rm_rcm_web_teleop dual_rcm_calibrate
```

Use `s left` or `s right` to enable hand-guiding. Align the tool tip with the trocar center and enter `r left` or `r right`; keep the tip at the point and enter `a left` or `a right` to validate and save. Tool-axis error must be below 2 mm. Use `x left` or `x right` to cancel a side, or `q` to exit.

Results are saved in meters in the MoveIt `world` frame to `data/calibration/dual_rcm_points.json`. The file must use the current `rm_dual_rcm_points_v2` format. Restart the dual-arm RCM launch to load and verify the points. See the [web teleoperation guide](src/ros2_rm_robot/rm_rcm_web_teleop/README.md) for details.

### Camera-to-World Extrinsics

For eye-to-hand calibration, start the camera and dual-arm RCM launch. Keep the camera and checkerboard fixed, then run:

```bash
ros2 run rm_rcm_web_teleop robot_checkerboard_calibrate \
  --image-topic /camera/camera/color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --base-frame world \
  --tool-frame SurgicalToolTip \
  --square-size-mm 20 \
  --output data/calibration/robot_checkerboard.json
```

Select the arm with `1` (left) or `2` (right). Click an inner checkerboard corner, gently touch it with the tool tip, and press `c` while the arm is still. Collect at least 8 well-distributed points per arm, then press `s` to fit. Review residuals and RMS error before using the result. The calibration is saved to `data/calibration/robot_checkerboard.json`.

## 5. Use the Robot Functions

### RCM Teleoperation

Start the RCM robot launch first, then start the web interface in another sourced terminal.

Single-arm simulation:

```bash
ros2 launch rm_rcm_web_teleop web_teleop.launch.py
```

Single-arm hardware:

```bash
ros2 launch rm_rcm_web_teleop web_teleop.launch.py use_sim_time:=false
```

Dual-arm hardware, with the camera and SCS215 configuration ready:

```bash
ros2 launch rm_rcm_web_teleop dual_web_teleop.launch.py
```

Open `http://127.0.0.1:8765`. The interface provides tilt, insertion/retraction, axial roll in dual-arm mode, gripper controls, and teaching recording. Dual-arm mode also streams RGB video and supports video recording. Use `host:=0.0.0.0` for LAN access only on a trusted network. Web stop buttons request software stops and do not replace the controller emergency stop. See the [teleoperation guide](src/ros2_rm_robot/rm_rcm_web_teleop/README.md) for step sizes, calibration, teaching, and playback.

### Codex Agent Mode

The repository-root `AGENTS.md` is the instruction file for Codex task execution. It defines the observation loop, action schema, task limits, and stop conditions. Agent mode uses online image servoing: Codex reads the latest RGB frame and robot state, sends one small normalized RCM action, waits for completion, and observes again. It does not use the classical `mis_perception` task pipeline or require camera-to-world extrinsics in the control loop; calibrated RCM points are still required.

Start the camera, both RCM chains, and the dual teleoperator first. Keep Agent control disabled while checking the system:

```bash
ros2 launch rm_rcm_web_teleop dual_web_teleop.launch.py \
  agent_enabled:=false
```

After checking the workspace, RCM points, camera view, gripper mapping, and physical emergency stop, explicitly enable the Agent interface:

```bash
ros2 launch rm_rcm_web_teleop dual_web_teleop.launch.py \
  agent_enabled:=true agent_max_action_hz:=2.0
```

In the same Codex session, issue: `Start the task and strictly follow the repository-root AGENTS.md.`

The Agent uses these local allowlisted endpoints:

```text
GET  http://127.0.0.1:8765/api/agent/observe
POST http://127.0.0.1:8765/api/agent/action
POST http://127.0.0.1:8765/api/stop
```

An action contains one arm and one small increment. Motion and gripper commands are sent separately:

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

Normalized motion values are in `[-1, 1]` and are rate-limited and checked by the local controller. Stop if the image, target, tool tip, RCM state, or action result is unclear. Software stop does not replace the physical emergency stop. Repository instructions are behavioral guidance, not an operating-system security boundary; use a restricted workspace or container when the Agent must be prevented from reading other files.

At task startup, the instrument tip may not be visible because insertion is insufficient. When the selected arm reports `rcm_initializing: true`, first use small axial insertion/retraction actions; tilt and roll are rejected during initialization. The current controller defines initialization as the tip being within 20 mm of the RCM point. This differs from the RCM calibration requirement of less than 2 mm tool-axis error.

### Dual-Arm Motion Examples

The basic dual-arm bringup is available separately:

```bash
ros2 launch rm_bringup rm_65_dual_bringup.launch.py
```

`rm_example` provides `rm_65_dual_movej` for joint targets and `rm_65_dual_pose` for Cartesian pose targets. Confirm target values, units, and collision clearance before sending motion commands. These commands issue separate arm goals and do not guarantee synchronized motion.

### Perception and Task Execution

Start the camera and dual-arm RCM launch first, then start perception, task execution, and the dashboard together:

```bash
ros2 launch mis_task_manager task_execution.launch.py
```

This defaults to `motion_enabled:=false` and `dry_run:=true`. For a real run, validate perception, calibration, frames, and workspace, then explicitly enable motion:

```bash
ros2 launch mis_task_manager task_execution.launch.py \
  motion_enabled:=true dry_run:=false \
  target_frame:=world \
  camera_extrinsics_file:=/absolute/path/to/FDU-Embodied-MIS/data/calibration/robot_checkerboard.json \
  camera_extrinsics_arm:=left \
  lesion_profile:=painted
```

Omit the extrinsics arguments when a valid camera-to-`world` TF is already available. Open the dashboard at `http://127.0.0.1:8780`; it reports system readiness and task status. The **Start** button runs one mission per process. The **Emergency Stop** button sends software stop commands but does not replace the hardware emergency stop.

The current task executor performs short RCM-constrained visual-servo steps toward object-relative waypoints. It does not implement tissue exposure, boundary-following dissection, grasping, or retrieval. Read the [task manager guide](src/ros2_rm_robot/mis_task_manager/README.md) and [perception guide](src/ros2_rm_robot/mis_perception/README.md) before enabling motion.

### Troubleshooting

- `Package 'rm_bringup' not found`: the current terminal may not have sourced the workspace, or the build may have failed. From the repository root, run `source /opt/ros/humble/setup.bash` and `source install/setup.bash`, then inspect the first `colcon build` error.
- The driver cannot connect to an arm: try `ping <arm_ip>`, then check `arm_ip`, the host network interface, `udp_ip`/`udp_port`, and the firewall. The IP settings are in YAML files, not only in launch arguments.
- `libapi_cpp.so` or `libRM_Service.so` is missing: verify that the library installation was run from `src/ros2_rm_robot/rm_driver/lib` and that `sudo ldconfig` completed. Check with `ldconfig -p | grep -E 'libapi_cpp|libRM_Service'`.
- The camera node starts but publishes no frames: check the USB 3 connection, `lsusb`/`rs-enumerate-devices`, and whether another program is using the camera. Try a lower resolution or frame rate.
- Perception publishes masks but no world coordinates: check `target_frame:=world`, the TF from the camera optical frame, or provide checkerboard extrinsics calibrated with `--base-frame world`.
- The task dashboard Start button is disabled: check that `/perception/scene` is fresh and contains a valid target, both arms are RCM-ready, and MoveIt has acknowledged the current obstacle revision. Start with the default `motion_enabled:=false dry_run:=true` mode.
- The web page opens but the grippers do not move: check `/dev/serial/by-id/`, the port/ID/baud rate/power in `scs215.yaml`, and that the SDK is installed for `/usr/bin/python3`. The grippers have no force feedback.

### Recommended Reproduction Order

1. Start the RM65 Gazebo RCM simulation and confirm that RViz, MoveIt, and web teleoperation work.
2. Start RealSense (or an equivalent recorded/replayed image source), run `mis_perception`, and inspect `/perception/overlay`, the masks, and `/perception/scene`.
3. Run `task_execution.launch.py` with the default dry-run settings to validate the state machine and dashboard; then check calibration, collision boxes, and RCM state.
4. Connect the real arms last. At low speed, check single-arm RCM, dual-arm RCM, camera extrinsics, and grippers. Keep the hardware emergency stop accessible before any real motion.

### Simulation and TCP Reference

For supported robot models and general Gazebo/MoveIt operation, see the [Gazebo documentation](src/ros2_rm_robot/rm_gazebo/README_CN.md). RM65 TCP model files and update steps are described in the calibration section above.
