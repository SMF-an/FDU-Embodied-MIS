# FDU-Embodied-MIS

ROS 2 Humble workspace for RealMan RM65-6FB arms, RealSense cameras, RCM-constrained motion, and embodied MIS task experiments. The project is developed on Ubuntu 22.04.

## 1. Get the Source

```bash
git clone https://github.com/SMF-an/FDU-Embodied-MIS.git
cd FDU-Embodied-MIS
```

## 2. Configure the Environment

Install ROS 2, MoveIt 2, and RealMan dependencies from the workspace root:

```bash
sudo bash src/ros2_rm_robot/rm_install/scripts/ros2_install.sh
sudo bash src/ros2_rm_robot/rm_install/scripts/moveit2_install.sh
sudo bash src/ros2_rm_robot/rm_driver/lib/lib_install.sh
sudo apt update
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
```

Build the ROS interfaces first, then the workspace:

```bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
```

In every new terminal, source the workspace:

```bash
cd /path/to/FDU-Embodied-MIS
source install/setup.bash
```

Rebuild after source changes with `colcon build`, or build a specific package with `colcon build --packages-select <package>`. For WSL2 hardware networking, follow [robot network setup](docs/机械臂与电脑网络通信配置说明.md) and [RealSense USB setup](docs/深度相机与电脑通信配置说明.md).

## 3. Start the Hardware

Start each component in its own sourced terminal. Check robot IP addresses in `src/ros2_rm_robot/rm_driver/config/` before connecting.

### Robot Drivers

For a single RM65-6FB real arm with MoveIt and RViz:

```bash
ros2 launch rm_bringup rm_65_6fb_bringup.launch.py
```

For single-arm RCM teleoperation, use the RCM launch and supply a calibrated fixed point in the MoveIt `world` frame:

```bash
ros2 launch rm_bringup rm_65_6fb_rcm_real.launch.py \
  derive_rcm_from_current:=false rcm_x:=<x_m> rcm_y:=<y_m> rcm_z:=<z_m>
```

For both arms with drivers, MoveIt, and RCM nodes:

```bash
ros2 launch rm_bringup rm_65_dual_6fb_rcm_real.launch.py
```

The dual-arm RCM launch loads `world`-frame calibration from `data/calibration/dual_rcm_points.json` when available. Without a valid file, it derives each point from the initial tool axis; this is a software setup value, not a measured trocar location.

For simulation, see [Gazebo and MoveIt instructions](src/ros2_rm_robot/rm_gazebo/README_CN.md). The RM65 RCM simulation entry point is:

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

The dual-arm teleop and task execution use `/camera/camera/color/image_raw` by default. Change the camera profiles or topic arguments to match the device. For WSL2 USB passthrough and tested low-bandwidth profiles, see the [camera setup guide](docs/深度相机与电脑通信配置说明.md).

### SCS215 Grippers

The dual-arm web teleoperator controls SCS215 servos directly over USB; it does not start a servo ROS node or publish servo topics. Install the SDK and build the teleop package:

```bash
/usr/bin/python3 -m pip install --user ftservo-python-sdk
```

Set each port, servo ID, baud rate, and open/close position in `src/ros2_rm_robot/rm_rcm_web_teleop/config/scs215.yaml`. Verify port mapping and clear the gripper area before enabling torque. Control is position-based and has no force feedback or ROS state topic.

## 4. Calibrate

### Tool TCP

When tool length changes, update the driver TCP offset, `SurgicalToolTip` and tool geometry in both `rm_65.urdf.xacro` and `rm_65_gazebo.urdf.xacro`, and payload values when needed. Values use meters. Rebuild `rm_driver`, `rm_description`, `rm_65_config`, and `rm_gazebo`, then verify the tip in RViz or Gazebo. TCP length is separate from the RCM distance. The current model assumes the tool extends along local `+Z`; other offsets require a full TCP calibration.

### Dual-Arm RCM Points

Start both real-arm RCM chains, stop the web teleoperator, and ensure no other controller is moving the arms. Build and run:

```bash
ros2 run rm_rcm_web_teleop dual_rcm_calibrate
```

Use `s left` or `s right` to enable hand-guiding. Align the tool tip with the trocar center and enter `r left` or `r right`; keep the tip at the point and enter `a left` or `a right` to validate and save. The tool-axis error must be below 2 mm. Use `x left|right` to cancel a side, or `q` to exit.

Results are saved in meters in the MoveIt `world` frame to `data/calibration/dual_rcm_points.json`. The file must use the current `rm_dual_rcm_points_v2` format. Restart the dual-arm RCM launch to load and verify the points. Full details are in the [web teleop guide](src/ros2_rm_robot/rm_rcm_web_teleop/README.md).

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

Select the arm with `1` (left) or `2` (right). Click an inner checkerboard corner, touch it gently with the tool tip, and press `c` while the arm is still. Collect at least 8 well-distributed points per arm, then press `s` to fit. Review the residuals and RMS error in the output before using the calibration. The result is saved to `data/calibration/robot_checkerboard.json`.

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

Open `http://127.0.0.1:8765`. The interface provides tilt, insertion/retraction, and (dual-arm) axial roll, gripper controls, and teaching recording. Dual-arm mode also streams RGB video and supports video recording. Use `host:=0.0.0.0` for LAN access only on a trusted network. Web stop buttons request software stops and do not replace the controller emergency stop. See the [teleop guide](src/ros2_rm_robot/rm_rcm_web_teleop/README.md) for step sizes, calibration, teaching, and playback.

### Dual-Arm Motion Examples

The basic dual-arm bringup is available separately:

```bash
ros2 launch rm_bringup rm_65_dual_bringup.launch.py
```

`rm_example` provides `rm_65_dual_movej` for joint targets and `rm_65_dual_pose` for Cartesian pose targets. Confirm target values, units, and collision clearance before sending motion commands. These commands issue separate arm goals and do not guarantee synchronized motion.

### Perception and Task Execution

Start the camera and dual-arm RCM launch first. Then start perception, task execution, and the dashboard together:

```bash
ros2 launch mis_task_manager task_execution.launch.py
```

This defaults to `motion_enabled:=false` and `dry_run:=true`. For a real run, validate perception, calibration, frames, and workspace, then explicitly enable motion:

```bash
ros2 launch mis_task_manager task_execution.launch.py \
  motion_enabled:=true dry_run:=false \
  target_frame:=world \
  camera_extrinsics_file:=/path/to/data/calibration/robot_checkerboard.json \
  camera_extrinsics_arm:=left \
  lesion_profile:=pink-real
```

Omit the extrinsics arguments when a valid camera-to-`world` TF is already available. Open the dashboard at `http://127.0.0.1:8780`; it reports system readiness and task status. The **Start** button runs one mission per process. The **Emergency Stop** button sends software stop commands but does not replace the hardware emergency stop.

The current task executor performs short RCM-constrained visual-servo steps toward object-relative waypoints. It does not implement tissue exposure, boundary-following dissection, grasping, or retrieval. Inspect the [task manager guide](src/ros2_rm_robot/mis_task_manager/README.md) and [perception guide](src/ros2_rm_robot/mis_perception/README.md) before enabling motion.

### Simulation and TCP Reference

For supported robot models and general Gazebo/MoveIt operation, see [rm_gazebo documentation](src/ros2_rm_robot/rm_gazebo/README_CN.md). For RM65 TCP model files and update steps, see the calibration section above.
