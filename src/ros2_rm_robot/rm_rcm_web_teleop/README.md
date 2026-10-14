# RM65 RCM Web Teleoperation

Browser-based controls for a running `rm65_rcm_demo`. This package does not start the robot, MoveIt, or Gazebo. The RCM demo handles planning, constraint checks, and execution.

## Single-Arm Setup

Start the RCM demo first. For simulation:

```bash
ros2 launch rm_bringup rm_65_6fb_rcm_gazebo.launch.py
ros2 launch rm_rcm_web_teleop web_teleop.launch.py
```

Open <http://127.0.0.1:8765>. Hold and move the joystick to tilt the tool in 2-degree steps. Hold **Insert** or **Retract** for 2 mm axial steps. A new step is sent after the current motion completes. **Stop** requests that MoveIt stop its current trajectory.

The server defaults to `127.0.0.1:8765` and simulation time. For hardware, start and verify the real-robot RCM stack, then run:

```bash
ros2 launch rm_rcm_web_teleop web_teleop.launch.py use_sim_time:=false
```

For access from another computer, set `host:=0.0.0.0` and browse to the robot host's LAN address. Only expose the server on a trusted network. The stop button is not an emergency stop.

Real-robot RCM must use a calibrated fixed point in the MoveIt planning frame (`world`):

```bash
ros2 launch rm_bringup rm_65_6fb_rcm_real.launch.py \
  derive_rcm_from_current:=false rcm_x:=<x_m> rcm_y:=<y_m> rcm_z:=<z_m> \
  velocity_scaling:=0.02 acceleration_scaling:=0.02 \
  max_rcm_rotation_deg:=4.0 max_rcm_insertion_m:=0.010
```

Replace the coordinates with measured calibration values. Verify feedback and small motions at low speed in a clear workspace.

## Dual-Arm Setup

The dual-arm launch starts both real-robot RCM chains. It loads calibrated points from `data/calibration/dual_rcm_points.json` when available; otherwise, each point is derived from the initial tool axis using `rcm_distance_from_tip_m`.

```bash
ros2 launch rm_bringup rm_65_dual_6fb_rcm_real.launch.py
ros2 launch rm_rcm_web_teleop dual_web_teleop.launch.py
```

Open <http://127.0.0.1:8765>. The page provides left and right controls, a camera view, axial roll, insertion/retraction, and fine/large step modes. Fine steps are 2 degrees for tilt and roll and 5 mm for insertion; large steps are twice those values. The default camera topic is `/camera/camera/color/image_raw` and can be changed with `camera_topic:=...`.

Camera streaming requires `cv_bridge` and `sensor_msgs`. The two arms plan independently; there is no dual-arm collision checking. Keep their workspaces clear and separated, and avoid commands from other controllers while teleoperating.

The optional SCS215 grippers are controlled directly over USB using `config/scs215.yaml` for port, servo ID, and open/close positions. Install the SDK with `/usr/bin/python3 -m pip install --user ftservo-python-sdk`. Check port assignments and clear the grippers before use. Gripper control is position-based and has no ROS state feedback.

## RCM Calibration

Build and source the workspace, then start the dual-arm drivers, MoveIt, and RCM nodes. Stop the web teleoperator and other motion controllers before calibration:

```bash
colcon build --packages-select rm_rcm_web_teleop rm_bringup
source install/setup.bash
ros2 run rm_rcm_web_teleop dual_rcm_calibrate
```

Use `s left` or `s right` to enable hand-guiding, align that tool tip with the trocar center, then enter `r left` or `r right` to record the point. Keep the tip at the target and enter `a left` or `a right` to validate and save. The tool-axis error must be under 2 mm. Enter `x left` or `x right` to cancel without changing that point, or `q` to exit.

Points are saved in meters in the `world` frame to `data/calibration/dual_rcm_points.json`. Restart the dual-arm launch to verify that it loads the saved points. Keep the workspace clear and the hardware emergency stop within reach.

## Teaching and Playback

Use **Start Recording** and **Stop and Save** in the dual-arm page to save accepted target poses, current joint states, and timing to `data/teachings/`. This does not record video or raw joystick input. The camera's separate **Record** button downloads the RGB view as a timestamped `.webm` file; it requires browser support for `MediaRecorder` and Canvas `captureStream`.

Before playback, place both arms in the same safe starting state used for recording, clear the workspace, and keep the hardware emergency stop ready:

```bash
ros2 run rm_rcm_web_teleop rcm_teaching_playback --ros-args \
  -p file:=/path/to/data/teachings/dual_rcm_<timestamp>.json
```

Playback sends one target at a time and waits for completion. `speed` changes only the delays between actions; it does not change robot motion speed. Available parameters include `speed` (default `1.0`), `busy_timeout_sec` (default `30.0`), `start_pose_timeout_sec` (default `120.0`), and `pause_limit_sec` (default `10.0`). Playback still uses the RCM planner and checks. The web stop button is a software request, not an emergency stop.

Teaching files use format `rm_rcm_teaching_v2`. They must contain `start_joints`; playback first restores both arms using the recorded joint angles, preserving the original inverse-kinematics branch. The recorded `start_poses` remain as reference data. Files from the retired `v1` pose-only format are rejected and must be recorded again. All recorded actions after initialization still use the RCM planner and trajectory validation. Keep the workspace clear and the hardware emergency stop ready.
