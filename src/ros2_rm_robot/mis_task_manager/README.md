# MIS Task Manager

`mis_task_manager` is a single-start ROS 2 mission state machine for the three
competition tasks. It waits for a fresh valid perception scene, then advances
through task 1 planning, task 2 approach and exposure, and task 3 dissection and
retrieval. Each stage publishes progress on `/task/status`.

## Run

Build the interface and manager packages from the workspace root:

```bash
colcon build --packages-up-to mis_task_manager
source install/setup.bash
```

For standalone debugging, perception and the state machine can be started
separately:

```bash
ros2 launch mis_perception perception.launch.py
ros2 launch mis_task_manager task_state_machine.launch.py dry_run:=true
```

Send one start signal after both nodes are ready:

```bash
ros2 topic pub --once /task/start std_msgs/msg/Bool "{data: true}"
```

## Standalone Execution Dashboard

The executor, state machine, and dashboard can be started together:

```bash
ros2 launch mis_task_manager task_execution.launch.py
```

The combined launch defaults to `motion_enabled:=false` and `dry_run:=true`.
For real execution, explicitly enable motion and disable dry-run after checking
the robot, calibration, perception, and workspace:

```bash
ros2 launch mis_task_manager task_execution.launch.py motion_enabled:=true dry_run:=false
```

The three component launches remain available for debugging or standalone use.
The combined launch starts perception, task execution, and monitoring, so do not
start `mis_perception perception.launch.py` separately when using
`task_execution.launch.py`. Start the RealSense camera, robot drivers, and
MoveIt separately first. Pass `camera_extrinsics_file` and
`camera_extrinsics_arm:=left` or `right` to the combined launch when perception
must use the checkerboard calibration instead of a camera-to-world TF.

The dashboard has its own page and HTTP server and does not launch or use the Web
teleoperation interface.

Open `http://127.0.0.1:8780`. The page shows the RGB stream with the perception
node's colored segmentation masks overlaid, task phase,
perception readiness, both arm/RCM states, and read-only SCS215 communication
checks. The start button is enabled only when the state machine, executor,
camera, target scene, and both arms report ready. Its emergency stop button
cancels the task and sends RCM stop, driver motion stop, and controller
emergency-stop commands to both arms. The stop is latched until the dashboard
process is restarted; verify both controllers before restarting it. This
software command does not replace the physical emergency stop.

The displayed image comes from `/perception/overlay`; the source RGB topic is
`/camera/camera/color/image_raw`. The combined launch accepts `color_topic`,
`depth_topic`, `camera_info_topic`, `target_frame`, `camera_extrinsics_file`,
`camera_extrinsics_arm`, and `material_profile` to configure perception. Use
`lesion_profile:=pink-real`, `pink`, `official`, `painted`, or `auto` to
override the lesion mask independently; the default `material` keeps the
selected material profile's integrated lesion result.

Each RM65 MoveIt instance consumes the current `PerceptionScene`, inserts
valid non-liver/non-lesion AABBs into its own namespaced planning scene, and
acknowledges the applied obstacle geometry revision. The dashboard requires
both arms to acknowledge the current revision before enabling task start. The
executor waits for the moving arm to acknowledge each current revision and
stops if perception changes the revision during motion. It refuses motion when
a detected obstacle lacks valid depth geometry. The RCM Cartesian path uses
MoveIt's collision checking; it can reject an intersecting path, but it does
not search for an alternate route. The liver is excluded because it is the
lesion target surface. Validate box sizes and collision behavior with motion
disabled before real execution.

The HTTP server binds to `127.0.0.1` by default. To view the page from another
computer, set `host:=0.0.0.0` and `port:=8780`, then open the ROS computer's
address on the same trusted network. The SCS215 status reads the existing
`rm_rcm_web_teleop/config/scs215.yaml` settings and only pings the servos; the
teleoperation server is not started and no servo position commands are sent.

In dry-run mode, the manager still requires a fresh valid scene with at least
one detected object position in the configured target frame. It simulates each
stage and reports the progression without commanding either arm. Cancel an
active run with `/task/cancel` (`std_msgs/Bool`, `data: true`).

## Executor Interface

The package includes an RCM waypoint executor for the existing dual-arm RCM
demo nodes. Start perception and the RM65 RCM MoveIt nodes first, then start
the executor and state machine:

```bash
ros2 launch mis_task_manager rcm_task_executor.launch.py motion_enabled:=true
ros2 launch mis_task_manager task_state_machine.launch.py dry_run:=false
```

`motion_enabled` defaults to `false`; in that mode the executor rejects every
motion step. With motion enabled, it subscribes to the fresh perception scene,
filters the last three 3D target samples, and sends one RCM-constrained waypoint
at a time to `/<arm>_arm/rcm_demo/web_target`. After each waypoint it reads the
new target estimate and recomputes the next one. Defaults limit each update to
8 mm of insertion/retraction or 2 degrees of RCM rotation. The real dual-arm
MoveIt launch also defaults to 0.02 velocity and acceleration scaling and caps
RCM motion at 10 mm insertion per command. Target and arm state frame IDs must
match.

The executor monitors `/perception/scene` during motion. A stale scene, lost or
low-quality target, invalid transform, cancellation, or timeout aborts the
visual-servo step and sends `stop`. Useful tuning arguments include
`scene_max_age_sec`, `minimum_target_quality`, `servo_max_radial_step_m`,
`servo_max_rotation_deg`, `servo_position_tolerance_m`,
`servo_max_iterations`, and `servo_timeout_sec`.

This is receding-horizon position-based visual servoing: MoveIt plans and
executes short RCM-constrained Cartesian motions, then the executor closes the
loop with the next RGB-D target estimate. It does not yet implement full
collision-aware task planning, tissue exposure, boundary-following dissection,
grasping, or lesion retrieval. The named mission stages currently servo toward
object-relative waypoints; they must not be treated as completed surgical
actions. Verify the perception frame, RCM calibration, MoveIt planning frame,
and workspace with the robot in a controlled setup before enabling motion. A
failed step, timeout, cancel request, or invalid/stale scene aborts the mission.
The manager accepts only one start signal per process.
