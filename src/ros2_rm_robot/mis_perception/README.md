# MIS Perception

`d405_perception` subscribes to synchronized RGB, aligned depth, and `CameraInfo`,
then runs the material segmentation implementation included in this package. It
publishes one scene message, per-class binary masks, and a debug overlay. It does
not command the robot.

The segmentation implementation was brought into this package from the MIT-
licensed `d405_demo` project; its copyright and license notice are preserved in
`LICENSE`.

## Build and run

From the `FDU-Embodied-MIS` workspace, with ROS 2 Humble sourced:

```bash
ros2 launch realsense2_camera rs_launch.py enable_depth:=true align_depth.enable:=true
```

In another terminal:

```bash
colcon build --packages-up-to mis_perception
source install/setup.bash
ros2 launch mis_perception perception.launch.py
```

The default inputs are `/camera/camera/color/image_raw`,
`/camera/camera/aligned_depth_to_color/image_raw`, and
`/camera/camera/color/camera_info`. The depth image must be aligned to RGB. Use
launch arguments to override topics or the target frame:

```bash
ros2 launch mis_perception perception.launch.py \
  target_frame:=world \
  material_profile:=painted \
  lesion_profile:=material
```

`material_profile` selects the whole phantom mask set (`legacy` or `painted`).
`lesion_profile` selects the lesion detector independently: `material` uses the
lesion mask from the selected material profile; `painted`, `official`, `pink`,
`pink-real`, and `auto` run a standalone lesion detector. For example, to test
a pink practice target while retaining painted material masks:

```bash
ros2 launch mis_perception perception.launch.py \
  material_profile:=painted \
  lesion_profile:=pink-real
```

The updated `painted` profile separates liver and bowel using connected
component shape, which helps when bright exposure makes their red hues overlap.
Inspect `/perception/overlay` and `/perception/masks/lesion` on the actual
phantom before enabling robot motion; profile names do not guarantee correct
segmentation under different lighting or materials.

For the RM65-6FB MoveIt configuration in this workspace, `world` is the planning
frame used by the RCM controller. The camera optical frame must have a valid TF
transform to `target_frame` at the image timestamp. When this transform is
unavailable, masks and camera-frame positions are still published, while
`transform_valid` and each object's `world_position_valid` remain false unless
a matching camera extrinsics file is configured. Run the checkerboard
calibration with `--base-frame world`, then pass its output as
`camera_extrinsics_file`; the node uses TF when available and falls back to that
file only when its source and target frame IDs match. Keep `target_frame` equal
to the planning frame expected by the selected arm. Existing calibration files
tagged `base_link` do not match `world` and must be regenerated with
`--base-frame world`.

```bash
ros2 launch mis_perception perception.launch.py \
  target_frame:=world \
  camera_extrinsics_file:=/mnt/d/code/linux/FDU-Embodied-MIS/data/calibration/robot_checkerboard.json
```

For the right-arm transform, set `camera_extrinsics_arm:=right`. The dual-arm checkerboard calibration file stores transforms under `arms.left.matrix` and `arms.right.matrix`; the launch argument defaults to `left`.

## Outputs

- `/perception/scene` (`rm_ros_interfaces/PerceptionScene`): timestamped entity
  positions, depth validity, transform status, and frame ID.
- `/perception/masks/{liver,kidney,gallbladder,bowel,omentum,vessel,lesion}`
  (`sensor_msgs/Image`, `mono8`).
- `/perception/overlay` (`sensor_msgs/Image`, `bgr8`): visualization of masks.

An object's `position_camera` is the median of valid 3D points inside its mask;
`position_world` is populated only when the TF transform succeeds. `quality_score`
is a contour/depth quality heuristic, not a calibrated class probability. The
current segmenter is a color-rule phantom baseline, so its class masks must be
validated against the actual competition phantom and lighting. In particular,
some supplied material profiles may not reliably distinguish the kidney.

## Obstacle geometry

For every detected non-liver, non-lesion class, the node computes a world-frame
AABB from the 5th/95th percentiles of stable mask depth points, expands it by
`obstacle_margin_m` (default 15 mm), and enforces a minimum extent of
`obstacle_min_extent_m` (default 40 mm). The liver is the target surface for the
current lesion task, so it is intentionally not inserted as a collision box.

`PerceptionScene.obstacles_valid` is false when a detected obstacle has too few
valid depth points or lacks a world transform. The task executor then refuses
motion. `obstacle_geometry_revision` changes when an obstacle appears,
disappears, or its bounds move by at least 5 mm; each arm's MoveIt node reports
the revision it has applied. Masks and AABBs are single-view estimates: inspect
them with the phantom in its actual pose, camera angle, and lighting before
using real motion.

After changing the interface or MoveIt package, rebuild the dependent packages:

```bash
colcon build --packages-up-to rm_moveit2 mis_perception mis_task_manager
source install/setup.bash
```
