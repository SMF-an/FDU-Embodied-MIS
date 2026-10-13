from __future__ import annotations

import asyncio
import math
import threading
import time
from collections import deque
from statistics import median

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rm_ros_interfaces.action import TaskStep
from rm_ros_interfaces.msg import PerceptionScene
from std_msgs.msg import Bool
from std_msgs.msg import UInt64


def normalize_quaternion(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-12:
        raise ValueError("invalid zero quaternion")
    return tuple(value / norm for value in q)


def normalize_vector(vector):
    norm = math.sqrt(sum(value * value for value in vector))
    if norm < 1e-12:
        raise ValueError("cannot normalize a zero vector")
    return tuple(value / norm for value in vector)


def multiply_quaternions(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q, vector):
    x, y, z, w = q
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def align_tool_axis(current_q, desired_axis):
    current_axis = rotate_vector(current_q, (0.0, 0.0, 1.0))
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(current_axis, desired_axis))))
    cross = (
        current_axis[1] * desired_axis[2] - current_axis[2] * desired_axis[1],
        current_axis[2] * desired_axis[0] - current_axis[0] * desired_axis[2],
        current_axis[0] * desired_axis[1] - current_axis[1] * desired_axis[0],
    )
    if dot < -0.999999:
        helper = (1.0, 0.0, 0.0) if abs(current_axis[0]) < 0.9 else (0.0, 1.0, 0.0)
        axis = (
            current_axis[1] * helper[2] - current_axis[2] * helper[1],
            current_axis[2] * helper[0] - current_axis[0] * helper[2],
            current_axis[0] * helper[1] - current_axis[1] * helper[0],
        )
        axis_norm = math.sqrt(sum(value * value for value in axis))
        delta = tuple(value / axis_norm for value in axis) + (0.0,)
    else:
        delta = normalize_quaternion((*cross, 1.0 + dot))
    return normalize_quaternion(multiply_quaternions(delta, current_q))


class RcmTaskExecutor(Node):
    ARMS = ("left", "right")

    def __init__(self) -> None:
        super().__init__("rcm_task_executor")
        self.declare_parameter("motion_enabled", False)
        self.declare_parameter("scene_topic", "/perception/scene")
        self.declare_parameter("scene_max_age_sec", 0.5)
        self.declare_parameter("minimum_target_quality", 0.05)
        self.declare_parameter("state_timeout_sec", 2.0)
        self.declare_parameter("motion_start_timeout_sec", 3.0)
        self.declare_parameter("motion_timeout_sec", 60.0)
        self.declare_parameter("position_tolerance_m", 0.003)
        self.declare_parameter("servo_position_tolerance_m", 0.01)
        self.declare_parameter("servo_angle_tolerance_deg", 2.0)
        self.declare_parameter("servo_max_radial_step_m", 0.008)
        self.declare_parameter("servo_max_rotation_deg", 2.0)
        self.declare_parameter("servo_max_iterations", 30)
        self.declare_parameter("servo_timeout_sec", 110.0)
        self.declare_parameter("minimum_rcm_radius_m", 0.02)
        self.declare_parameter("maximum_rcm_radius_m", 1.0)
        self._motion_enabled = bool(self.get_parameter("motion_enabled").value)
        self._scene_topic = str(self.get_parameter("scene_topic").value)
        self._scene_max_age = float(self.get_parameter("scene_max_age_sec").value)
        self._minimum_target_quality = float(
            self.get_parameter("minimum_target_quality").value
        )
        self._state_timeout = float(self.get_parameter("state_timeout_sec").value)
        self._start_timeout = float(self.get_parameter("motion_start_timeout_sec").value)
        self._motion_timeout = float(self.get_parameter("motion_timeout_sec").value)
        self._position_tolerance = float(self.get_parameter("position_tolerance_m").value)
        self._servo_position_tolerance = float(
            self.get_parameter("servo_position_tolerance_m").value
        )
        self._servo_angle_tolerance = math.radians(
            float(self.get_parameter("servo_angle_tolerance_deg").value)
        )
        self._servo_max_radial_step = float(
            self.get_parameter("servo_max_radial_step_m").value
        )
        self._servo_max_rotation = math.radians(
            float(self.get_parameter("servo_max_rotation_deg").value)
        )
        self._servo_max_iterations = int(
            self.get_parameter("servo_max_iterations").value
        )
        self._servo_timeout = float(self.get_parameter("servo_timeout_sec").value)
        self._minimum_radius = float(self.get_parameter("minimum_rcm_radius_m").value)
        self._maximum_radius = float(self.get_parameter("maximum_rcm_radius_m").value)
        if (
            self._scene_max_age <= 0.0
            or not 0.0 <= self._minimum_target_quality <= 1.0
            or self._position_tolerance <= 0.0
            or self._servo_position_tolerance <= 0.0
            or self._servo_angle_tolerance <= 0.0
            or self._servo_max_radial_step <= 0.0
            or self._servo_max_rotation <= 0.0
            or self._servo_max_iterations <= 0
            or self._servo_timeout <= 0.0
        ):
            raise ValueError("visual servo parameters are outside their valid ranges")

        self._lock = threading.Lock()
        self._active_goal = False
        self._latest_scene = None
        self._latest_scene_at = 0.0
        self._target_history = {}
        self._state = {
            arm: {
                "pose": None,
                "pose_at": 0.0,
                "rcm": None,
                "rcm_at": 0.0,
                "busy": False,
                "busy_at": 0.0,
                "collision_scene_revision": None,
                "collision_scene_at": 0.0,
            }
            for arm in self.ARMS
        }
        busy_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._target_pubs = {}
        self._stop_pubs = {}
        for arm in self.ARMS:
            root = f"/{arm}_arm/rcm_demo"
            self._target_pubs[arm] = self.create_publisher(
                PoseStamped, f"{root}/web_target", 10
            )
            self._stop_pubs[arm] = self.create_publisher(Bool, f"{root}/stop", 10)
            self.create_subscription(
                PoseStamped,
                f"{root}/current_tool_pose",
                lambda msg, side=arm: self._on_pose(side, msg),
                10,
            )
            self.create_subscription(
                PointStamped,
                f"{root}/rcm_point",
                lambda msg, side=arm: self._on_rcm(side, msg),
                10,
            )
            self.create_subscription(
                Bool,
                f"{root}/motion_busy",
                lambda msg, side=arm: self._on_busy(side, msg),
                busy_qos,
            )
            self.create_subscription(
                UInt64,
                f"{root}/collision_scene_synced",
                lambda msg, side=arm: self._on_collision_scene_synced(side, msg),
                busy_qos,
            )
        self.create_subscription(
            PerceptionScene, self._scene_topic, self._on_scene, 10
        )

        self._action_server = ActionServer(
            self,
            TaskStep,
            "/task_executor/step",
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )
        self.get_logger().info(
            f"RCM visual-servo executor ready; motion_enabled={self._motion_enabled}, "
            f"scene_topic={self._scene_topic}"
        )

    def _on_pose(self, arm: str, message: PoseStamped) -> None:
        with self._lock:
            self._state[arm]["pose"] = message
            self._state[arm]["pose_at"] = time.monotonic()

    def _on_rcm(self, arm: str, message: PointStamped) -> None:
        with self._lock:
            self._state[arm]["rcm"] = message
            self._state[arm]["rcm_at"] = time.monotonic()

    def _on_busy(self, arm: str, message: Bool) -> None:
        with self._lock:
            self._state[arm]["busy"] = bool(message.data)
            self._state[arm]["busy_at"] = time.monotonic()

    def _on_collision_scene_synced(self, arm: str, message: UInt64) -> None:
        with self._lock:
            self._state[arm]["collision_scene_revision"] = int(message.data)
            self._state[arm]["collision_scene_at"] = time.monotonic()

    def _on_scene(self, message: PerceptionScene) -> None:
        with self._lock:
            self._latest_scene = message
            self._latest_scene_at = time.monotonic()
            seen = set()
            for item in message.objects:
                if not (
                    item.detected
                    and item.position_valid
                    and item.world_position_valid
                ):
                    continue
                position = (
                    item.position_world.x,
                    item.position_world.y,
                    item.position_world.z,
                )
                if not all(math.isfinite(value) for value in position):
                    continue
                seen.add(item.class_name)
                history = self._target_history.setdefault(
                    item.class_name, deque(maxlen=3)
                )
                history.append(
                    (
                        *position,
                    )
                )
            for class_name in tuple(self._target_history):
                if class_name not in seen:
                    self._target_history[class_name].clear()

    def _accept_goal(self, request: TaskStep.Goal) -> GoalResponse:
        if request.step != "identify_and_plan" and not self._motion_enabled:
            self.get_logger().error(
                "Rejected motion step: set motion_enabled:=true on the executor explicitly"
            )
            return GoalResponse.REJECT
        with self._lock:
            if self._active_goal:
                return GoalResponse.REJECT
            self._active_goal = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        result = TaskStep.Result()
        request = goal_handle.request
        error = self._validate_scene(request.scene)
        if error:
            result.success = False
            result.message = error
            goal_handle.abort()
            self._release_goal()
            return result

        target = self._find_object(request.scene, request.target_class)
        if target is None:
            result.success = False
            result.message = f"target class not detected: {request.target_class}"
            goal_handle.abort()
            self._release_goal()
            return result

        if request.step == "identify_and_plan":
            result.success = True
            result.message = "scene and lesion position validated"
            goal_handle.succeed()
            self._release_goal()
            return result

        if not request.arms:
            result.success = False
            result.message = f"no arms assigned to step {request.step}"
            goal_handle.abort()
            self._release_goal()
            return result

        try:
            for index, arm in enumerate(request.arms):
                if arm not in self.ARMS:
                    raise ValueError(f"unsupported arm name: {arm}")
                if goal_handle.is_cancel_requested:
                    self._stop_all()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "motion step cancelled"
                    self._release_goal()
                    return result
                success, message = await self._move_arm_visual_servo(
                    goal_handle,
                    arm,
                    request.target_class,
                    float(request.approach_offset_m),
                    request.step,
                    index,
                    len(request.arms),
                )
                if not success:
                    raise RuntimeError(f"{arm} arm: {message}")

            result.success = True
            result.message = f"{request.step}: RCM-constrained waypoint reached"
            goal_handle.succeed()
        except Exception as exc:
            result.success = False
            result.message = str(exc)
            if goal_handle.is_cancel_requested:
                self._stop_all()
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            self._release_goal()
        return result

    def _validate_scene(self, scene: PerceptionScene) -> str:
        if not scene.valid or not scene.transform_valid:
            return f"perception scene is invalid: {scene.status}"
        if not scene.header.frame_id:
            return "perception scene frame_id is empty"
        if not scene.obstacles_valid:
            return f"obstacle geometry is invalid: {scene.obstacles_status}"
        return ""

    @staticmethod
    def _find_object(scene: PerceptionScene, class_name: str):
        return next(
            (
                item
                for item in scene.objects
                if item.class_name == class_name
                and item.detected
                and item.position_valid
                and item.world_position_valid
            ),
            None,
        )

    def _snapshot_arm(self, arm: str):
        with self._lock:
            return dict(self._state[arm])

    def _fresh_target(self, class_name: str):
        with self._lock:
            scene = self._latest_scene
            scene_age = time.monotonic() - self._latest_scene_at
            history = tuple(self._target_history.get(class_name, ()))
        if scene is None or scene_age > self._scene_max_age:
            return None, None, "perception scene is stale or unavailable"
        error = self._validate_scene(scene)
        if error:
            return None, None, error
        target = self._find_object(scene, class_name)
        if target is None:
            return None, None, f"target class lost: {class_name}"
        if (
            not math.isfinite(target.quality_score)
            or target.quality_score < self._minimum_target_quality
        ):
            return None, None, (
                f"target quality {target.quality_score:.3f} is below "
                f"minimum {self._minimum_target_quality:.3f}"
            )
        if not history:
            return None, None, f"no recent 3D samples for target class: {class_name}"
        filtered_position = tuple(
            median(sample[axis] for sample in history) for axis in range(3)
        )
        return scene, (target, filtered_position), ""

    def _make_servo_target(
        self, arm_state, scene, target_position, approach_offset
    ):
        pose = arm_state["pose"]
        rcm = arm_state["rcm"]
        expected_frame = scene.header.frame_id
        if pose.header.frame_id != expected_frame or rcm.header.frame_id != expected_frame:
            raise ValueError(
                f"frame mismatch: scene={expected_frame}, pose={pose.header.frame_id}, "
                f"rcm={rcm.header.frame_id}"
            )
        if not math.isfinite(approach_offset) or approach_offset < 0.0:
            raise ValueError("approach_offset_m must be finite and nonnegative")

        pivot = (rcm.point.x, rcm.point.y, rcm.point.z)
        point = target_position
        if not all(math.isfinite(value) for value in (*pivot, *point)):
            raise ValueError("target or RCM point contains a non-finite coordinate")
        radial = tuple(point[i] - pivot[i] for i in range(3))
        distance = math.sqrt(sum(value * value for value in radial))
        if (
            not math.isfinite(distance)
            or distance <= self._minimum_radius
            or distance > self._maximum_radius
        ):
            raise ValueError(
                f"target radius {distance:.3f} m is outside RCM limits "
                f"({self._minimum_radius:.3f}, {self._maximum_radius:.3f}] m"
            )
        if approach_offset >= distance - self._minimum_radius:
            raise ValueError("approach offset would place the tip at or inside the RCM point")

        desired_axis = tuple(value / distance for value in radial)
        desired_radius = distance - approach_offset
        tip = pose.pose.position
        current_radial = (tip.x - pivot[0], tip.y - pivot[1], tip.z - pivot[2])
        current_radius = math.sqrt(sum(value * value for value in current_radial))
        if (
            not math.isfinite(current_radius)
            or current_radius > self._maximum_radius
        ):
            raise ValueError("current tool radius is outside the configured RCM limits")
        current_orientation = normalize_quaternion((
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ))
        current_axis = rotate_vector(current_orientation, (0.0, 0.0, 1.0))
        dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(current_axis, desired_axis))))
        angular_error = math.acos(dot)
        initializing = current_radius < self._minimum_radius
        next_axis = (
            current_axis
            if initializing
            else self._rotate_toward(current_axis, desired_axis, self._servo_max_rotation)
        )
        next_radius = current_radius + max(
            -self._servo_max_radial_step,
            min(self._servo_max_radial_step, desired_radius - current_radius),
        )
        waypoint_position = tuple(
            pivot[i] + next_radius * next_axis[i] for i in range(3)
        )
        final_position = tuple(
            point[i] - approach_offset * desired_axis[i] for i in range(3)
        )
        target_orientation = align_tool_axis(current_orientation, next_axis)
        target = PoseStamped()
        target.header.frame_id = expected_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = waypoint_position
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = target_orientation
        return target, final_position, desired_axis, angular_error

    @staticmethod
    def _rotate_toward(current_axis, desired_axis, maximum_angle):
        dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(current_axis, desired_axis))))
        angle = math.acos(dot)
        if angle <= maximum_angle or angle < 1e-9:
            return desired_axis
        cross = (
            current_axis[1] * desired_axis[2] - current_axis[2] * desired_axis[1],
            current_axis[2] * desired_axis[0] - current_axis[0] * desired_axis[2],
            current_axis[0] * desired_axis[1] - current_axis[1] * desired_axis[0],
        )
        cross_norm = math.sqrt(sum(value * value for value in cross))
        if cross_norm < 1e-9:
            helper = (1.0, 0.0, 0.0) if abs(current_axis[0]) < 0.9 else (0.0, 1.0, 0.0)
            cross = (
                current_axis[1] * helper[2] - current_axis[2] * helper[1],
                current_axis[2] * helper[0] - current_axis[0] * helper[2],
                current_axis[0] * helper[1] - current_axis[1] * helper[0],
            )
            cross_norm = math.sqrt(sum(value * value for value in cross))
        axis = tuple(value / cross_norm for value in cross)
        angle = min(angle, maximum_angle)
        cosine, sine = math.cos(angle), math.sin(angle)
        axis_cross_current = (
            axis[1] * current_axis[2] - axis[2] * current_axis[1],
            axis[2] * current_axis[0] - axis[0] * current_axis[2],
            axis[0] * current_axis[1] - axis[1] * current_axis[0],
        )
        axis_dot_current = sum(a * b for a, b in zip(axis, current_axis))
        return normalize_vector(
            tuple(
                current_axis[i] * cosine
                + axis_cross_current[i] * sine
                + axis[i] * axis_dot_current * (1.0 - cosine)
                for i in range(3)
            )
        )

    async def _move_arm_visual_servo(
        self, goal_handle, arm, target_class, approach_offset, step_name, arm_index, arm_count
    ):
        overall_deadline = time.monotonic() + self._servo_timeout
        for iteration in range(self._servo_max_iterations + 1):
            if goal_handle.is_cancel_requested:
                self._stop_arm(arm)
                return False, "cancelled during visual servo"
            if time.monotonic() >= overall_deadline:
                self._stop_arm(arm)
                return False, "visual servo timed out"

            wait_deadline = min(overall_deadline, time.monotonic() + self._state_timeout)
            arm_state = self._snapshot_arm(arm)
            while (
                arm_state["pose"] is None
                or arm_state["rcm"] is None
                or time.monotonic() - arm_state["pose_at"] > self._state_timeout
                or time.monotonic() - arm_state["rcm_at"] > self._state_timeout
                or time.monotonic() - arm_state["busy_at"] > self._state_timeout
                or arm_state["busy"]
            ):
                if goal_handle.is_cancel_requested:
                    self._stop_arm(arm)
                    return False, "cancelled while waiting for fresh arm state"
                if time.monotonic() > wait_deadline:
                    return False, "fresh idle pose/RCM state unavailable"
                await asyncio.sleep(0.05)
                arm_state = self._snapshot_arm(arm)

            scene, filtered_target, error = self._fresh_target(target_class)
            if error:
                return False, error
            scene_sync_deadline = min(
                overall_deadline, time.monotonic() + self._state_timeout
            )
            while True:
                expected_scene_revision = int(scene.obstacle_geometry_revision)
                synced_state = self._snapshot_arm(arm)
                if (
                    synced_state["collision_scene_revision"] == expected_scene_revision
                    and time.monotonic() - synced_state["collision_scene_at"] <= self._state_timeout
                ):
                    break
                if goal_handle.is_cancel_requested:
                    self._stop_arm(arm)
                    return False, "cancelled while waiting for MoveIt obstacle-scene update"
                if time.monotonic() >= scene_sync_deadline:
                    return False, "MoveIt has not acknowledged the current perception obstacle scene"
                await asyncio.sleep(0.05)
                scene, filtered_target, error = self._fresh_target(target_class)
                if error:
                    return False, error
            arm_state = self._snapshot_arm(arm)
            if (
                time.monotonic() - arm_state["pose_at"] > self._state_timeout
                or time.monotonic() - arm_state["rcm_at"] > self._state_timeout
                or time.monotonic() - arm_state["busy_at"] > self._state_timeout
                or arm_state["busy"]
            ):
                return False, "arm state changed while waiting for obstacle-scene synchronization"
            _, target_position = filtered_target
            try:
                target, final_position, desired_axis, angular_error = self._make_servo_target(
                    arm_state, scene, target_position, approach_offset
                )
            except ValueError as exc:
                return False, str(exc)

            final_error = self._distance_to_point(arm_state["pose"], final_position)
            if (
                final_error <= self._servo_position_tolerance
                and angular_error <= self._servo_angle_tolerance
            ):
                return True, (
                    f"visual servo converged: {final_error * 1000:.1f} mm, "
                    f"{math.degrees(angular_error):.1f} deg"
                )
            if iteration == self._servo_max_iterations:
                break

            feedback = TaskStep.Feedback()
            feedback.phase = (
                f"{step_name}: {arm} arm visual-servo step {iteration + 1}, "
                f"position error {final_error * 1000:.1f} mm"
            )
            feedback.progress = min(
                0.99,
                (arm_index + (iteration + 1) / self._servo_max_iterations)
                / arm_count,
            )
            goal_handle.publish_feedback(feedback)
            self._target_pubs[arm].publish(target)

            started_at = time.monotonic()
            saw_busy = False
            idle_since = None
            segment_deadline = min(overall_deadline, started_at + self._motion_timeout)
            start_deadline = started_at + self._start_timeout
            while time.monotonic() < segment_deadline:
                if goal_handle.is_cancel_requested:
                    self._stop_arm(arm)
                    return False, "cancelled during RCM waypoint motion"
                current_scene, _, scene_error = self._fresh_target(target_class)
                if scene_error:
                    self._stop_arm(arm)
                    return False, f"stopping because {scene_error}"
                current_revision = int(current_scene.obstacle_geometry_revision)
                synced_state = self._snapshot_arm(arm)
                if synced_state["collision_scene_revision"] != current_revision:
                    self._stop_arm(arm)
                    return False, (
                        "stopping because MoveIt has not acknowledged the latest "
                        "perception obstacle scene"
                    )
                arm_state = self._snapshot_arm(arm)
                now = time.monotonic()
                if (
                    now - arm_state["pose_at"] > self._state_timeout
                    or now - arm_state["rcm_at"] > self._state_timeout
                    or now - arm_state["busy_at"] > self._state_timeout
                ):
                    self._stop_arm(arm)
                    return False, "stopping because arm state became stale"
                if arm_state["pose"] is None:
                    await asyncio.sleep(0.05)
                    continue
                if arm_state["busy"]:
                    saw_busy = True
                    idle_since = None
                endpoint_error = self._distance_to_pose(arm_state["pose"], target)
                if not arm_state["busy"] and endpoint_error <= self._position_tolerance:
                    break
                if saw_busy and not arm_state["busy"]:
                    if idle_since is None:
                        idle_since = time.monotonic()
                    if time.monotonic() - idle_since < 0.5:
                        await asyncio.sleep(0.05)
                        continue
                    return False, f"RCM waypoint ended {endpoint_error * 1000:.1f} mm from target"
                if not saw_busy and time.monotonic() > start_deadline:
                    return False, "RCM node did not start the requested motion"
                await asyncio.sleep(0.05)
            else:
                self._stop_arm(arm)
                return False, "RCM waypoint motion timed out"

        self._stop_arm(arm)
        return False, "visual servo reached its maximum iteration count"

    @staticmethod
    def _distance_to_pose(pose: PoseStamped, target: PoseStamped) -> float:
        a = pose.pose.position
        b = target.pose.position
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    @staticmethod
    def _distance_to_point(pose: PoseStamped, point) -> float:
        p = pose.pose.position
        return math.sqrt((p.x - point[0]) ** 2 + (p.y - point[1]) ** 2 + (p.z - point[2]) ** 2)

    def _stop_arm(self, arm: str) -> None:
        message = Bool()
        message.data = True
        self._stop_pubs[arm].publish(message)

    def _stop_all(self) -> None:
        for arm in self.ARMS:
            self._stop_arm(arm)

    def _release_goal(self) -> None:
        with self._lock:
            self._active_goal = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RcmTaskExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._stop_all()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
