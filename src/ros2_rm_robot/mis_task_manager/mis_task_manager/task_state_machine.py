from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rm_ros_interfaces.action import TaskStep
from rm_ros_interfaces.msg import PerceptionScene, TaskStatus
from std_msgs.msg import Bool


@dataclass(frozen=True)
class MissionStep:
    task_id: int
    name: str
    arms: tuple[str, ...] = ()
    target_class: str = "lesion"
    approach_offset_m: float = 0.0


MISSION_STEPS = (
    MissionStep(1, "identify_and_plan"),
    MissionStep(2, "approach_and_avoid", ("left", "right"), "liver", 0.08),
    MissionStep(2, "left_arm_expose_lesion", ("left",), "lesion", 0.02),
    MissionStep(3, "dual_arm_coordinated_dissect_boundary", ("right",), "lesion", 0.02),
    MissionStep(3, "retrieve_lesion", ("left",), "lesion", 0.08),
)


class TaskStateMachine(Node):
    IDLE = "IDLE"
    WAITING_FOR_SCENE = "WAITING_FOR_SCENE"
    WAITING_FOR_EXECUTOR = "WAITING_FOR_EXECUTOR"
    EXECUTING = "EXECUTING"
    COMPLETED = "COMPLETED"
    ABORTED = "ABORTED"

    def __init__(self) -> None:
        super().__init__("mis_task_state_machine")
        self.declare_parameter("scene_topic", "/perception/scene")
        self.declare_parameter("start_topic", "/task/start")
        self.declare_parameter("cancel_topic", "/task/cancel")
        self.declare_parameter("status_topic", "/task/status")
        self.declare_parameter("executor_action", "/task_executor/step")
        self.declare_parameter("scene_timeout_sec", 15.0)
        self.declare_parameter("scene_max_age_sec", 1.0)
        self.declare_parameter("step_timeout_sec", 120.0)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("dry_run_step_sec", 1.0)

        self._scene: PerceptionScene | None = None
        self._scene_received_at = 0.0
        self._scene_generation = 0
        self._required_scene_generation = -1
        self._state = self.IDLE
        self._message = "waiting for one start signal"
        self._started_at = 0.0
        self._step_started_at = 0.0
        self._dry_run_deadline = 0.0
        self._step_index = 0
        self._step_progress = 0.0
        self._goal_handle = None
        self._goal_pending = False
        self._goal_generation = 0
        self._start_consumed = False

        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._scene_timeout = float(self.get_parameter("scene_timeout_sec").value)
        self._scene_max_age = float(self.get_parameter("scene_max_age_sec").value)
        self._step_timeout = float(self.get_parameter("step_timeout_sec").value)
        self._dry_run_step_sec = float(self.get_parameter("dry_run_step_sec").value)

        self._status_pub = self.create_publisher(
            TaskStatus, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            PerceptionScene,
            str(self.get_parameter("scene_topic").value),
            self._on_scene,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("start_topic").value),
            self._on_start,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("cancel_topic").value),
            self._on_cancel,
            10,
        )
        self._executor = ActionClient(
            self, TaskStep, str(self.get_parameter("executor_action").value)
        )
        self.create_timer(0.1, self._tick)
        self._publish_status()
        self.get_logger().info(
            f"Task state machine ready; dry_run={self._dry_run}, "
            f"steps={len(MISSION_STEPS)}"
        )

    def _on_scene(self, message: PerceptionScene) -> None:
        self._scene = message
        self._scene_received_at = time.monotonic()
        self._scene_generation += 1

    def _on_start(self, message: Bool) -> None:
        if not message.data:
            return
        if self._start_consumed:
            self.get_logger().warning("Start signal ignored: this mission is single-shot")
            return
        self._start_consumed = True
        self._started_at = time.monotonic()
        self._state = self.WAITING_FOR_SCENE
        self._message = "start accepted; waiting for a fresh valid scene"
        self._publish_status()

    def _on_cancel(self, message: Bool) -> None:
        if not message.data or self._state in (self.IDLE, self.COMPLETED, self.ABORTED):
            return
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._finish(self.ABORTED, "mission cancelled")

    def _scene_is_ready(self) -> bool:
        if self._scene is None:
            return False
        if time.monotonic() - self._scene_received_at > self._scene_max_age:
            return False
        if not self._scene.valid or not self._scene.transform_valid:
            return False
        return any(
            item.detected and item.position_valid and item.world_position_valid
            for item in self._scene.objects
        )

    def _tick(self) -> None:
        now = time.monotonic()
        if self._state == self.WAITING_FOR_SCENE:
            if self._scene_generation > self._required_scene_generation and self._scene_is_ready():
                if self._step_index >= len(MISSION_STEPS):
                    self._finish(self.COMPLETED, "all mission steps completed")
                else:
                    self._begin_step(now)
            elif now - self._started_at > self._scene_timeout:
                self._finish(self.ABORTED, "timed out waiting for a valid perception scene")
        elif self._state == self.WAITING_FOR_EXECUTOR:
            if self._dry_run:
                self._state = self.EXECUTING
                self._dry_run_deadline = now + self._dry_run_step_sec
                self._message = f"dry run: {self._current_step().name}"
            elif self._executor.wait_for_server(timeout_sec=0.0):
                self._send_step_goal()
            elif now - self._step_started_at > self._step_timeout:
                self._finish(self.ABORTED, "timed out waiting for task executor")
        elif self._state == self.EXECUTING:
            if self._dry_run and now >= self._dry_run_deadline:
                self._complete_step(f"dry run completed {self._current_step().name}")
            elif not self._dry_run and now - self._step_started_at > self._step_timeout:
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
                self._finish(self.ABORTED, f"step timed out: {self._current_step().name}")
        self._publish_status()

    def _current_step(self) -> MissionStep:
        return MISSION_STEPS[self._step_index]

    def _begin_step(self, now: float) -> None:
        self._state = self.WAITING_FOR_EXECUTOR
        self._step_started_at = now
        self._step_progress = 0.0
        step = self._current_step()
        self._message = f"starting task {step.task_id}: {step.name}"

    def _send_step_goal(self) -> None:
        if self._goal_pending or self._scene is None:
            return
        step = self._current_step()
        goal = TaskStep.Goal()
        goal.task_id = step.task_id
        goal.step = step.name
        goal.arms = list(step.arms)
        goal.target_class = step.target_class
        goal.approach_offset_m = step.approach_offset_m
        goal.scene = self._scene
        self._goal_pending = True
        self._state = self.EXECUTING
        self._message = f"executor running task {step.task_id}: {step.name}"
        self._goal_generation += 1
        generation = self._goal_generation
        future = self._executor.send_goal_async(
            goal, feedback_callback=lambda feedback: self._on_feedback(feedback, generation)
        )
        future.add_done_callback(lambda result: self._on_goal_response(result, generation))

    def _on_goal_response(self, future, generation: int) -> None:
        if generation != self._goal_generation:
            return
        self._goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish(self.ABORTED, f"executor goal failed: {exc}")
            return
        if self._state != self.EXECUTING:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self._goal_handle = goal_handle
        if not goal_handle.accepted:
            self._finish(self.ABORTED, "task executor rejected the step")
            return
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_step_result(result, generation)
        )

    def _on_step_result(self, future, generation: int) -> None:
        if generation != self._goal_generation or self._state != self.EXECUTING:
            return
        self._goal_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self._finish(self.ABORTED, f"executor result failed: {exc}")
            return
        if not result.success:
            self._finish(self.ABORTED, result.message or "task step failed")
            return
        self._complete_step(result.message or f"completed {self._current_step().name}")

    def _on_feedback(self, feedback_message, generation: int) -> None:
        if generation != self._goal_generation or self._state != self.EXECUTING:
            return
        feedback = feedback_message.feedback
        self._step_progress = min(1.0, max(0.0, float(feedback.progress)))
        if feedback.phase:
            self._message = feedback.phase
        self._publish_status()

    def _complete_step(self, message: str) -> None:
        self._step_index += 1
        self._step_progress = 0.0
        if self._step_index >= len(MISSION_STEPS):
            self._finish(self.COMPLETED, message)
            return
        self._required_scene_generation = self._scene_generation
        self._state = self.WAITING_FOR_SCENE
        self._started_at = time.monotonic()
        self._message = message
        self._publish_status()

    def _finish(self, state: str, message: str) -> None:
        self._state = state
        self._goal_pending = False
        self._goal_handle = None
        self._message = message
        self.get_logger().info(f"Mission {state}: {message}")
        self._publish_status()

    def _publish_status(self) -> None:
        status = TaskStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self._scene.header.frame_id if self._scene else ""
        status.state = self._state
        status.active = self._state in (
            self.WAITING_FOR_SCENE,
            self.WAITING_FOR_EXECUTOR,
            self.EXECUTING,
        )
        status.message = self._message
        if self._step_index < len(MISSION_STEPS):
            step = self._current_step()
            status.task_id = step.task_id
            status.step = step.name
            status.progress = (
                self._step_index + self._step_progress
            ) / len(MISSION_STEPS)
        elif self._state == self.COMPLETED:
            status.progress = 1.0
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskStateMachine()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
