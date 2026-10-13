#!/usr/bin/env python3
"""Replay accepted dual-arm RCM targets recorded by the web teleop server."""

import json
import os
import time

import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool

from .dual_server import ServoController


class RcmTeachingPlayback(Node):
    SIDES = ("left", "right")

    def __init__(self):
        super().__init__("rcm_teaching_playback")
        self.declare_parameter("file", "")
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("busy_timeout_sec", 30.0)
        self.declare_parameter("pause_limit_sec", 10.0)
        self._pubs = {
            side: self.create_publisher(
                PoseStamped, f"/{side}_arm/rcm_demo/web_target", 10
            ) for side in self.SIDES
        }
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._busy = {side: False for side in self.SIDES}
        config_path = os.path.join(
            get_package_share_directory("rm_rcm_web_teleop"), "config", "scs215.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as stream:
            servo_config = yaml.safe_load(stream)
        self._grippers = {
            side: ServoController(servo_config["servos"][side])
            for side in self.SIDES
        }
        for side in self.SIDES:
            self.create_subscription(
                Bool,
                f"/{side}_arm/rcm_demo/motion_busy",
                lambda message, arm=side: self._busy.__setitem__(arm, message.data),
                qos,
            )

    def load_actions(self):
        path = self.get_parameter("file").value
        if not path:
            raise ValueError("parameter file is required")
        path = os.path.abspath(os.path.expanduser(path))
        with open(path, "r", encoding="utf-8") as stream:
            document = json.load(stream)
        if document.get("format") != "rm_rcm_teaching_v1":
            raise ValueError("unsupported teaching file format")
        actions = document.get("actions")
        if not isinstance(actions, list) or not actions:
            raise ValueError("teaching file contains no actions")
        for action in actions:
            if action.get("arm") not in self.SIDES:
                raise ValueError("teaching action has invalid arm")
            if action.get("type") == "gripper":
                if action.get("action") not in ("open", "close"):
                    raise ValueError("teaching gripper action must be open or close")
                continue
            for key in ("position", "orientation"):
                if not isinstance(action.get(key), dict):
                    raise ValueError(f"teaching action missing {key}")
        self._start_poses = document.get("start_poses")
        if self._start_poses is not None:
            if not isinstance(self._start_poses, dict):
                raise ValueError("teaching start_poses must be an object")
            for side in self.SIDES:
                pose = self._start_poses.get(side)
                if not isinstance(pose, dict):
                    raise ValueError(f"teaching start pose missing {side} arm")
                for key in ("position", "orientation"):
                    if not isinstance(pose.get(key), dict):
                        raise ValueError(f"teaching {side} start pose missing {key}")
        return path, actions

    def wait_for_motion(self, side):
        timeout = float(self.get_parameter("busy_timeout_sec").value)
        deadline = time.monotonic() + timeout
        saw_busy = False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._busy[side]:
                saw_busy = True
            if saw_busy and not self._busy[side]:
                return
        raise TimeoutError(f"{side} arm did not finish within {timeout:.1f} seconds")

    def make_target(self, action):
        target = PoseStamped()
        target.header.frame_id = action.get("frame_id", "base_link")
        position = action["position"]
        orientation = action["orientation"]
        target.pose.position.x = float(position["x"])
        target.pose.position.y = float(position["y"])
        target.pose.position.z = float(position["z"])
        target.pose.orientation.x = float(orientation["x"])
        target.pose.orientation.y = float(orientation["y"])
        target.pose.orientation.z = float(orientation["z"])
        target.pose.orientation.w = float(orientation["w"])
        target.header.stamp = self.get_clock().now().to_msg()
        return target

    def replay(self, actions):
        speed = float(self.get_parameter("speed").value)
        pause_limit = float(self.get_parameter("pause_limit_sec").value)
        if not speed > 0.0:
            raise ValueError("speed must be greater than zero")
        previous_time = 0.0
        replay_clock = 0.0
        replay_started = time.monotonic()
        if self._start_poses:
            self.get_logger().warn("先将左右臂移动到记录起始位姿")
            for side in self.SIDES:
                while self._busy[side]:
                    rclpy.spin_once(self, timeout_sec=0.05)
                start_action = {"arm": side, **self._start_poses[side]}
                self._pubs[side].publish(self.make_target(start_action))
                self.get_logger().info(f"移动到 {side} 臂起始位姿")
                self.wait_for_motion(side)
            replay_started = time.monotonic()
        for index, action in enumerate(actions, 1):
            recorded_time = float(action.get("time_sec", previous_time))
            pause = min(max(recorded_time - previous_time, 0.0), pause_limit)
            replay_clock += pause / speed
            wait = replay_started + replay_clock - time.monotonic()
            if wait > 0.0:
                time.sleep(wait)
            side = action["arm"]
            if action.get("type") == "gripper":
                success, message = self._grippers[side].set_grip(action["action"])
                if not success:
                    raise RuntimeError(f"{side} gripper command failed: {message}")
                self.get_logger().info(
                    f"回放 {index}/{len(actions)}：{side} 臂舵机{action['action']}"
                )
                previous_time = recorded_time
                continue
            while self._busy[side]:
                rclpy.spin_once(self, timeout_sec=0.05)
            self._pubs[side].publish(self.make_target(action))
            self.get_logger().info(f"回放 {index}/{len(actions)}：{side} 臂")
            self.wait_for_motion(side)
            previous_time = recorded_time


def main(args=None):
    rclpy.init(args=args)
    node = RcmTeachingPlayback()
    try:
        path, actions = node.load_actions()
        node.get_logger().warn(
            f"开始回放 {path}，共 {len(actions)} 个动作；请确认工作区安全并准备急停"
        )
        node.replay(actions)
        node.get_logger().info("示教回放完成")
    except (OSError, RuntimeError, ValueError, TimeoutError) as error:
        node.get_logger().error(f"示教回放失败: {error}")
    finally:
        for gripper in node._grippers.values():
            gripper.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
