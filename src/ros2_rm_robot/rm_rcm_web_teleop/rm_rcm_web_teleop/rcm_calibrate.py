#!/usr/bin/env python3
"""Interactively set and save fixed RCM points for the dual RM65 arms."""

import argparse
import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Empty


class RcmCalibrator(Node):
    SIDES = ("left", "right")
    PLANNING_FRAME = "world"

    def __init__(self, output_path):
        super().__init__("dual_rcm_calibrator")
        self.output_path = os.path.abspath(os.path.expanduser(output_path))
        self.lock = threading.Lock()
        self.poses = {side: None for side in self.SIDES}
        self.results = {side: {} for side in self.SIDES}
        self.active_drag = None
        self.points = self.load_existing()
        self.arm_publishers = {}
        for side in self.SIDES:
            root = f"/{side}_arm"
            self.arm_publishers[side] = {
                "start": self.create_publisher(Empty, f"{root}/rm_driver/start_drag_teach_cmd", 10),
                "stop": self.create_publisher(Empty, f"{root}/rm_driver/stop_drag_teach_cmd", 10),
                "rcm": self.create_publisher(PointStamped, f"{root}/rcm_demo/set_rcm_point", 10),
            }
            self.create_subscription(
                PoseStamped, f"{root}/rcm_demo/current_tool_pose",
                lambda msg, arm=side: self.on_pose(arm, msg), 10,
            )
            self.create_subscription(
                Bool, f"{root}/rm_driver/start_drag_teach_result",
                lambda msg, arm=side: self.on_result(arm, "start", msg), 10,
            )
            self.create_subscription(
                Bool, f"{root}/rm_driver/stop_drag_teach_result",
                lambda msg, arm=side: self.on_result(arm, "stop", msg), 10,
            )
            self.create_subscription(
                Bool, f"{root}/rcm_demo/set_rcm_point_result",
                lambda msg, arm=side: self.on_result(arm, "set_rcm", msg), 10,
            )

    def load_existing(self):
        try:
            with open(self.output_path, "r", encoding="utf-8") as stream:
                data = json.load(stream)
            if (
                isinstance(data, dict)
                and data.get("format") == "rm_dual_rcm_points_v2"
                and isinstance(data.get("arms"), dict)
            ):
                return {
                    side: point for side, point in data["arms"].items()
                    if side in self.SIDES and isinstance(point, dict)
                    and point.get("frame_id") == self.PLANNING_FRAME
                    and all(
                        isinstance(point.get(axis), (int, float))
                        and math.isfinite(point[axis])
                        for axis in ("x", "y", "z")
                    )
                }
        except (OSError, ValueError, AttributeError, TypeError):
            pass
        return {}

    def on_pose(self, side, message):
        with self.lock:
            self.poses[side] = (message, time.monotonic())

    def on_result(self, side, operation, message):
        with self.lock:
            event = self.results[side].get(operation)
            if event is not None:
                event.set_result(bool(message.data))

    def wait_pose(self, side):
        with self.lock:
            entry = self.poses[side]
        if entry is None or time.monotonic() - entry[1] > 1.0:
            raise RuntimeError(f"{side} 臂当前尖端位姿不可用")
        return entry[0]

    def operation(self, side, name, timeout=5.0):
        event = threading.Event()
        result = {"value": None}

        class ResultEvent:
            def set_result(_self, value):
                result["value"] = value
                event.set()

        with self.lock:
            self.results[side][name] = ResultEvent()
        if name == "start":
            self.arm_publishers[side]["start"].publish(Empty())
        elif name == "stop":
            self.arm_publishers[side]["stop"].publish(Empty())
        else:
            raise ValueError(name)
        if not event.wait(timeout):
            raise RuntimeError(f"等待 {side} 臂 {name} 响应超时")
        if not result["value"]:
            raise RuntimeError(f"{side} 臂驱动拒绝 {name} 命令")
        with self.lock:
            self.results[side].pop(name, None)

    def start(self, side):
        if side not in self.SIDES:
            raise ValueError("机械臂必须是 left 或 right")
        if self.active_drag:
            raise RuntimeError(f"请先结束 {self.active_drag} 臂手扶拖动")
        self.wait_pose(side)
        self.operation(side, "start")
        self.active_drag = side
        print(f"{side} 臂已进入手扶拖动。轻扶尖端对准穿刺孔中心后输入 r。")

    def record(self, side):
        if self.active_drag != side:
            raise RuntimeError(f"请先对 {side} 臂执行 s")
        pose = self.wait_pose(side)
        point = pose.pose.position
        self.points[side] = {
            "frame_id": pose.header.frame_id,
            "x": float(point.x), "y": float(point.y), "z": float(point.z),
        }
        print(f"记录 {side}: frame={pose.header.frame_id}, "
              f"({point.x * 1000:.1f}, {point.y * 1000:.1f}, {point.z * 1000:.1f}) mm")

    def finish(self, side):
        if self.active_drag != side or side not in self.points:
            raise RuntimeError(f"{side} 臂需要先手扶拖动并记录孔位")
        point = self.points[side]
        pose = self.wait_pose(side)
        if point["frame_id"] != self.PLANNING_FRAME:
            raise RuntimeError(
                f"当前 planning frame 是 {point['frame_id']}；本项目 RCM 标定统一要求 {self.PLANNING_FRAME}"
            )
        if pose.header.frame_id != point["frame_id"]:
            raise RuntimeError("记录点和当前位姿 frame 不一致")
        p, q = pose.pose.position, pose.pose.orientation
        qn = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if qn < 1e-12:
            raise RuntimeError("当前尖端姿态四元数无效")
        x, y, z, w = q.x/qn, q.y/qn, q.z/qn, q.w/qn
        axis = (2*(x*z + w*y), 2*(y*z - w*x), 1 - 2*(x*x + y*y))
        delta = (p.x-point["x"], p.y-point["y"], p.z-point["z"])
        projection = sum(delta[i]*axis[i] for i in range(3))
        error = math.sqrt(sum((delta[i]-projection*axis[i])**2 for i in range(3)))
        radius = math.sqrt(sum(v*v for v in delta))
        if radius > 1.0 or error > 0.002:
            raise RuntimeError(f"当前器具轴线距目标点 {error*1000:.1f} mm，需小于 2 mm")
        self.operation(side, "stop")
        self.active_drag = None
        message = PointStamped()
        message.header.frame_id = point["frame_id"]
        message.header.stamp = self.get_clock().now().to_msg()
        message.point.x, message.point.y, message.point.z = point["x"], point["y"], point["z"]
        event = threading.Event()
        result = {"value": None}

        class ResultEvent:
            def set_result(_self, value):
                result["value"] = value
                event.set()

        with self.lock:
            self.results[side]["set_rcm"] = ResultEvent()
        self.arm_publishers[side]["rcm"].publish(message)
        if not event.wait(5.0) or not result["value"]:
            raise RuntimeError(f"{side} RCM 节点未确认应用标定点")
        with self.lock:
            self.results[side].pop("set_rcm", None)
        self.save()
        print(f"{side} RCM 已应用并保存到 {self.output_path}")

    def cancel(self, side):
        if self.active_drag != side:
            print(f"{side} 臂当前不在手扶拖动模式")
            return
        self.operation(side, "stop")
        self.active_drag = None
        print(f"{side} 手扶拖动已停止；未更改 RCM")

    def save(self):
        os.makedirs(os.path.dirname(self.output_path) or ".", mode=0o700, exist_ok=True)
        data = {
            "format": "rm_dual_rcm_points_v2",
            "units": "meters",
            "arms": self.points,
        }
        temporary = self.output_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
        os.replace(temporary, self.output_path)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="data/calibration/dual_rcm_points.json")
    parsed = parser.parse_args(args=args)
    rclpy.init(args=None)
    node = RcmCalibrator(parsed.output)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    print("RCM 标定命令: s left|right 开启手扶拖动, r 记录孔位, a 应用并保存, "
          "x 取消拖动, q 退出。先停止双臂 Web 遥控器。")
    try:
        while True:
            try:
                words = input("rcm> ").strip().split()
            except EOFError:
                break
            if not words:
                continue
            if words[0] in ("q", "quit"):
                break
            if len(words) != 2 or words[1] not in RcmCalibrator.SIDES:
                print("输入 s|r|a|x left|right 或 q")
                continue
            action, side = words
            try:
                if action == "s":
                    node.start(side)
                elif action == "r":
                    node.record(side)
                elif action == "a":
                    node.finish(side)
                elif action == "x":
                    node.cancel(side)
                else:
                    print("输入 s|r|a|x left|right 或 q")
            except (RuntimeError, ValueError) as error:
                print(f"操作失败: {error}")
    finally:
        if node.active_drag:
            try:
                node.operation(node.active_drag, "stop")
            except RuntimeError:
                pass
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
