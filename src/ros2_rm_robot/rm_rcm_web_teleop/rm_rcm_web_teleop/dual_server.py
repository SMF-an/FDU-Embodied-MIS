#!/usr/bin/env python3
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .server import clamp, multiply_quaternions, normalize_quaternion, rotate_vector


class DualRcmWebTeleop(Node):
    SIDES = ("left", "right")

    def __init__(self):
        super().__init__("dual_rcm_web_teleop")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("step_angle_deg", 2.0)
        self.declare_parameter("step_insertion_m", 0.002)
        self.declare_parameter("camera_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_jpeg_quality", 78)
        self._host = self.get_parameter("host").value
        self._port = int(self.get_parameter("port").value)
        self._step_angle = math.radians(float(self.get_parameter("step_angle_deg").value))
        self._step_insertion = float(self.get_parameter("step_insertion_m").value)
        self._camera_topic = self.get_parameter("camera_topic").value
        self._camera_jpeg_quality = int(self.get_parameter("camera_jpeg_quality").value)
        self._mode = "simulation" if self.get_parameter("use_sim_time").value else "real"
        if not 1 <= self._port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if not 0.0 < self._step_angle <= math.radians(10.0):
            raise ValueError("step_angle_deg must be in (0, 10]")
        if not 0.0 < self._step_insertion <= 0.02:
            raise ValueError("step_insertion_m must be in (0, 0.02]")
        if not 1 <= self._camera_jpeg_quality <= 100:
            raise ValueError("camera_jpeg_quality must be between 1 and 100")

        self._lock = threading.Lock()
        self._camera_condition = threading.Condition()
        self._camera_bridge = CvBridge()
        self._camera_jpeg = None
        self._camera_sequence = 0
        self._camera_received = 0.0
        self._camera_width = 0
        self._camera_height = 0
        self._last_camera_encode = 0.0
        self._poses = {side: None for side in self.SIDES}
        self._pose_received = {side: 0.0 for side in self.SIDES}
        self._rcms = {side: None for side in self.SIDES}
        self._rcm_received = {side: 0.0 for side in self.SIDES}
        self._busy = {side: False for side in self.SIDES}
        self._active_arm = None
        self._active_since = 0.0
        self._active_saw_busy = False
        self._target_pubs = {}
        self._stop_pubs = {}
        self._camera_sub = self.create_subscription(
            Image,
            self._camera_topic,
            self._on_camera_image,
            qos_profile_sensor_data,
        )

        for side in self.SIDES:
            topic_root = f"/{side}_arm/rcm_demo"
            self._target_pubs[side] = self.create_publisher(
                PoseStamped, f"{topic_root}/web_target", 10
            )
            self._stop_pubs[side] = self.create_publisher(
                Bool, f"{topic_root}/stop", 10
            )
            self.create_subscription(
                PoseStamped,
                f"{topic_root}/current_tool_pose",
                lambda msg, arm=side: self._on_pose(arm, msg),
                10,
            )
            self.create_subscription(
                PointStamped,
                f"{topic_root}/rcm_point",
                lambda msg, arm=side: self._on_rcm(arm, msg),
                10,
            )
            self.create_subscription(
                Bool,
                f"{topic_root}/motion_busy",
                lambda msg, arm=side: self._on_busy(arm, msg),
                rclpy.qos.QoSProfile(
                    depth=1,
                    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                ),
            )

    def _on_pose(self, side, message):
        with self._lock:
            self._poses[side] = message
            self._pose_received[side] = time.monotonic()

    def _on_rcm(self, side, message):
        with self._lock:
            self._rcms[side] = message
            self._rcm_received[side] = time.monotonic()

    def _on_busy(self, side, message):
        with self._lock:
            self._busy[side] = message.data
            if side == self._active_arm:
                if message.data:
                    self._active_saw_busy = True
                elif self._active_saw_busy:
                    self._clear_active()

    def _on_camera_image(self, message):
        now = time.monotonic()
        if now - self._last_camera_encode < 1.0 / 15.0:
            return
        self._last_camera_encode = now
        try:
            image = self._camera_bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            success, encoded = cv2.imencode(
                ".jpg",
                image,
                [cv2.IMWRITE_JPEG_QUALITY, self._camera_jpeg_quality],
            )
            if not success:
                return
        except Exception as error:
            self.get_logger().warning(f"Cannot encode camera frame: {error}")
            return

        with self._camera_condition:
            self._camera_jpeg = encoded.tobytes()
            self._camera_sequence += 1
            self._camera_received = now
            self._camera_width = int(message.width)
            self._camera_height = int(message.height)
            self._camera_condition.notify_all()

    def wait_camera_frame(self, after_sequence, timeout):
        with self._camera_condition:
            if self._camera_sequence <= after_sequence:
                self._camera_condition.wait(timeout)
            if self._camera_sequence <= after_sequence:
                return after_sequence, None
            return self._camera_sequence, self._camera_jpeg

    def camera_state(self):
        with self._camera_condition:
            age = time.monotonic() - self._camera_received if self._camera_received else None
            return {
                "topic": self._camera_topic,
                "available": age is not None and age < 2.0,
                "age_sec": age,
                "width": self._camera_width,
                "height": self._camera_height,
            }

    def _clear_active(self):
        self._active_arm = None
        self._active_since = 0.0
        self._active_saw_busy = False

    def state(self):
        with self._lock:
            now = time.monotonic()
            if (
                self._active_arm is not None
                and not self._busy[self._active_arm]
                and not self._active_saw_busy
                and now - self._active_since > 3.0
            ):
                self._clear_active()

            arms = {}
            for side in self.SIDES:
                pose = self._poses[side]
                rcm = self._rcms[side]
                ready = (
                    pose is not None
                    and rcm is not None
                    and now - self._pose_received[side] < 1.0
                    and now - self._rcm_received[side] < 1.0
                )
                busy = self._busy[side] or self._active_arm == side
                result = {"ready": ready, "busy": busy}
                if ready:
                    p = pose.pose.position
                    q = pose.pose.orientation
                    result.update(
                        {
                            "frame": pose.header.frame_id,
                            "tip": {"x": p.x, "y": p.y, "z": p.z},
                            "orientation": {
                                "x": q.x,
                                "y": q.y,
                                "z": q.z,
                                "w": q.w,
                            },
                            "rcm": {
                                "x": rcm.point.x,
                                "y": rcm.point.y,
                                "z": rcm.point.z,
                            },
                            "tip_to_rcm_m": math.sqrt(
                                (p.x - rcm.point.x) ** 2
                                + (p.y - rcm.point.y) ** 2
                                + (p.z - rcm.point.z) ** 2
                            ),
                        }
                    )
                else:
                    mode_name = "仿真" if self._mode == "simulation" else "真机"
                    result["message"] = f"等待 {side} 臂 RCM {mode_name}状态"
                arms[side] = result
            return {
                "mode": self._mode,
                "step_angle_deg": math.degrees(self._step_angle),
                "step_insertion_mm": self._step_insertion * 1000.0,
                "locked_by": self._active_arm,
                "camera": self.camera_state(),
                "arms": arms,
            }

    def jog(self, body):
        side = body.get("arm")
        if side not in self.SIDES:
            return 400, {"error": "arm must be left or right"}
        try:
            tilt_x = float(body.get("tilt_x", 0.0))
            tilt_y = float(body.get("tilt_y", 0.0))
            insertion = float(body.get("insertion", 0.0))
        except (TypeError, ValueError):
            return 400, {"error": "joystick values must be numbers"}
        if not all(math.isfinite(v) for v in (tilt_x, tilt_y, insertion)):
            return 400, {"error": "joystick values must be finite"}
        tilt_x = clamp(tilt_x, -1.0, 1.0)
        tilt_y = clamp(tilt_y, -1.0, 1.0)
        insertion = clamp(insertion, -1.0, 1.0)
        if max(abs(tilt_x), abs(tilt_y), abs(insertion)) < 0.05:
            return 400, {"error": "no joystick input"}

        with self._lock:
            now = time.monotonic()
            if self._active_arm is not None or any(self._busy.values()):
                return 409, {"error": "another arm is moving; wait for it to stop"}
            pose = self._poses[side]
            rcm = self._rcms[side]
            if (
                pose is None
                or rcm is None
                or now - self._pose_received[side] >= 1.0
                or now - self._rcm_received[side] >= 1.0
            ):
                return 503, {"error": f"{side} arm RCM state is unavailable"}

            p = pose.pose.position
            q_msg = pose.pose.orientation
            q_current = normalize_quaternion((q_msg.x, q_msg.y, q_msg.z, q_msg.w))
            current_axis = rotate_vector(q_current, (0.0, 0.0, 1.0))
            qx = (
                math.sin(-tilt_y * self._step_angle / 2.0),
                0.0,
                0.0,
                math.cos(tilt_y * self._step_angle / 2.0),
            )
            qy = (
                0.0,
                math.sin(tilt_x * self._step_angle / 2.0),
                0.0,
                math.cos(tilt_x * self._step_angle / 2.0),
            )
            q_delta = multiply_quaternions(qx, qy)
            target_axis = rotate_vector(q_delta, current_axis)
            radius = math.sqrt(
                (p.x - rcm.point.x) ** 2
                + (p.y - rcm.point.y) ** 2
                + (p.z - rcm.point.z) ** 2
            ) + insertion * self._step_insertion
            if not 0.02 <= radius <= 1.0:
                return 400, {"error": "requested insertion is outside the supported range"}

            target = PoseStamped()
            target.header = pose.header
            target.header.stamp = self.get_clock().now().to_msg()
            target.pose.position.x = rcm.point.x + radius * target_axis[0]
            target.pose.position.y = rcm.point.y + radius * target_axis[1]
            target.pose.position.z = rcm.point.z + radius * target_axis[2]
            q_target = multiply_quaternions(q_delta, q_current)
            target.pose.orientation.x = q_target[0]
            target.pose.orientation.y = q_target[1]
            target.pose.orientation.z = q_target[2]
            target.pose.orientation.w = q_target[3]
            self._active_arm = side
            self._active_since = now
            self._active_saw_busy = False

        self._target_pubs[side].publish(target)
        return 202, {"accepted": True, "arm": side}

    def stop(self, side):
        if side not in self.SIDES:
            return False
        with self._lock:
            if self._active_arm not in (None, side):
                return False
        message = Bool()
        message.data = True
        self._stop_pubs[side].publish(message)
        return True


def make_handler(node, page):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *_args):
            return

        def _send(self, status, payload, content_type="application/json; charset=utf-8"):
            data = payload if isinstance(payload, bytes) else json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _body(self):
            length = int(self.headers.get("Content-Length", "0"))
            if length > 4096:
                raise ValueError("request too large")
            try:
                body = json.loads(self.rfile.read(length).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                raise ValueError("invalid JSON") from error
            if not isinstance(body, dict):
                raise ValueError("JSON object required")
            return body

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(200, page, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, node.state())
            elif self.path == "/camera.mjpg":
                self._stream_camera()
            else:
                self._send(404, {"error": "not found"})

        def _stream_camera(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            sequence = 0
            try:
                while True:
                    sequence, jpeg = node.wait_camera_frame(sequence, 2.0)
                    if jpeg is None:
                        continue
                    self.wfile.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                        + jpeg
                        + b"\r\n"
                    )
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                return

        def do_POST(self):
            if self.path not in ("/api/jog", "/api/stop"):
                self._send(404, {"error": "not found"})
                return
            try:
                body = self._body()
            except ValueError as error:
                code = 413 if str(error) == "request too large" else 400
                self._send(code, {"error": str(error)})
                return
            if self.path == "/api/stop":
                if not node.stop(body.get("arm")):
                    self._send(409, {"error": "another arm is moving"})
                    return
                self._send(202, {"accepted": True})
                return
            status, result = node.jog(body)
            self._send(status, result)

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = DualRcmWebTeleop()
    try:
        package_share = get_package_share_directory("rm_rcm_web_teleop")
        with open(os.path.join(package_share, "web", "dual.html"), "rb") as page_file:
            page = page_file.read()
        server = ThreadingHTTPServer((node._host, node._port), make_handler(node, page))
        server.daemon_threads = True
    except (OSError, ValueError) as error:
        node.get_logger().fatal(f"Cannot start dual RCM web teleop: {error}")
        node.destroy_node()
        rclpy.shutdown()
        return

    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"Dual RCM web teleop ready at http://{node._host}:{node._port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
