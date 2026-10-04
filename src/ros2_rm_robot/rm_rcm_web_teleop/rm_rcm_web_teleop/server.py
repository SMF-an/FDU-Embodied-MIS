#!/usr/bin/env python3
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_quaternion(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def multiply_quaternions(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quaternion(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def rotate_vector(q, vector):
    x, y, z, w = normalize_quaternion(q)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


class RcmWebTeleop(Node):
    def __init__(self):
        super().__init__("rcm_web_teleop")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("step_angle_deg", 2.0)
        self.declare_parameter("step_insertion_m", 0.002)
        self._host = self.get_parameter("host").value
        self._port = int(self.get_parameter("port").value)
        self._mode = "simulation" if self.get_parameter("use_sim_time").value else "real"
        self._step_angle = math.radians(float(self.get_parameter("step_angle_deg").value))
        self._step_insertion = float(self.get_parameter("step_insertion_m").value)
        if not 1 <= self._port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if not 0.0 < self._step_angle <= math.radians(10.0):
            raise ValueError("step_angle_deg must be in (0, 10]")
        if not 0.0 < self._step_insertion <= 0.02:
            raise ValueError("step_insertion_m must be in (0, 0.02]")

        self._lock = threading.Lock()
        self._pose = None
        self._pose_received = 0.0
        self._rcm = None
        self._rcm_received = 0.0
        self._busy = None
        self._target_pub = self.create_publisher(PoseStamped, "/rcm_demo/web_target", 10)
        self._stop_pub = self.create_publisher(Bool, "/rcm_demo/stop", 10)
        self.create_subscription(PoseStamped, "/rcm_demo/current_tool_pose", self._on_pose, 10)
        self.create_subscription(PointStamped, "/rcm_demo/rcm_point", self._on_rcm, 10)
        self.create_subscription(
            Bool,
            "/rcm_demo/motion_busy",
            self._on_busy,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            ),
        )

    def _on_pose(self, message):
        with self._lock:
            self._pose = message
            self._pose_received = time.monotonic()

    def _on_rcm(self, message):
        with self._lock:
            self._rcm = message
            self._rcm_received = time.monotonic()

    def _on_busy(self, message):
        with self._lock:
            self._busy = message.data

    def state(self):
        with self._lock:
            now = time.monotonic()
            pose = self._pose
            rcm = self._rcm
            pose_age = now - self._pose_received
            rcm_age = now - self._rcm_received
            ready = pose is not None and rcm is not None and pose_age < 1.0 and rcm_age < 1.0
            if not ready:
                mode_name = "仿真" if self._mode == "simulation" else "真机"
                return {
                    "ready": False,
                    "busy": bool(self._busy),
                    "mode": self._mode,
                    "message": f"等待 RCM {mode_name}状态",
                }
            p = pose.pose.position
            q = pose.pose.orientation
            return {
                "ready": True,
                "busy": bool(self._busy),
                "mode": self._mode,
                "frame": pose.header.frame_id,
                "tip": {"x": p.x, "y": p.y, "z": p.z},
                "orientation": {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
                "rcm": {"x": rcm.point.x, "y": rcm.point.y, "z": rcm.point.z},
                "tip_to_rcm_m": math.sqrt(
                    (p.x - rcm.point.x) ** 2
                    + (p.y - rcm.point.y) ** 2
                    + (p.z - rcm.point.z) ** 2
                ),
                "step_angle_deg": math.degrees(self._step_angle),
                "step_insertion_mm": self._step_insertion * 1000.0,
            }

    def jog(self, body):
        try:
            tilt_x = float(body.get("tilt_x", 0.0))
            tilt_y = float(body.get("tilt_y", 0.0))
            insertion = float(body.get("insertion", 0.0))
        except (TypeError, ValueError):
            return 400, {"error": "joystick values must be numbers"}
        if not all(math.isfinite(value) for value in (tilt_x, tilt_y, insertion)):
            return 400, {"error": "joystick values must be finite"}
        tilt_x = clamp(tilt_x, -1.0, 1.0)
        tilt_y = clamp(tilt_y, -1.0, 1.0)
        insertion = clamp(insertion, -1.0, 1.0)
        if max(abs(tilt_x), abs(tilt_y), abs(insertion)) < 0.05:
            return 400, {"error": "no joystick input"}

        with self._lock:
            now = time.monotonic()
            if self._busy:
                return 409, {"error": "robot is moving; wait for the current step"}
            if (
                self._pose is None
                or self._rcm is None
                or now - self._pose_received >= 1.0
                or now - self._rcm_received >= 1.0
            ):
                return 503, {"error": "RCM robot state is unavailable"}
            current_pose = self._pose
            pivot = self._rcm

        position = current_pose.pose.position
        q_msg = current_pose.pose.orientation
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
            (position.x - pivot.point.x) ** 2
            + (position.y - pivot.point.y) ** 2
            + (position.z - pivot.point.z) ** 2
        ) + insertion * self._step_insertion
        if not 0.02 <= radius <= 1.0:
            return 400, {"error": "requested insertion is outside the supported range"}

        target = PoseStamped()
        target.header = current_pose.header
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = pivot.point.x + radius * target_axis[0]
        target.pose.position.y = pivot.point.y + radius * target_axis[1]
        target.pose.position.z = pivot.point.z + radius * target_axis[2]
        q_target = multiply_quaternions(q_delta, q_current)
        target.pose.orientation.x = q_target[0]
        target.pose.orientation.y = q_target[1]
        target.pose.orientation.z = q_target[2]
        target.pose.orientation.w = q_target[3]
        self._target_pub.publish(target)
        return 202, {"accepted": True}

    def stop(self):
        message = Bool()
        message.data = True
        self._stop_pub.publish(message)


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

        def do_GET(self):
            if self.path == "/" or self.path == "/index.html":
                self._send(200, page, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, node.state())
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            length = int(self.headers.get("Content-Length", "0"))
            if length > 4096:
                self._send(413, {"error": "request too large"})
                return
            if self.path == "/api/stop":
                node.stop()
                self._send(202, {"accepted": True})
                return
            if self.path != "/api/jog":
                self._send(404, {"error": "not found"})
                return
            try:
                body = json.loads(self.rfile.read(length).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send(400, {"error": "invalid JSON"})
                return
            if not isinstance(body, dict):
                self._send(400, {"error": "JSON object required"})
                return
            status, result = node.jog(body)
            self._send(status, result)

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = RcmWebTeleop()
    try:
        package_share = get_package_share_directory("rm_rcm_web_teleop")
        with open(f"{package_share}/web/index.html", "rb") as page_file:
            page = page_file.read()
        handler = make_handler(node, page)
        server = ThreadingHTTPServer((node._host, node._port), handler)
        server.daemon_threads = True
    except (OSError, ValueError) as error:
        node.get_logger().fatal(f"Cannot start RCM web teleop: {error}")
        node.destroy_node()
        rclpy.shutdown()
        return

    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(f"RCM web teleop ready at http://{node._host}:{node._port}")
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
