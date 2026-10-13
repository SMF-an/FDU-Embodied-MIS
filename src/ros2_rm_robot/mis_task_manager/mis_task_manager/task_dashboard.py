from __future__ import annotations

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rm_ros_interfaces.action import TaskStep
from rm_ros_interfaces.msg import PerceptionScene, Stop, TaskStatus
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Empty, UInt64


def normalize_quaternion(q):
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion")
    return tuple(value / norm for value in q)


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


class ServoProbe:
    def __init__(self, settings):
        self.port_name = str(settings.get("port", ""))
        self.servo_id = int(settings.get("servo_id", 1))
        self.baud_rate = int(settings.get("baud_rate", 1_000_000))
        self._lock = threading.Lock()
        self._port = None
        self._servo = None
        self._port_open = False
        self._connected = False
        self._message = "尚未检测"
        self._last_check = 0.0

    def probe(self):
        with self._lock:
            try:
                if self._servo is None:
                    from scservo_sdk import PortHandler, scscl

                    self._port = PortHandler(self.port_name)
                    self._servo = scscl(self._port)
                if not self._port_open:
                    if not self._port.openPort():
                        raise RuntimeError(f"无法打开串口 {self.port_name}")
                    self._port_open = True
                    if not self._port.setBaudRate(self.baud_rate):
                        raise RuntimeError(f"无法设置波特率 {self.baud_rate}")
                model, result, error = self._servo.ping(self.servo_id)
                self._connected = result == 0 and error == 0
                self._message = (
                    f"在线，型号 {model}"
                    if self._connected
                    else f"ping 失败 result={result} error={error}"
                )
                if not self._connected:
                    self._close_port()
            except Exception as exc:
                self._connected = False
                self._message = str(exc)
                self._close_port()
            self._last_check = time.monotonic()

    def _close_port(self):
        if self._port is not None and self._port_open:
            try:
                self._port.closePort()
            except Exception:
                pass
        self._port_open = False

    def state(self):
        with self._lock:
            age = time.monotonic() - self._last_check if self._last_check else None
            return {
                "connected": self._connected and age is not None and age < 6.0,
                "port": self.port_name,
                "servo_id": self.servo_id,
                "message": self._message if age is None or age < 6.0 else "检测超时",
                "age_sec": age,
            }

    def close(self):
        with self._lock:
            self._close_port()
            self._connected = False


class TaskDashboard(Node):
    ARMS = ("left", "right")
    ARM_STATUS_TOPIC = "/task/status"

    def __init__(self):
        super().__init__("mis_task_dashboard")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8780)
        self.declare_parameter("camera_topic", "/perception/overlay")
        self.declare_parameter("scene_topic", "/perception/scene")
        self.declare_parameter("status_topic", self.ARM_STATUS_TOPIC)
        self.declare_parameter("servo_config_file", "")
        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)
        self._camera_topic = str(self.get_parameter("camera_topic").value)
        self._scene_topic = str(self.get_parameter("scene_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        if not 1 <= self._port <= 65535:
            raise ValueError("port must be between 1 and 65535")

        self._lock = threading.RLock()
        self._camera_condition = threading.Condition()
        self._camera_bridge = CvBridge()
        self._camera_jpeg = None
        self._camera_sequence = 0
        self._camera_received = 0.0
        self._camera_width = 0
        self._camera_height = 0
        self._last_camera_encode = 0.0
        self._scene = None
        self._scene_received = 0.0
        self._task_status = None
        self._task_status_received = 0.0
        self._executor_online = False
        self._estop_latched = False
        self._estop_time = 0.0
        self._arms = {
            arm: {
                "pose": None,
                "pose_at": 0.0,
                "rcm": None,
                "rcm_at": 0.0,
                "busy": False,
                "busy_at": 0.0,
                "collision_scene_revision": None,
                "collision_scene_at": 0.0,
                "joint_state": None,
                "joint_at": 0.0,
                "estop_ack": None,
                "estop_ack_at": 0.0,
            }
            for arm in self.ARMS
        }

        self._start_pub = self.create_publisher(Bool, "/task/start", 10)
        self._cancel_pub = self.create_publisher(Bool, "/task/cancel", 10)
        self._rcm_stop_pubs = {}
        self._driver_stop_pubs = {}
        self._driver_estop_pubs = {}
        self._action_client = ActionClient(self, TaskStep, "/task_executor/step")
        self.create_timer(0.5, self._refresh_executor_state)
        self.create_subscription(
            PerceptionScene, self._scene_topic, self._on_scene, 10
        )
        self.create_subscription(
            TaskStatus, self._status_topic, self._on_task_status, 10
        )
        self.create_subscription(
            Image,
            self._camera_topic,
            self._on_camera_image,
            qos_profile_sensor_data,
        )
        for arm in self.ARMS:
            root = f"/{arm}_arm"
            self._rcm_stop_pubs[arm] = self.create_publisher(
                Bool, f"{root}/rcm_demo/stop", 10
            )
            self._driver_stop_pubs[arm] = self.create_publisher(
                Empty, f"{root}/rm_driver/move_stop_cmd", 10
            )
            self._driver_estop_pubs[arm] = self.create_publisher(
                Stop, f"{root}/rm_driver/emergency_stop_cmd", 10
            )
            self.create_subscription(
                PoseStamped,
                f"{root}/rcm_demo/current_tool_pose",
                lambda msg, side=arm: self._on_pose(side, msg),
                10,
            )
            self.create_subscription(
                PointStamped,
                f"{root}/rcm_demo/rcm_point",
                lambda msg, side=arm: self._on_rcm(side, msg),
                10,
            )
            self.create_subscription(
                Bool,
                f"{root}/rcm_demo/motion_busy",
                lambda msg, side=arm: self._on_busy(side, msg),
                10,
            )
            self.create_subscription(
                UInt64,
                f"{root}/rcm_demo/collision_scene_synced",
                lambda msg, side=arm: self._on_collision_scene_synced(side, msg),
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            )
            self.create_subscription(
                JointState,
                f"{root}/moveit_joint_states",
                lambda msg, side=arm: self._on_joint_state(side, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Bool,
                f"{root}/rm_driver/emergency_stop_result",
                lambda msg, side=arm: self._on_estop_ack(side, msg),
                10,
            )

        config_file = str(self.get_parameter("servo_config_file").value)
        if not config_file:
            package_share = get_package_share_directory("rm_rcm_web_teleop")
            config_file = os.path.join(package_share, "config", "scs215.yaml")
        self._servos = self._load_servo_probes(config_file)
        self._servo_stop = threading.Event()
        self._servo_thread = threading.Thread(
            target=self._probe_servos, daemon=True, name="servo-status-probe"
        )
        self._servo_thread.start()
        self.get_logger().info(
            f"Task dashboard ROS ready; camera={self._camera_topic}, "
            f"HTTP=http://{self._host}:{self._port}"
        )

    def _load_servo_probes(self, filename):
        try:
            with open(os.path.expanduser(filename), "r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            servos = config.get("servos", {})
            return {
                arm: ServoProbe(servos[arm])
                for arm in self.ARMS
                if isinstance(servos.get(arm), dict)
            }
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            self.get_logger().error(f"Cannot load servo config {filename}: {exc}")
            return {}

    def _probe_servos(self):
        while not self._servo_stop.is_set():
            for probe in self._servos.values():
                probe.probe()
            self._servo_stop.wait(3.0)

    def _on_scene(self, message):
        with self._lock:
            self._scene = message
            self._scene_received = time.monotonic()

    def _refresh_executor_state(self):
        with self._lock:
            self._executor_online = self._action_client.server_is_ready()

    def _on_task_status(self, message):
        with self._lock:
            self._task_status = message
            self._task_status_received = time.monotonic()

    def _on_pose(self, arm, message):
        with self._lock:
            self._arms[arm]["pose"] = message
            self._arms[arm]["pose_at"] = time.monotonic()

    def _on_rcm(self, arm, message):
        with self._lock:
            self._arms[arm]["rcm"] = message
            self._arms[arm]["rcm_at"] = time.monotonic()

    def _on_busy(self, arm, message):
        with self._lock:
            self._arms[arm]["busy"] = bool(message.data)
            self._arms[arm]["busy_at"] = time.monotonic()

    def _on_collision_scene_synced(self, arm, message):
        with self._lock:
            self._arms[arm]["collision_scene_revision"] = int(message.data)
            self._arms[arm]["collision_scene_at"] = time.monotonic()

    def _on_joint_state(self, arm, message):
        with self._lock:
            self._arms[arm]["joint_state"] = message
            self._arms[arm]["joint_at"] = time.monotonic()

    def _on_estop_ack(self, arm, message):
        with self._lock:
            self._arms[arm]["estop_ack"] = bool(message.data)
            self._arms[arm]["estop_ack_at"] = time.monotonic()

    def _on_camera_image(self, message):
        now = time.monotonic()
        if now - self._last_camera_encode < 1.0 / 12.0:
            return
        self._last_camera_encode = now
        try:
            image = self._camera_bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 78]
            )
            if not ok:
                return
        except Exception as exc:
            self.get_logger().warning(f"Cannot encode camera image: {exc}")
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

    def _camera_state(self):
        with self._camera_condition:
            age = (
                time.monotonic() - self._camera_received
                if self._camera_received
                else None
            )
            return {
                "online": age is not None and age < 2.0,
                "topic": self._camera_topic,
                "age_sec": age,
                "width": self._camera_width,
                "height": self._camera_height,
            }

    def _arm_state(self, arm, sample, now):
        joint_online = (
            sample["joint_state"] is not None
            and now - sample["joint_at"] < 1.0
            and bool(sample["joint_state"].position)
        )
        pose_online = sample["pose"] is not None and now - sample["pose_at"] < 1.0
        rcm_online = sample["rcm"] is not None and now - sample["rcm_at"] < 1.0
        rcm_valid = False
        rcm_error_m = None
        tip_radius_m = None
        frame_match = False
        geometry_error = ""
        frame = ""
        if pose_online and rcm_online:
            pose = sample["pose"]
            rcm = sample["rcm"]
            frame = pose.header.frame_id
            frame_match = frame == rcm.header.frame_id
            try:
                q = pose.pose.orientation
                axis = rotate_vector(
                    normalize_quaternion((q.x, q.y, q.z, q.w)), (0.0, 0.0, 1.0)
                )
                p = pose.pose.position
                delta = (rcm.point.x - p.x, rcm.point.y - p.y, rcm.point.z - p.z)
                projection = sum(delta[i] * axis[i] for i in range(3))
                rcm_error_m = math.sqrt(
                    sum((delta[i] - projection * axis[i]) ** 2 for i in range(3))
                )
                tip_radius_m = math.sqrt(sum(value * value for value in delta))
                rcm_valid = (
                    frame_match
                    and rcm_error_m <= 0.002
                    and tip_radius_m <= 1.0
                )
            except (ValueError, OverflowError) as exc:
                rcm_valid = False
                geometry_error = str(exc)
        servo = self._servos.get(arm)
        return {
            "online": joint_online and pose_online and rcm_online,
            "joint_state_online": joint_online,
            "rcm_online": pose_online and rcm_online,
            "rcm_valid": rcm_valid,
            "rcm_error_mm": rcm_error_m * 1000.0 if rcm_error_m is not None else None,
            "tip_radius_m": tip_radius_m,
            "tilt_allowed": tip_radius_m is not None and tip_radius_m >= 0.02,
            "frame_match": frame_match,
            "geometry_error": geometry_error,
            "busy": sample["busy"],
            "frame": frame,
            "driver_message": "关节状态流在线" if joint_online else "等待关节状态流",
            "estop_ack": sample["estop_ack"],
            "servo": servo.state() if servo else {
                "connected": False,
                "port": "",
                "message": "未配置舵机",
                "age_sec": None,
            },
        }

    def state(self):
        now = time.monotonic()
        with self._lock:
            task = self._task_status
            task_age = now - self._task_status_received if task else None
            scene = self._scene
            scene_age = now - self._scene_received if scene else None
            arm_samples = {arm: dict(self._arms[arm]) for arm in self.ARMS}
            estop_latched = self._estop_latched
            estop_time = self._estop_time
            executor_online = self._executor_online

        camera = self._camera_state()
        arms = {
            arm: self._arm_state(arm, arm_samples[arm], now) for arm in self.ARMS
        }
        task_online = task_age is not None and task_age < 1.0
        scene_online = scene_age is not None and scene_age < 1.0
        scene_ready = bool(
            scene_online
            and scene.valid
            and scene.transform_valid
            and scene.obstacles_valid
            and scene.header.frame_id
            and all(
                any(
                    item.class_name == class_name
                    and item.detected
                    and item.position_valid
                    and item.world_position_valid
                    for item in scene.objects
                )
                for class_name in ("liver", "lesion")
            )
        )
        scene_revision = (
            int(scene.obstacle_geometry_revision)
            if scene is not None else None
        )
        for arm in self.ARMS:
            arms[arm]["collision_scene_ready"] = bool(
                scene_revision is not None
                and arm_samples[arm]["collision_scene_revision"] == scene_revision
                and now - arm_samples[arm]["collision_scene_at"] < 1.0
            )
        arms_ready = all(
            arms[arm]["online"]
            and arms[arm]["rcm_valid"]
            and arms[arm]["collision_scene_ready"]
            for arm in self.ARMS
        )
        mission = {
            "online": task_online,
            "state": task.state if task_online else "UNKNOWN",
            "task_id": int(task.task_id) if task_online else 0,
            "step": task.step if task_online else "",
            "active": bool(task.active) if task_online else False,
            "progress": float(task.progress) if task_online else 0.0,
            "message": task.message if task_online else "任务状态机未连接",
            "age_sec": task_age,
        }
        blockers = []
        if estop_latched:
            blockers.append("急停已触发，需检查机械臂并重启 dashboard")
        if not task_online:
            blockers.append("任务状态机未连接")
        elif task.state != "IDLE":
            blockers.append("任务状态机不在待机状态")
        if not executor_online:
            blockers.append("任务执行器未连接")
        if not camera["online"]:
            blockers.append("相机无新图像")
        if not scene_ready:
            blockers.append("感知场景无效或未检测到所需目标")
        if not arms_ready:
            blockers.append("机械臂关节/RCM 状态未就绪")
        can_start = not blockers
        return {
            "camera": camera,
            "mission": mission,
            "systems": {
                "task_manager": task_online,
                "executor": executor_online,
                "perception": scene_ready,
                "arms_ready": arms_ready,
            },
            "scene": {
                "online": scene_online,
                "valid": bool(scene.valid) if scene else False,
                "transform_valid": bool(scene.transform_valid) if scene else False,
                "obstacles_valid": bool(scene.obstacles_valid) if scene else False,
                "frame": scene.header.frame_id if scene else "",
                "age_sec": scene_age,
                "status": scene.status if scene else "等待感知场景",
                "obstacles_status": scene.obstacles_status if scene else "等待障碍物几何",
            },
            "arms": arms,
            "estop": {
                "latched": estop_latched,
                "time": estop_time,
                "acknowledgements": {
                    arm: arms[arm]["estop_ack"] for arm in self.ARMS
                },
            },
            "can_start": can_start,
            "start_blockers": blockers,
        }

    def start_mission(self):
        state = self.state()
        with self._lock:
            if self._estop_latched:
                return False, "急停已锁定，请检查机械臂并重启 dashboard"
            task = self._task_status
            task_age = (
                time.monotonic() - self._task_status_received if task else None
            )
            if task is None or task_age >= 1.0 or task.state != "IDLE":
                return False, "任务状态机不在线或不在待机状态"
            if not state["can_start"]:
                return False, "; ".join(state["start_blockers"])
            message = Bool()
            message.data = True
            self._start_pub.publish(message)
        return True, "开始信号已发送"

    def emergency_stop(self):
        with self._lock:
            self._estop_latched = True
            self._estop_time = time.time()
        cancel = Bool()
        cancel.data = True
        self._cancel_pub.publish(cancel)
        stop = Bool()
        stop.data = True
        driver_stop = Empty()
        estop = Stop()
        estop.state = True
        for arm in self.ARMS:
            self._rcm_stop_pubs[arm].publish(stop)
            self._driver_stop_pubs[arm].publish(driver_stop)
            self._driver_estop_pubs[arm].publish(estop)
        return {
            "accepted": True,
            "message": "已发送任务取消、RCM 停止和机械臂急停命令；请确认控制器反馈",
        }

    def destroy_node(self):
        self._servo_stop.set()
        self._servo_thread.join(timeout=1.0)
        for probe in self._servos.values():
            probe.close()
        return super().destroy_node()


def make_handler(node, page):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *_args):
            return

        def _send(self, status, payload, content_type="application/json; charset=utf-8"):
            body = (
                payload
                if isinstance(payload, bytes)
                else json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
            )
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

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
            if self.path == "/api/start":
                accepted, message = node.start_mission()
                self._send(
                    202 if accepted else 409,
                    {"accepted": accepted, "message": message},
                )
            elif self.path == "/api/estop":
                self._send(202, node.emergency_stop())
            else:
                self._send(404, {"error": "not found"})

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = TaskDashboard()
    server = None
    try:
        package_share = get_package_share_directory("mis_task_manager")
        with open(
            os.path.join(package_share, "web", "task_dashboard.html"), "rb"
        ) as page_file:
            page = page_file.read()
        server = ThreadingHTTPServer(
            (node._host, node._port), make_handler(node, page)
        )
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        node.get_logger().info(
            f"Task dashboard available at http://{node._host}:{node._port}"
        )
        rclpy.spin(node)
    except (OSError, ValueError) as exc:
        node.get_logger().fatal(f"Cannot start task dashboard: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
