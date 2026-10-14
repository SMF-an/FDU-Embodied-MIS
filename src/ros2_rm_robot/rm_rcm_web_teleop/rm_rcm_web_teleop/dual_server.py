#!/usr/bin/env python3
import json
import math
import os
import base64
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .server import clamp, multiply_quaternions, normalize_quaternion, rotate_vector


class ServoController:
    TORQUE_ENABLE = 40

    def __init__(self, settings):
        self.settings = settings
        self.lock = threading.Lock()
        self.port = None
        self.port_open = False
        self.servo = None
        self.connected = False
        self.message = "尚未连接"
        try:
            from scservo_sdk import PortHandler, scscl

            self.port = PortHandler(str(settings["port"]))
            self.servo = scscl(self.port)
            if not self.port.openPort():
                raise RuntimeError(f"无法打开串口 {settings['port']}")
            self.port_open = True
            if not self.port.setBaudRate(int(settings["baud_rate"])):
                raise RuntimeError(f"无法设置波特率 {settings['baud_rate']}")
            servo_id = int(settings["servo_id"])
            model, result, error = self.servo.ping(servo_id)
            if result != 0 or error != 0:
                raise RuntimeError(
                    f"ping 失败，型号={model}，通信结果={result}，舵机错误={error}"
                )
            result, error = self.servo.write1ByteTxRx(servo_id, self.TORQUE_ENABLE, 1)
            if result != 0 or error != 0:
                raise RuntimeError(f"力矩使能失败 result={result} error={error}")
            self.connected = True
            self.message = f"已连接：{settings['port']}，ID={servo_id}"
        except Exception as error:
            self.message = str(error)
            if self.port and self.port_open:
                try:
                    self.port.closePort()
                except Exception:
                    pass
                self.port_open = False

    def set_grip(self, action):
        if not self.connected:
            return False, self.message
        position_key = "close_position" if action == "close" else "open_position"
        target = int(self.settings[position_key])
        target = max(
            int(self.settings["min_position"]),
            min(int(self.settings["max_position"]), target),
        )
        try:
            with self.lock:
                result, error = self.servo.WritePos(
                    int(self.settings["servo_id"]),
                    target,
                    0,
                    int(self.settings["speed"]),
                )
        except Exception as error:
            self.connected = False
            self.message = str(error)
            return False, self.message
        if result != 0 or error != 0:
            self.connected = result == 0
            self.message = f"舵机命令失败 result={result} error={error}"
            return False, self.message
        return True, f"目标位置 {target} 已发送"

    def close(self):
        if self.port is None:
            return
        try:
            if self.servo is not None and self.port_open:
                with self.lock:
                    self.servo.write1ByteTxRx(
                        int(self.settings["servo_id"]), self.TORQUE_ENABLE, 0
                    )
        except Exception:
            pass
        self.connected = False
        try:
            self.port.closePort()
            self.port_open = False
        except Exception:
            pass


class DualRcmWebTeleop(Node):
    SIDES = ("left", "right")

    def __init__(self):
        super().__init__("dual_rcm_web_teleop")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("agent_enabled", False)
        self.declare_parameter("agent_max_action_hz", 2.0)
        self.declare_parameter("step_angle_deg", 2.0)
        self.declare_parameter("step_roll_deg", 2.0)
        self.declare_parameter("step_insertion_m", 0.005)
        self.declare_parameter("large_step_multiplier", 2.0)
        self.declare_parameter("camera_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_jpeg_quality", 78)
        self.declare_parameter("data_directory", "")
        self.declare_parameter("calibration_file", "")
        self._host = self.get_parameter("host").value
        self._port = int(self.get_parameter("port").value)
        self._agent_enabled = bool(self.get_parameter("agent_enabled").value)
        agent_max_action_hz = float(self.get_parameter("agent_max_action_hz").value)
        if not math.isfinite(agent_max_action_hz) or agent_max_action_hz <= 0.0:
            raise ValueError("agent_max_action_hz must be positive")
        self._agent_max_action_period = 1.0 / agent_max_action_hz
        self._step_angle = math.radians(float(self.get_parameter("step_angle_deg").value))
        self._step_roll = math.radians(float(self.get_parameter("step_roll_deg").value))
        self._step_insertion = float(self.get_parameter("step_insertion_m").value)
        self._large_step_multiplier = float(self.get_parameter("large_step_multiplier").value)
        self._motion_mode = "fine"
        self._camera_topic = self.get_parameter("camera_topic").value
        self._camera_jpeg_quality = int(self.get_parameter("camera_jpeg_quality").value)
        self._mode = "simulation" if self.get_parameter("use_sim_time").value else "real"
        if not 1 <= self._port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if not 0.0 < self._step_angle <= math.radians(10.0):
            raise ValueError("step_angle_deg must be in (0, 10]")
        if not 0.0 < self._step_roll <= math.radians(10.0):
            raise ValueError("step_roll_deg must be in (0, 10]")
        if not 0.0 < self._step_insertion <= 0.02:
            raise ValueError("step_insertion_m must be in (0, 0.02]")
        if not 1.0 < self._large_step_multiplier <= 5.0:
            raise ValueError("large_step_multiplier must be in (1, 5]")
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
        self._joints = {side: None for side in self.SIDES}
        self._joint_received = {side: 0.0 for side in self.SIDES}
        self._rcms = {side: None for side in self.SIDES}
        self._rcm_received = {side: 0.0 for side in self.SIDES}
        self._busy = {side: False for side in self.SIDES}
        self._active_arm = None
        self._active_gripper = None
        self._agent_last_action = 0.0
        self._active_since = 0.0
        self._active_saw_busy = False
        self._teach_recording = False
        self._teach_started = 0.0
        self._teach_actions = []
        self._teach_start_poses = {}
        self._teach_start_joints = {}
        self._teach_lock = threading.Lock()
        data_directory = self.get_parameter("data_directory").value
        if not data_directory:
            package_share = get_package_share_directory("rm_rcm_web_teleop")
            workspace_root = os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.dirname(package_share)))
            )
            data_directory = os.path.join(workspace_root, "data")
        data_directory = os.path.abspath(os.path.expanduser(data_directory))
        self._teach_directory = os.path.join(data_directory, "teachings")
        calibration_file = self.get_parameter("calibration_file").value
        self._calibration_path = os.path.abspath(os.path.expanduser(
            calibration_file or os.path.join(data_directory, "calibration", "dual_rcm_points.json")
        ))
        self._target_pubs = {}
        self._stop_pubs = {}
        config_path = os.path.join(
            get_package_share_directory("rm_rcm_web_teleop"), "config", "scs215.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as stream:
            servo_config = yaml.safe_load(stream)
        self._grippers = {
            side: ServoController(servo_config["servos"][side])
            for side in self.SIDES
        }
        self._gripper_locks = {side: threading.Lock() for side in self.SIDES}
        self._camera_sub = self.create_subscription(
            Image,
            self._camera_topic,
            self._on_camera_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"从 {self._calibration_path} 读取 RCM 标定；示教轨迹保存到 {self._teach_directory}"
        )

        for side in self.SIDES:
            gripper = self._grippers[side]
            if gripper.connected:
                self.get_logger().info(f"{side} SCS215 {gripper.message}")
            else:
                self.get_logger().error(f"{side} SCS215 未连接：{gripper.message}")
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
                JointState,
                f"/{side}_arm/joint_states",
                lambda msg, arm=side: self._on_joints(arm, msg),
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

    def _on_joints(self, side, message):
        with self._lock:
            self._joints[side] = message
            self._joint_received[side] = time.monotonic()

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

    def agent_observation(self):
        """Return one bounded observation for an external vision agent."""
        with self._camera_condition:
            age = time.monotonic() - self._camera_received if self._camera_received else None
            jpeg = self._camera_jpeg
            sequence = self._camera_sequence
        observation = self.state()
        observation["agent"] = {
            "enabled": self._agent_enabled,
            "action_space": {
                "tilt_x": "[-1,1], one fine RCM tilt step",
                "tilt_y": "[-1,1], one fine RCM tilt step",
                "insertion": "[-1,1], one fine insertion step",
                "roll": "[-1,1], one fine axial roll step",
                "gripper": ["hold", "open", "close"],
            },
            "max_action_hz": 1.0 / self._agent_max_action_period,
        }
        observation["image"] = {
            "available": jpeg is not None and age is not None and age < 2.0,
            "age_sec": age,
            "sequence": sequence,
            "mime_type": "image/jpeg",
            "base64": base64.b64encode(jpeg).decode("ascii") if jpeg else None,
        }
        return observation

    def agent_action(self, body):
        if not self._agent_enabled:
            return 403, {"accepted": False, "error": "agent control is disabled"}
        now = time.monotonic()
        if now - self._agent_last_action < self._agent_max_action_period:
            return 429, {"accepted": False, "error": "agent action rate limit exceeded"}
        if not isinstance(body, dict):
            return 400, {"accepted": False, "error": "JSON object required"}
        confidence = body.get("confidence", 1.0)
        try:
            confidence = float(confidence)
        except (TypeError, ValueError):
            return 400, {"accepted": False, "error": "confidence must be a number"}
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            return 400, {"accepted": False, "error": "confidence must be in [0, 1]"}
        if confidence < 0.5:
            return 409, {"accepted": False, "error": "confidence is below 0.5"}
        arm = body.get("arm")
        if arm not in self.SIDES:
            return 400, {"accepted": False, "error": "arm must be left or right"}
        gripper = body.get("gripper", "hold")
        if gripper not in ("hold", "open", "close"):
            return 400, {"accepted": False, "error": "gripper must be hold, open, or close"}
        values = {key: body.get(key, 0.0) for key in ("tilt_x", "tilt_y", "insertion", "roll")}
        try:
            values = {key: float(value) for key, value in values.items()}
        except (TypeError, ValueError):
            return 400, {"accepted": False, "error": "action values must be numbers"}
        if not all(math.isfinite(value) and -1.0 <= value <= 1.0 for value in values.values()):
            return 400, {"accepted": False, "error": "action values must be finite and in [-1, 1]"}
        has_motion = max(abs(value) for value in values.values()) >= 0.05
        if has_motion and gripper != "hold":
            return 409, {"accepted": False, "error": "send motion and gripper actions separately"}
        if not has_motion and gripper == "hold":
            return 400, {"accepted": False, "error": "action is empty"}

        status, result = 202, {"accepted": True, "arm": arm}
        if has_motion:
            status, result = self.jog({"arm": arm, **values})
        if status < 300 and gripper != "hold":
            grip_status, grip_result = self.control_gripper(arm, gripper)
            if grip_status >= 300:
                return grip_status, {"accepted": False, "motion": result, "gripper": grip_result}
            result["gripper"] = grip_result
        if status < 300:
            self._agent_last_action = now
            result.update({"confidence": confidence, "accepted": True})
        return status, result

    def _clear_active(self):
        self._active_arm = None
        self._active_since = 0.0
        self._active_saw_busy = False

    def _load_calibration_points(self):
        try:
            with open(self._calibration_path, "r", encoding="utf-8") as stream:
                document = json.load(stream)
            if document.get("format") != "rm_dual_rcm_points_v2":
                return {}
            arms = document.get("arms", {})
            return {
                side: point
                for side in self.SIDES
                if isinstance((point := arms.get(side)), dict)
                and point.get("frame_id") == "world"
                and all(
                    isinstance(point.get(axis), (int, float))
                    and math.isfinite(point[axis])
                    for axis in ("x", "y", "z")
                )
            }
        except (OSError, ValueError, AttributeError):
            return {}

    def state(self):
        calibration_points = self._load_calibration_points()
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
                has_state = (
                    pose is not None
                    and rcm is not None
                    and now - self._pose_received[side] < 1.0
                    and now - self._rcm_received[side] < 1.0
                )
                rcm_valid = False
                rcm_initializing = False
                if has_state:
                    p = pose.pose.position
                    q = pose.pose.orientation
                    try:
                        axis = rotate_vector(
                            normalize_quaternion((q.x, q.y, q.z, q.w)), (0.0, 0.0, 1.0)
                        )
                        delta = (rcm.point.x - p.x, rcm.point.y - p.y, rcm.point.z - p.z)
                        offset = sum(delta[i] * axis[i] for i in range(3))
                        error = math.sqrt(sum((delta[i] - offset * axis[i]) ** 2 for i in range(3)))
                        radius = math.sqrt(sum(v * v for v in delta))
                        rcm_valid = error <= 0.002 and 0.02 <= radius <= 1.0
                        rcm_initializing = error <= 0.002 and radius < 0.02
                    except ValueError:
                        pass
                ready = has_state and (rcm_valid or rcm_initializing)
                busy = self._busy[side] or self._active_arm == side
                pose_available = (
                    pose is not None and now - self._pose_received[side] < 1.0
                )
                result = {
                    "ready": ready, "busy": busy,
                    "rcm_valid": rcm_valid, "pose_available": pose_available,
                    "rcm_initializing": rcm_initializing,
                    "gripper_ready": self._grippers[side].connected,
                    "gripper_message": self._grippers[side].message,
                }
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
                    result["message"] = (
                        f"{side} 臂当前位姿不满足 RCM，请重新标定"
                        if has_state and not rcm_valid
                        else f"等待 {side} 臂 RCM {mode_name}状态"
                    )
                arms[side] = result
            with self._teach_lock:
                teaching = {
                    "recording": self._teach_recording,
                    "action_count": len(self._teach_actions),
                    "elapsed_sec": (
                        time.monotonic() - self._teach_started
                        if self._teach_recording else 0.0
                    ),
                }
            return {
                "mode": self._mode,
                "motion_mode": self._motion_mode,
                "step_angle_deg": math.degrees(self._step_angle) * self._motion_multiplier(),
                "step_roll_deg": math.degrees(self._step_roll) * self._motion_multiplier(),
                "step_insertion_mm": self._step_insertion * 1000.0 * self._motion_multiplier(),
                "locked_by": self._active_arm,
                "gripper_busy": self._active_gripper,
                "camera": self.camera_state(),
                "arms": arms,
                "teaching": teaching,
                "calibration_points": calibration_points,
                "calibration_file": self._calibration_path,
            }

    def _motion_multiplier(self):
        return self._large_step_multiplier if self._motion_mode == "large" else 1.0

    def set_motion_mode(self, mode):
        if mode not in ("fine", "large"):
            return False, "mode must be fine or large"
        with self._lock:
            if self._active_arm is not None or self._active_gripper is not None or any(self._busy.values()):
                return False, "请等待当前运动完成后再切换步长模式"
            self._motion_mode = mode
        return True, "精细调整" if mode == "fine" else "大步长"

    def teaching_start(self):
        now = time.monotonic()
        with self._lock:
            with self._teach_lock:
                if self._teach_recording:
                    return False, "already recording"
            if self._active_arm is not None or self._active_gripper is not None or any(self._busy.values()):
                return False, "请等待双臂和舵机动作完成后再开始记录"
            if any(
                self._poses[side] is None or now - self._pose_received[side] > 1.0
                for side in self.SIDES
            ):
                return False, "等待左右臂最新位姿后再开始记录"
            if any(
                self._joints[side] is None or now - self._joint_received[side] > 1.0
                for side in self.SIDES
            ):
                return False, "等待左右臂最新关节状态后再开始记录"
            start_poses = {
                side: self._pose_to_document(self._poses[side])
                for side in self.SIDES
            }
            start_joints = {
                side: self._joint_to_document(self._joints[side])
                for side in self.SIDES
            }
            with self._teach_lock:
                self._teach_recording = True
                self._teach_started = time.monotonic()
                self._teach_actions = []
                self._teach_start_poses = start_poses
                self._teach_start_joints = start_joints
        return True, "recording started"

    @staticmethod
    def _pose_to_document(pose):
        p = pose.pose.position
        q = pose.pose.orientation
        return {
            "frame_id": pose.header.frame_id,
            "position": {"x": p.x, "y": p.y, "z": p.z},
            "orientation": {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
        }

    @staticmethod
    def _joint_to_document(joints):
        return {
            "names": list(joints.name),
            "positions": [float(value) for value in joints.position],
        }

    def teaching_stop(self):
        with self._teach_lock:
            if not self._teach_recording:
                return None, "not recording"
            actions = list(self._teach_actions)
            self._teach_recording = False
        if not actions:
            return None, "no accepted actions"
        os.makedirs(self._teach_directory, mode=0o700, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = os.path.join(self._teach_directory, f"dual_rcm_{stamp}.json")
        document = {
            "format": "rm_rcm_teaching_v2",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "step_angle_deg": math.degrees(self._step_angle),
            "step_roll_deg": math.degrees(self._step_roll),
            "step_insertion_m": self._step_insertion,
            "start_poses": self._teach_start_poses,
            "start_joints": self._teach_start_joints,
            "actions": actions,
            "duration_sec": actions[-1]["time_sec"] if actions else 0.0,
        }
        temporary = f"{path}.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
        os.replace(temporary, path)
        return path, f"saved {len(actions)} actions"

    def teaching_record_action(self, side, target):
        with self._teach_lock:
            if not self._teach_recording:
                return
            message = target.pose
            self._teach_actions.append(
                {
                    "time_sec": round(time.monotonic() - self._teach_started, 4),
                    "arm": side,
                    "frame_id": target.header.frame_id,
                    "position": {
                        "x": message.position.x,
                        "y": message.position.y,
                        "z": message.position.z,
                    },
                    "orientation": {
                        "x": message.orientation.x,
                        "y": message.orientation.y,
                        "z": message.orientation.z,
                        "w": message.orientation.w,
                    },
                }
            )

    def teaching_record_gripper(self, side, action):
        with self._teach_lock:
            if not self._teach_recording:
                return
            self._teach_actions.append(
                {
                    "time_sec": round(time.monotonic() - self._teach_started, 4),
                    "type": "gripper",
                    "arm": side,
                    "action": action,
                }
            )

    def jog(self, body):
        side = body.get("arm")
        if side not in self.SIDES:
            return 400, {"error": "arm must be left or right"}
        try:
            tilt_x = float(body.get("tilt_x", 0.0))
            tilt_y = float(body.get("tilt_y", 0.0))
            insertion = float(body.get("insertion", 0.0))
            roll = float(body.get("roll", 0.0))
        except (TypeError, ValueError):
            return 400, {"error": "joystick values must be numbers"}
        if not all(math.isfinite(v) for v in (tilt_x, tilt_y, insertion, roll)):
            return 400, {"error": "joystick values must be finite"}
        tilt_x = clamp(tilt_x, -1.0, 1.0)
        tilt_y = clamp(tilt_y, -1.0, 1.0)
        insertion = clamp(insertion, -1.0, 1.0)
        roll = clamp(roll, -1.0, 1.0)
        if max(abs(tilt_x), abs(tilt_y), abs(insertion), abs(roll)) < 0.05:
            return 400, {"error": "no joystick input"}

        with self._lock:
            now = time.monotonic()
            if self._active_arm is not None or self._active_gripper is not None or any(self._busy.values()):
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
            delta_to_rcm = (rcm.point.x - p.x, rcm.point.y - p.y, rcm.point.z - p.z)
            axial = sum(delta_to_rcm[i] * current_axis[i] for i in range(3))
            rcm_error = math.sqrt(sum((delta_to_rcm[i] - axial * current_axis[i]) ** 2 for i in range(3)))
            current_radius = math.sqrt(sum(value * value for value in delta_to_rcm))
            if rcm_error > 0.002:
                return 409, {"error": f"当前姿态偏离 RCM 轴线 {rcm_error * 1000:.1f} mm，请先重新标定"}
            initializing = current_radius < 0.02
            multiplier = self._motion_multiplier()
            if initializing and (abs(tilt_x) >= 0.05 or abs(tilt_y) >= 0.05 or abs(roll) >= 0.05):
                return 409, {"error": "尖端距 RCM 不足 20 mm，初始化阶段只允许沿器具轴线进退"}
            if initializing:
                q_delta = (0.0, 0.0, 0.0, 1.0)
                target_axis = current_axis
            else:
                qx = (
                    math.sin(-tilt_y * self._step_angle * multiplier / 2.0),
                    0.0,
                    0.0,
                    math.cos(tilt_y * self._step_angle * multiplier / 2.0),
                )
                qy = (
                    0.0,
                    math.sin(tilt_x * self._step_angle * multiplier / 2.0),
                    0.0,
                    math.cos(tilt_x * self._step_angle * multiplier / 2.0),
                )
                q_delta = multiply_quaternions(qx, qy)
                target_axis = rotate_vector(q_delta, current_axis)
            radius = current_radius + insertion * self._step_insertion * multiplier
            min_radius = 1e-4 if initializing else 0.02
            if not min_radius < radius <= 1.0:
                return 400, {"error": "requested insertion is outside the supported range"}

            target = PoseStamped()
            target.header = pose.header
            target.header.stamp = self.get_clock().now().to_msg()
            target.pose.position.x = rcm.point.x + radius * target_axis[0]
            target.pose.position.y = rcm.point.y + radius * target_axis[1]
            target.pose.position.z = rcm.point.z + radius * target_axis[2]
            q_target = multiply_quaternions(q_delta, q_current)
            q_roll = (
                0.0,
                0.0,
                    math.sin(roll * self._step_roll * multiplier / 2.0),
                math.cos(roll * self._step_roll * multiplier / 2.0),
            )
            q_target = normalize_quaternion(multiply_quaternions(q_target, q_roll))
            target.pose.orientation.x = q_target[0]
            target.pose.orientation.y = q_target[1]
            target.pose.orientation.z = q_target[2]
            target.pose.orientation.w = q_target[3]
            self._active_arm = side
            self._active_since = now
            self._active_saw_busy = False

        self._target_pubs[side].publish(target)
        self.teaching_record_action(side, target)
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

    def control_gripper(self, side, action):
        if side not in self.SIDES or action not in ("open", "close"):
            return 400, {"error": "arm/action must be left or right and open or close"}
        lock = self._gripper_locks[side]
        if not lock.acquire(blocking=False):
            return 409, {"error": "该侧舵机命令正在执行"}
        with self._lock:
            if self._active_arm is not None or self._active_gripper is not None or any(self._busy.values()):
                lock.release()
                return 409, {"error": "请等待当前机械臂或舵机动作结束"}
            self._active_gripper = side
        try:
            success, message = self._grippers[side].set_grip(action)
            if not success:
                return 503, {"error": message}
            self.teaching_record_gripper(side, action)
            return 200, {"success": True, "message": message}
        finally:
            lock.release()
            with self._lock:
                self._active_gripper = None

    def destroy_node(self):
        for gripper in self._grippers.values():
            gripper.close()
        return super().destroy_node()


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
            elif self.path == "/api/agent/observe":
                self._send(200, node.agent_observation())
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
            if self.path not in (
                "/api/jog", "/api/stop", "/api/teaching/start", "/api/teaching/stop",
                "/api/motion-mode",
                "/api/gripper/open", "/api/gripper/close",
                "/api/agent/action",
            ):
                self._send(404, {"error": "not found"})
                return
            if self.path == "/api/teaching/start":
                accepted, message = node.teaching_start()
                self._send(202 if accepted else 409, {"accepted": accepted, "message": message})
                return
            if self.path == "/api/teaching/stop":
                path, message = node.teaching_stop()
                if path is None:
                    self._send(409, {"error": message})
                else:
                    self._send(200, {"saved": True, "path": path, "message": message})
                return
            try:
                body = self._body()
            except ValueError as error:
                code = 413 if str(error) == "request too large" else 400
                self._send(code, {"error": str(error)})
                return
            if self.path == "/api/agent/action":
                status, result = node.agent_action(body)
                self._send(status, result)
                return
            if self.path == "/api/motion-mode":
                accepted, message = node.set_motion_mode(body.get("mode"))
                self._send(200 if accepted else 409, {
                    "accepted": accepted,
                    "mode": body.get("mode"),
                    "message": message,
                })
                return
            if self.path == "/api/stop":
                if not node.stop(body.get("arm")):
                    self._send(409, {"error": "another arm is moving"})
                    return
                self._send(202, {"accepted": True})
                return
            if self.path.startswith("/api/gripper/"):
                status, result = node.control_gripper(
                    body.get("arm"), self.path.rsplit("/", 1)[-1]
                )
                self._send(status, result)
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
