from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rm_ros_interfaces.msg import PerceptionObject, PerceptionScene
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .segmenter import PhantomLesionSegmenter, PhantomMaterialSegmenter


CLASS_NAMES = ("liver", "kidney", "gallbladder", "bowel", "omentum", "vessel", "lesion")


def mask_quality(mask: np.ndarray, valid_depth_ratio: float) -> float:
    area = int(np.count_nonzero(mask))
    if area == 0:
        return 0.0
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0
    contour = max(contours, key=cv2.contourArea)
    x, y, width, height = cv2.boundingRect(contour)
    fill = float(cv2.contourArea(contour)) / max(1, width * height)
    perimeter = float(cv2.arcLength(contour, True))
    circularity = 4.0 * np.pi * cv2.contourArea(contour) / max(perimeter * perimeter, 1.0)
    shape_quality = float(np.clip(0.5 * fill + 0.5 * min(circularity, 1.0), 0.0, 1.0))
    return float(np.clip(shape_quality * valid_depth_ratio, 0.0, 1.0))


def draw_lesion_annotation(
    image: np.ndarray,
    mask: np.ndarray,
    center_px: tuple[int, int],
    position_camera: np.ndarray | None,
) -> None:
    contours, _ = cv2.findContours(
        mask.astype(np.uint8) * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    if not contours:
        return

    cv2.drawContours(image, contours, -1, (255, 255, 255), 4, cv2.LINE_AA)
    cv2.drawContours(image, contours, -1, (0, 0, 255), 2, cv2.LINE_AA)
    radius = max(4, int(round(np.sqrt(np.count_nonzero(mask) / np.pi))))
    cv2.circle(image, center_px, radius, (255, 255, 255), 4, cv2.LINE_AA)
    cv2.circle(image, center_px, radius, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.drawMarker(image, center_px, (255, 255, 255), cv2.MARKER_CROSS, 20, 4, cv2.LINE_AA)
    cv2.drawMarker(image, center_px, (0, 0, 255), cv2.MARKER_CROSS, 20, 2, cv2.LINE_AA)

    x, y, _, _ = cv2.boundingRect(max(contours, key=cv2.contourArea))
    lines = [f"LESION px=({center_px[0]},{center_px[1]})"]
    if position_camera is not None:
        xyz_mm = np.asarray(position_camera, dtype=np.float64) * 1000.0
        lines.append(f"camera XYZ=({xyz_mm[0]:.0f},{xyz_mm[1]:.0f},{xyz_mm[2]:.0f}) mm")
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 2
    sizes = [cv2.getTextSize(line, font, scale, thickness)[0] for line in lines]
    text_width = max(size[0] for size in sizes)
    text_height = sum(size[1] + 7 for size in sizes)
    image_height, image_width = image.shape[:2]
    text_x = min(max(4, x), max(4, image_width - text_width - 8))
    text_y = y - text_height - 8
    if text_y < 4:
        text_y = min(image_height - text_height - 4, y + 24)
    cv2.rectangle(
        image,
        (text_x - 4, text_y - 4),
        (text_x + text_width + 4, text_y + text_height),
        (0, 0, 0),
        cv2.FILLED,
    )
    baseline = text_y
    for line, (width, height) in zip(lines, sizes):
        baseline += height + 5
        cv2.putText(image, line, (text_x, baseline), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("TF contains an invalid zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class D405PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("d405_perception")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("camera_extrinsics_file", "")
        self.declare_parameter("camera_extrinsics_arm", "left")
        self.declare_parameter("material_profile", "painted")
        self.declare_parameter("lesion_profile", "material")
        self.declare_parameter("min_depth_m", 0.03)
        self.declare_parameter("max_depth_m", 0.70)
        self.declare_parameter("obstacle_margin_m", 0.015)
        self.declare_parameter("obstacle_min_extent_m", 0.04)
        self.declare_parameter("min_valid_depth_ratio", 0.15)
        self.declare_parameter("sync_slop_s", 0.08)
        self.declare_parameter("sync_queue_size", 10)

        self.segmenter = PhantomMaterialSegmenter(
            profile=str(self.get_parameter("material_profile").value)
        )
        lesion_profile = str(self.get_parameter("lesion_profile").value)
        self.lesion_segmenter = (
            None if lesion_profile == "material" else PhantomLesionSegmenter(lesion_profile)
        )
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.camera_extrinsics_arm = str(self.get_parameter("camera_extrinsics_arm").value)
        self.camera_extrinsics = self.load_camera_extrinsics(
            str(self.get_parameter("camera_extrinsics_file").value)
        )
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.obstacle_margin_m = float(self.get_parameter("obstacle_margin_m").value)
        self.obstacle_min_extent_m = float(self.get_parameter("obstacle_min_extent_m").value)
        self.obstacle_update_threshold_m = 0.005
        self.obstacle_geometry_revision = 0
        self._last_obstacle_bounds = {}
        if (
            not np.isfinite(self.obstacle_margin_m)
            or not np.isfinite(self.obstacle_min_extent_m)
            or self.obstacle_margin_m < 0.0
            or self.obstacle_min_extent_m <= 0.0
        ):
            raise ValueError("obstacle geometry margin and minimum extent must be nonnegative/positive")
        self.min_valid_depth_ratio = float(self.get_parameter("min_valid_depth_ratio").value)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scene_pub = self.create_publisher(PerceptionScene, "/perception/scene", 10)
        self.overlay_pub = self.create_publisher(Image, "/perception/overlay", 2)
        self.mask_pubs = {
            name: self.create_publisher(Image, f"/perception/masks/{name}", 2)
            for name in CLASS_NAMES
        }

        qos = rclpy.qos.qos_profile_sensor_data
        color_sub = Subscriber(
            self, Image, str(self.get_parameter("color_topic").value), qos_profile=qos
        )
        depth_sub = Subscriber(
            self, Image, str(self.get_parameter("depth_topic").value), qos_profile=qos
        )
        info_sub = Subscriber(
            self, CameraInfo, str(self.get_parameter("camera_info_topic").value), qos_profile=qos
        )
        self.sync = ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop_s").value),
        )
        self.sync.registerCallback(self.process_frame)
        self.get_logger().info(
            f"Perception ready: color={self.get_parameter('color_topic').value}, "
            f"depth={self.get_parameter('depth_topic').value}, target_frame={self.target_frame}, "
            f"material_profile={self.get_parameter('material_profile').value}, "
            f"lesion_profile={lesion_profile}"
        )

    @staticmethod
    def set_point(target: Point, values: np.ndarray) -> None:
        target.x, target.y, target.z = (float(v) for v in values)

    def lookup_camera_to_target(self, camera_frame: str, stamp) -> tuple[np.ndarray | None, str]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            if self.camera_extrinsics is not None:
                source_frame, matrix = self.camera_extrinsics
                if source_frame == camera_frame:
                    return matrix[:3, :], ""
            return None, str(exc)
        t = transform.transform.translation
        q = transform.transform.rotation
        matrix = quaternion_matrix(q.x, q.y, q.z, q.w)
        return np.column_stack((matrix, np.array([t.x, t.y, t.z]))), ""

    def load_camera_extrinsics(self, filename: str) -> tuple[str, np.ndarray] | None:
        if not filename:
            return None
        try:
            with Path(filename).expanduser().open("r", encoding="utf-8") as stream:
                document = json.load(stream)
            if document.get("target_frame") != self.target_frame:
                raise ValueError(
                    f"target_frame is {document.get('target_frame')!r}, expected {self.target_frame!r}"
                )
            if document.get("units") != "meters":
                raise ValueError("units must be 'meters'")
            source_frame = document.get("source_frame")
            if document.get("format") == "rm_dual_robot_checkerboard_v1":
                arms = document.get("arms")
                arm_result = arms.get(self.camera_extrinsics_arm) if isinstance(arms, dict) else None
                if not isinstance(arm_result, dict):
                    raise ValueError(
                        f"calibration for arm {self.camera_extrinsics_arm!r} is missing"
                    )
                matrix_value = arm_result.get("matrix")
            else:
                matrix_value = document.get("matrix")
            matrix = np.asarray(matrix_value, dtype=np.float64)
            if not isinstance(source_frame, str) or not source_frame:
                raise ValueError("source_frame is missing")
            if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
                raise ValueError("matrix must be a finite 4x4 transform")
            if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-6):
                raise ValueError("matrix must have homogeneous bottom row [0, 0, 0, 1]")
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            self.get_logger().error(f"Invalid camera extrinsics file '{filename}': {exc}")
            return None
        self.get_logger().info(
            f"Loaded {self.camera_extrinsics_arm} camera extrinsics: "
            f"{self.target_frame} <- {source_frame} from {filename}"
        )
        return source_frame, matrix

    def process_frame(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth_raw.shape[:2] != color.shape[:2]:
                raise ValueError("Depth image must be aligned to the color image and have matching dimensions")
            if info_msg.width != color.shape[1] or info_msg.height != color.shape[0]:
                raise ValueError("CameraInfo dimensions do not match the RGB image")
            if depth_raw.dtype == np.uint16:
                depth_m = depth_raw.astype(np.float32) * 0.001
            else:
                depth_m = depth_raw.astype(np.float32)
            depth_m[~np.isfinite(depth_m)] = 0.0
            fx, fy = float(info_msg.k[0]), float(info_msg.k[4])
            cx, cy = float(info_msg.k[2]), float(info_msg.k[5])
            if fx <= 0.0 or fy <= 0.0:
                raise ValueError("CameraInfo has invalid focal lengths")

            masks = self.segmenter.predict_all(color)
            if self.lesion_segmenter is not None:
                lesion_masks = self.lesion_segmenter.predict(color)
                lesion = lesion_masks[0] if lesion_masks else np.zeros(color.shape[:2], dtype=bool)
                masks["lesion"] = lesion
                masks["liver"] = masks["liver"] & ~lesion
            camera_frame = info_msg.header.frame_id or color_msg.header.frame_id
            t_target_camera, tf_error = self.lookup_camera_to_target(camera_frame, color_msg.header.stamp)
            scene = PerceptionScene()
            scene.header.stamp = color_msg.header.stamp
            scene.header.frame_id = self.target_frame if t_target_camera is not None else camera_frame
            scene.valid = True
            scene.transform_valid = t_target_camera is not None
            scene.status = "ok" if scene.transform_valid else f"camera-frame only: {tf_error}"
            scene.obstacles_valid = scene.transform_valid
            missing_obstacle_geometry = []
            obstacle_bounds = {}

            overlay = color.copy()
            lesion_mask = None
            lesion_center_px = None
            lesion_position_camera = None
            colors = {
                "liver": (40, 80, 220), "kidney": (180, 80, 200),
                "gallbladder": (40, 220, 40), "bowel": (40, 180, 240),
                "omentum": (0, 220, 220), "vessel": (220, 40, 40), "lesion": (0, 0, 255),
            }
            for name in CLASS_NAMES:
                raw_mask = masks.get(name)
                mask = np.zeros(color.shape[:2], dtype=bool) if raw_mask is None else np.asarray(raw_mask, dtype=bool)
                if mask.shape != color.shape[:2]:
                    raise ValueError(f"Segmenter mask '{name}' dimensions do not match RGB image")
                mask_msg = self.bridge.cv2_to_imgmsg(mask.astype(np.uint8) * 255, encoding="mono8")
                mask_msg.header = color_msg.header
                self.mask_pubs[name].publish(mask_msg)
                if np.any(mask):
                    overlay[mask] = (0.55 * overlay[mask] + 0.45 * np.asarray(colors[name])).astype(np.uint8)

                item = PerceptionObject()
                item.class_name = name
                item.detected = bool(np.any(mask))
                item.mask_pixels = int(np.count_nonzero(mask))
                if name == "lesion" and item.detected:
                    ys, xs = np.nonzero(mask)
                    lesion_mask = mask
                    lesion_center_px = (
                        int(round(float(np.mean(xs)))),
                        int(round(float(np.mean(ys)))),
                    )
                if item.detected:
                    valid = mask & np.isfinite(depth_m) & (depth_m >= self.min_depth_m) & (depth_m <= self.max_depth_m)
                    valid_count = int(np.count_nonzero(valid))
                    item.valid_depth_pixels = valid_count
                    item.valid_depth_ratio = float(valid_count / max(1, item.mask_pixels))
                    if valid_count:
                        z = depth_m[valid]
                        z_median = float(np.median(z))
                        item.depth_median_m = z_median
                        item.depth_mad_m = float(np.median(np.abs(z - z_median)))
                        stable = valid & (np.abs(depth_m - z_median) <= max(0.004, 3.0 * item.depth_mad_m))
                        py, px = np.nonzero(stable)
                        if px.size >= 20 and item.valid_depth_ratio >= self.min_valid_depth_ratio:
                            z_points = depth_m[py, px]
                            points_camera = np.column_stack(
                                ((px - cx) * z_points / fx, (py - cy) * z_points / fy, z_points)
                            )
                            point_camera = np.median(points_camera, axis=0)
                            self.set_point(item.position_camera, point_camera)
                            item.position_valid = True
                            if name == "lesion":
                                lesion_center_px = (
                                    int(round(point_camera[0] * fx / point_camera[2] + cx)),
                                    int(round(point_camera[1] * fy / point_camera[2] + cy)),
                                )
                                lesion_position_camera = point_camera
                            if t_target_camera is not None:
                                point_target = t_target_camera[:, :3] @ point_camera + t_target_camera[:, 3]
                                self.set_point(item.position_world, point_target)
                                item.world_position_valid = True
                                if name not in ("liver", "lesion") and len(points_camera) >= 10:
                                    points_target = (
                                        t_target_camera[:, :3] @ points_camera.T
                                    ).T + t_target_camera[:, 3]
                                    bounds_min = np.percentile(points_target, 5.0, axis=0)
                                    bounds_max = np.percentile(points_target, 95.0, axis=0)
                                    center = 0.5 * (bounds_min + bounds_max)
                                    extent = np.maximum(
                                        bounds_max - bounds_min + 2.0 * self.obstacle_margin_m,
                                        self.obstacle_min_extent_m,
                                    )
                                    bounds_min = center - 0.5 * extent
                                    bounds_max = center + 0.5 * extent
                                    self.set_point(item.obstacle_bounds_min_world, bounds_min)
                                    self.set_point(item.obstacle_bounds_max_world, bounds_max)
                                    item.obstacle_geometry_valid = True
                                    obstacle_bounds[name] = tuple(
                                        float(value) for value in (*bounds_min, *bounds_max)
                                    )
                    item.quality_score = mask_quality(mask, item.valid_depth_ratio)
                    if name not in ("liver", "lesion") and not item.obstacle_geometry_valid:
                        scene.obstacles_valid = False
                        missing_obstacle_geometry.append(name)
                scene.objects.append(item)

            if lesion_mask is not None and lesion_center_px is not None:
                draw_lesion_annotation(
                    overlay, lesion_mask, lesion_center_px, lesion_position_camera
                )
            scene.obstacles_status = (
                "ok" if scene.obstacles_valid else
                "missing world-frame obstacle geometry: " + ", ".join(sorted(set(missing_obstacle_geometry)))
            )
            if not scene.transform_valid:
                scene.obstacles_status = "camera-to-target transform unavailable"
            if scene.obstacles_valid:
                changed = obstacle_bounds.keys() != self._last_obstacle_bounds.keys()
                if not changed:
                    threshold = self.obstacle_update_threshold_m
                    changed = any(
                        abs(value - self._last_obstacle_bounds[name][index]) >= threshold
                        for name, bounds in obstacle_bounds.items()
                        for index, value in enumerate(bounds)
                    )
                if changed:
                    self.obstacle_geometry_revision += 1
                    self._last_obstacle_bounds = obstacle_bounds
            scene.obstacle_geometry_revision = self.obstacle_geometry_revision

            self.scene_pub.publish(scene)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header = color_msg.header
            self.overlay_pub.publish(overlay_msg)
        except Exception as exc:  # Keep a bad frame from terminating the perception process.
            self.get_logger().error(f"Failed to process synchronized RGB-D frame: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = D405PerceptionNode()
        rclpy.spin(node)
    except RuntimeError as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f"d405_perception: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
