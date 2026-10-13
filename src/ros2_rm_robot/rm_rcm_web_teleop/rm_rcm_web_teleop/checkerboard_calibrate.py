#!/usr/bin/env python3
"""Collect robot-tip/checkerboard correspondences and solve T_target_checkerboard.

The node only observes the robot TF tree. Robot motion and contact are manual.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException


def symmetry_variants(grid: np.ndarray) -> list[np.ndarray]:
    variants = []
    for transpose in (False, True):
        base = np.transpose(grid, (1, 0, 2)) if transpose else grid
        for flip_y in (False, True):
            for flip_x in (False, True):
                item = base
                if flip_y:
                    item = item[::-1, :, :]
                if flip_x:
                    item = item[:, ::-1, :]
                variants.append(np.ascontiguousarray(item))
    return variants


def canonicalize_corners(corners: np.ndarray, reference: np.ndarray | None) -> np.ndarray:
    grid = corners.reshape(6, 6, 2)
    if reference is None:
        return np.ascontiguousarray(grid.reshape(-1, 1, 2))
    ref = reference.reshape(6, 6, 2)
    candidates = symmetry_variants(grid)
    aligned = min(candidates, key=lambda candidate: float(np.mean((candidate - ref) ** 2)))
    return np.ascontiguousarray(aligned.reshape(-1, 1, 2))


def board_object_points(square_size_m: float) -> np.ndarray:
    points = np.zeros((36, 3), dtype=np.float64)
    grid = np.mgrid[0:6, 0:6].T.reshape(-1, 2)
    points[:, :2] = grid * square_size_m
    return points


def estimate_checkerboard_camera(corners: np.ndarray, camera_matrix: np.ndarray,
                                 distortion: np.ndarray,
                                 square_size_m: float) -> tuple[np.ndarray, float] | None:
    ok, rvec, tvec = cv2.solvePnP(
        board_object_points(square_size_m), corners.reshape(-1, 2),
        camera_matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None
    rotation_camera_world, _ = cv2.Rodrigues(rvec)
    transform_camera_world = np.eye(4, dtype=np.float64)
    transform_camera_world[:3, :3] = rotation_camera_world
    transform_camera_world[:3, 3] = tvec.reshape(3)
    projected, _ = cv2.projectPoints(
        board_object_points(square_size_m), rvec, tvec, camera_matrix, distortion
    )
    delta = projected.reshape(-1, 2) - corners.reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum(delta * delta, axis=1))))
    return np.linalg.inv(transform_camera_world), rms


def fit_rigid_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("Point arrays must have matching shape (N, 3)")
    if source.shape[0] < 3:
        raise ValueError("At least three point pairs are required")
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    if np.linalg.matrix_rank(source_zero, tol=1e-8) < 2:
        raise ValueError("Calibration points must span a plane; three collinear points are insufficient")
    if np.linalg.matrix_rank(target_zero, tol=1e-5) < 2:
        raise ValueError(
            "Robot tip coordinates do not span a plane. Check that the arm TF/joint states "
            "change while moving between checkerboard corners"
        )
    u, _, vt = np.linalg.svd(source_zero.T @ target_zero)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = target_center - rotation @ source_center
    return transform


def robust_fit(world_points: np.ndarray, base_points: np.ndarray,
               threshold_m: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    if len(world_points) < 3:
        raise ValueError("At least three point pairs are required for a rigid fit")

    def fit_all_samples() -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
        transform = fit_rigid_transform(world_points, base_points)
        predicted = (transform[:3, :3] @ world_points.T).T + transform[:3, 3]
        residuals = np.linalg.norm(predicted - base_points, axis=1)
        return transform, np.ones(len(world_points), dtype=bool), residuals, "all_samples_least_squares"

    keep = np.ones(len(world_points), dtype=bool)
    for _ in range(8):
        if np.count_nonzero(keep) < 3:
            return fit_all_samples()
        transform = fit_rigid_transform(world_points[keep], base_points[keep])
        predicted = (transform[:3, :3] @ world_points.T).T + transform[:3, 3]
        residuals = np.linalg.norm(predicted - base_points, axis=1)
        updated = residuals <= threshold_m
        if np.array_equal(updated, keep):
            break
        if np.count_nonzero(updated) < 3:
            return fit_all_samples()
        keep = updated
    if np.count_nonzero(keep) < 3:
        return fit_all_samples()
    transform = fit_rigid_transform(world_points[keep], base_points[keep])
    predicted = (transform[:3, :3] @ world_points.T).T + transform[:3, 3]
    residuals = np.linalg.norm(predicted - base_points, axis=1)
    keep &= residuals <= threshold_m
    if np.count_nonzero(keep) < 3:
        return fit_all_samples()
    transform = fit_rigid_transform(world_points[keep], base_points[keep])
    predicted = (transform[:3, :3] @ world_points.T).T + transform[:3, 3]
    residuals = np.linalg.norm(predicted - base_points, axis=1)
    return transform, keep, residuals, "robust_inliers"


class CheckerboardCalibrationNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("robot_checkerboard_calibration")
        self.args = args
        self.bridge = CvBridge()
        self.tf_buffers = {arm: Buffer() for arm in ("left", "right")}
        self.tf_subscriptions = []
        for arm in ("left", "right"):
            tf_topic = getattr(args, f"{arm}_tf_topic")
            tf_static_topic = getattr(args, f"{arm}_tf_static_topic")
            self.tf_subscriptions.append(self.create_subscription(
                TFMessage, tf_topic,
                lambda msg, side=arm: self.on_tf(side, msg, static=False), 100,
            ))
            static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.tf_subscriptions.append(self.create_subscription(
                TFMessage, tf_static_topic,
                lambda msg, side=arm: self.on_tf(side, msg, static=True), static_qos,
            ))
        self.image: np.ndarray | None = None
        self.camera_matrix: np.ndarray | None = None
        self.distortion: np.ndarray | None = None
        self.camera_frame = ""
        self.image_size: tuple[int, int] | None = None
        self.reference_corners: np.ndarray | None = None
        self.current_corners: np.ndarray | None = None
        self.current_world_camera: np.ndarray | None = None
        self.current_reprojection_rms: float | None = None
        self.selected_index: int | None = None
        self.selected_world_camera: np.ndarray | None = None
        self.selected_reprojection_rms: float | None = None
        self.samples: dict[str, list[dict[str, object]]] = {"left": [], "right": []}
        self.active_arm = "left"
        self._last_info_shape: tuple[int, int] | None = None
        self.display_scale = 1.5
        self.create_subscription(Image, args.image_topic, self.on_image, 5)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.on_camera_info, 5)
        cv2.namedWindow("Robot-checkerboard calibration", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Robot-checkerboard calibration", 960, 720)
        cv2.setMouseCallback("Robot-checkerboard calibration", self.on_mouse)

    def on_image(self, msg: Image) -> None:
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Could not decode image: {exc}")

    def on_tf(self, arm: str, message: TFMessage, static: bool) -> None:
        buffer = self.tf_buffers[arm]
        for transform in message.transforms:
            if static:
                buffer.set_transform_static(transform, f"{arm}_robot_state_publisher")
            else:
                buffer.set_transform(transform, f"{arm}_robot_state_publisher")

    def on_camera_info(self, msg: CameraInfo) -> None:
        shape = (msg.width, msg.height)
        if self._last_info_shape is not None and self._last_info_shape != shape:
            self.reference_corners = None
        self._last_info_shape = shape
        self.image_size = shape
        self.camera_frame = msg.header.frame_id
        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(msg.d, dtype=np.float64)

    def on_mouse(self, event: int, x: int, y: int, _flags: int, _param: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or self.current_corners is None:
            return
        points = self.current_corners.reshape(-1, 2)
        image_xy = np.array([x, y], dtype=np.float64) / self.display_scale
        nearest = int(np.argmin(np.sum((points - image_xy) ** 2, axis=1)))
        if float(np.linalg.norm(points[nearest] - image_xy)) < 35.0:
            self.selected_index = nearest
            self.selected_world_camera = None
            self.selected_reprojection_rms = None
            if self.current_world_camera is not None:
                self.selected_world_camera = self.current_world_camera.copy()
                self.selected_reprojection_rms = self.current_reprojection_rms

    def read_tip_position(self, arm: str) -> np.ndarray:
        transform = self.tf_buffers[arm].lookup_transform(
            self.args.base_frame, self.args.tool_frame, Time()
        ).transform
        p = transform.translation
        return np.array([p.x, p.y, p.z], dtype=np.float64)

    def capture(self) -> None:
        if self.selected_index is None or self.selected_world_camera is None:
            self.get_logger().warning("Detect the board and click one inner corner first")
            return
        arm = self.active_arm
        try:
            base_point = self.read_tip_position(arm)
        except TransformException as exc:
            self.get_logger().error(
                f"TF {self.args.base_frame} <- {self.args.tool_frame} unavailable for {arm}: {exc}"
            )
            return
        checkerboard_point = board_object_points(
            self.args.square_size_mm / 1000.0
        )[self.selected_index]
        self.samples[arm].append({
            "corner_index": self.selected_index,
            "checkerboard_m": checkerboard_point.tolist(),
            "target_m": base_point.tolist(),
            "T_checkerboard_camera": self.selected_world_camera.tolist(),
            "reprojection_rms_px": self.selected_reprojection_rms,
            "timestamp_unix_s": time.time(),
        })
        self.get_logger().info(
            f"Captured {arm} corner {self.selected_index}: target=({base_point[0]:.5f}, "
            f"{base_point[1]:.5f}, {base_point[2]:.5f}) m; total={len(self.samples[arm])}"
        )

    def save_result(self) -> None:
        insufficient = [arm for arm in ("left", "right") if len(self.samples[arm]) < 3]
        if insufficient:
            counts = ", ".join(f"{arm}={len(self.samples[arm])}" for arm in ("left", "right"))
            self.get_logger().error(f"Capture at least three point pairs per arm ({counts})")
            return
        arm_results = {}
        for arm in ("left", "right"):
            samples = self.samples[arm]
            checkerboard_points = np.asarray(
                [sample["checkerboard_m"] for sample in samples], dtype=np.float64
            )
            target_points = np.asarray(
                [sample["target_m"] for sample in samples], dtype=np.float64
            )
            try:
                transform_target_checkerboard, fit_points, residuals, fit_mode = robust_fit(
                    checkerboard_points, target_points,
                    self.args.outlier_threshold_mm / 1000.0,
                )
            except ValueError as exc:
                self.get_logger().error(f"{arm}: {exc}")
                return
            if fit_mode == "all_samples_least_squares":
                self.get_logger().warning(
                    f"{arm}: robust rejection left fewer than three points; "
                    "using all samples for an approximate least-squares fit"
                )
            checkerboard_camera = np.asarray(
                samples[-1]["T_checkerboard_camera"], dtype=np.float64
            )
            transform_target_camera = transform_target_checkerboard @ checkerboard_camera
            arm_results[arm] = {
                "transform": f"T_{self.args.base_frame}_camera",
                "matrix": transform_target_camera.tolist(),
                "convention": (
                    f"p_{self.args.base_frame} = "
                    f"T_{self.args.base_frame}_camera @ p_camera; column vectors"
                ),
                "T_target_checkerboard": {
                    "matrix": transform_target_checkerboard.tolist(),
                    "convention": (
                        f"p_{self.args.base_frame} = "
                        "T_target_checkerboard @ p_checkerboard; column vectors"
                    ),
                },
                "fit": {
                    "method": fit_mode,
                    "sample_count": len(samples),
                    "fit_point_count": int(np.count_nonzero(fit_points)),
                    "outlier_threshold_mm": self.args.outlier_threshold_mm,
                    "fit_rms_mm": float(
                        np.sqrt(np.mean(residuals[fit_points] ** 2)) * 1000.0
                    ),
                    "fit_max_mm": float(np.max(residuals[fit_points]) * 1000.0),
                    "samples": [
                        {**sample, "residual_mm": float(residuals[i] * 1000.0),
                         "used_in_fit": bool(fit_points[i])}
                        for i, sample in enumerate(samples)
                    ],
                },
                "T_checkerboard_camera": {
                    "matrix": checkerboard_camera.tolist(),
                    "convention": (
                        "p_checkerboard = T_checkerboard_camera @ p_camera; column vectors"
                    ),
                    "reprojection_rms_px": samples[-1]["reprojection_rms_px"],
                },
            }
        output = Path(self.args.output).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        camera_info = {
            "frame_id": self.camera_frame,
            "width": self.image_size[0] if self.image_size else None,
            "height": self.image_size[1] if self.image_size else None,
            "fx": float(self.camera_matrix[0, 0]) if self.camera_matrix is not None else None,
            "fy": float(self.camera_matrix[1, 1]) if self.camera_matrix is not None else None,
            "cx": float(self.camera_matrix[0, 2]) if self.camera_matrix is not None else None,
            "cy": float(self.camera_matrix[1, 2]) if self.camera_matrix is not None else None,
            "distortion": self.distortion.tolist() if self.distortion is not None else [],
        }
        for arm in arm_results.values():
            arm["T_checkerboard_camera"]["camera_info"] = camera_info
        result = {
            "format": "rm_dual_robot_checkerboard_v1",
            "source_frame": self.camera_frame,
            "target_frame": self.args.base_frame,
            "units": "meters",
            "checkerboard": {
                "inner_corners": [6, 6],
                "square_size_mm": self.args.square_size_mm,
                "origin": "first canonicalized inner corner; axes follow detected grid order",
            },
            "arms": arm_results,
        }
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        self.get_logger().info(
            f"Saved {output}; left RMS={arm_results['left']['fit']['fit_rms_mm']:.2f} mm, "
            f"right RMS={arm_results['right']['fit']['fit_rms_mm']:.2f} mm"
        )

    def run(self) -> None:
        self.get_logger().info(
            f"Image={self.args.image_topic}; CameraInfo={self.args.camera_info_topic}; "
            f"TF={self.args.base_frame} <- {self.args.tool_frame}; "
            f"left TF={self.args.left_tf_topic}, right TF={self.args.right_tf_topic}. "
            "1/2=select arm, click corner and touch with tip, c=capture, s=solve/save, q=quit."
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.image is None:
                continue
            shown = cv2.resize(
                self.image, None, fx=self.display_scale, fy=self.display_scale,
                interpolation=cv2.INTER_LINEAR,
            )
            self.current_corners = None
            if self.camera_matrix is not None and self.distortion is not None:
                gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
                found, corners = cv2.findChessboardCornersSB(
                    gray, (6, 6),
                    flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
                )
                if found and corners is not None:
                    canonical = canonicalize_corners(corners, self.reference_corners)
                    if self.reference_corners is None:
                        self.reference_corners = canonical.copy()
                    self.current_corners = canonical
                    pose = estimate_checkerboard_camera(
                        canonical, self.camera_matrix, self.distortion,
                        self.args.square_size_mm / 1000.0,
                    )
                    if pose is not None:
                        self.current_world_camera, self.current_reprojection_rms = pose
                        cv2.drawChessboardCorners(shown, (6, 6), canonical, True)
                        for index, (u, v) in enumerate(canonical.reshape(-1, 2)):
                            center = (int(round(u * self.display_scale)), int(round(v * self.display_scale)))
                            color = (0, 0, 255) if index == self.selected_index else (30, 210, 30)
                            cv2.circle(shown, center, 7, color, -1)
                            cv2.putText(shown, str(index), (center[0] + 5, center[1] - 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
                        status = (
                            f"ARM={self.active_arm.upper()} | "
                            f"board RMS={self.current_reprojection_rms:.2f}px | "
                            f"L={len(self.samples['left'])} R={len(self.samples['right'])}"
                        )
                    else:
                        status = "solvePnP failed"
                else:
                    self.current_world_camera = None
                    status = "checkerboard not found"
            else:
                status = "waiting for CameraInfo"
            cv2.putText(shown, status, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 220, 255), 2, cv2.LINE_AA)
            cv2.putText(shown, "1/2 select arm | click corner, touch with tip, c=capture | s=save | q=quit",
                        (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow("Robot-checkerboard calibration", shown)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("c"):
                self.capture()
            elif key == ord("1"):
                self.active_arm = "left"
            elif key == ord("2"):
                self.active_arm = "right"
            elif key == ord("s"):
                self.save_result()
            elif key in (ord("q"), 27):
                break
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--base-frame", default="world")
    parser.add_argument("--tool-frame", default="SurgicalToolTip")
    parser.add_argument("--left-tf-topic", default="/left_arm/tf")
    parser.add_argument("--left-tf-static-topic", default="/left_arm/tf_static")
    parser.add_argument("--right-tf-topic", default="/right_arm/tf")
    parser.add_argument("--right-tf-static-topic", default="/right_arm/tf_static")
    parser.add_argument("--square-size-mm", type=float, default=20.0)
    parser.add_argument("--outlier-threshold-mm", type=float, default=5.0)
    parser.add_argument("--output", default="data/calibration/robot_checkerboard.json")
    args, ros_args = parser.parse_known_args()
    args.ros_args = ros_args
    if args.square_size_mm <= 0 or args.outlier_threshold_mm <= 0:
        parser.error("square size and outlier threshold must be positive")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init(args=args.ros_args)
    node = CheckerboardCalibrationNode(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
