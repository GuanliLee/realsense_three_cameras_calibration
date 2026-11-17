#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


CAMERAS = {
    "cam_high": {
        "rgb": "/cam_high/color/image_raw/compressed",
        "depth": "/cam_high/aligned_depth_to_color/image_raw",
        "camera_info": "/cam_high/color/camera_info",
    },
    "cam_left": {
        "rgb": "/cam_left/color/image_raw/compressed",
        "depth": "/cam_left/aligned_depth_to_color/image_raw",
        "camera_info": "/cam_left/color/camera_info",
        "pose": "/left/gripper_end_pos",
    },
    "cam_right": {
        "rgb": "/cam_right/color/image_raw/compressed",
        "depth": "/cam_right/aligned_depth_to_color/image_raw",
        "camera_info": "/cam_right/color/camera_info",
        "pose": "/right/gripper_end_pos",
    },
}


class CalibrationCapture(Node):
    def __init__(self) -> None:
        super().__init__("calibration_capture_once")
        self.rgb: dict[str, CompressedImage] = {}
        self.depth: dict[str, Image] = {}
        self.camera_info: dict[str, CameraInfo] = {}
        self.pose: dict[str, Pose] = {}
        self._subscriptions = []

        for camera_id, topics in CAMERAS.items():
            self._subscriptions.append(
                self.create_subscription(
                    CompressedImage,
                    topics["rgb"],
                    lambda msg, cid=camera_id: self.rgb.__setitem__(cid, msg),
                    10,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    Image,
                    topics["depth"],
                    lambda msg, cid=camera_id: self.depth.__setitem__(cid, msg),
                    10,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    CameraInfo,
                    topics["camera_info"],
                    lambda msg, cid=camera_id: self.camera_info.__setitem__(cid, msg),
                    10,
                )
            )
            pose_topic = topics.get("pose")
            if pose_topic:
                self._subscriptions.append(
                    self.create_subscription(
                        Pose,
                        pose_topic,
                        lambda msg, cid=camera_id: self.pose.__setitem__(cid, msg),
                        10,
                    )
                )

    def complete(self, camera_ids: list[str]) -> bool:
        return all(cid in self.rgb and cid in self.depth and cid in self.camera_info for cid in camera_ids)


def decode_compressed_rgb(msg: CompressedImage) -> np.ndarray:
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    image_bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise RuntimeError("cv2.imdecode returned None")
    return image_bgr


def decode_depth(msg: Image) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    encoding = str(msg.encoding).lower()
    step = int(msg.step)
    raw = bytes(msg.data)
    if encoding in {"16uc1", "mono16"}:
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        item_size = 2
        scale = 0.001
    elif encoding == "32fc1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        item_size = 4
        scale = 1.0
    else:
        raise RuntimeError(f"unsupported depth encoding: {msg.encoding}")
    row_values = (step or width * item_size) // item_size
    depth = np.frombuffer(raw, dtype=dtype, count=row_values * height).reshape(height, row_values)[:, :width]
    depth_m = depth.astype(np.float32) * np.float32(scale)
    depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
    return depth_m


def detect_chessboard(image_bgr: np.ndarray, pattern_size: tuple[int, int]) -> tuple[bool, Any, np.ndarray]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    found = False
    corners = None
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(
            gray,
            pattern_size,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
        )
    if not found:
        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    annotated = image_bgr.copy()
    cv2.drawChessboardCorners(annotated, pattern_size, corners, found)
    return bool(found), corners, annotated


def depth_preview(depth_m: np.ndarray) -> np.ndarray:
    valid = depth_m[np.isfinite(depth_m) & (depth_m > 0.0)]
    if valid.size == 0:
        return np.zeros((*depth_m.shape, 3), dtype=np.uint8)
    lo = float(np.percentile(valid, 1.0))
    hi = float(np.percentile(valid, 99.0))
    if not math.isfinite(lo) or not math.isfinite(hi) or hi <= lo:
        hi = lo + 1.0
    scaled = np.clip((depth_m - lo) / (hi - lo), 0.0, 1.0)
    preview = (scaled * 255.0).astype(np.uint8)
    preview[depth_m <= 0.0] = 0
    return cv2.applyColorMap(preview, cv2.COLORMAP_TURBO)


def camera_info_payload(msg: CameraInfo) -> dict[str, Any]:
    return {
        "width": int(msg.width),
        "height": int(msg.height),
        "distortion_model": str(msg.distortion_model),
        "d": [float(v) for v in msg.d],
        "k": [float(v) for v in msg.k],
        "r": [float(v) for v in msg.r],
        "p": [float(v) for v in msg.p],
        "frame_id": str(msg.header.frame_id),
    }


def pose_payload(msg: Pose | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    return {
        "position": {
            "x": float(msg.position.x),
            "y": float(msg.position.y),
            "z": float(msg.position.z),
        },
        "orientation": {
            "x": float(msg.orientation.x),
            "y": float(msg.orientation.y),
            "z": float(msg.orientation.z),
            "w": float(msg.orientation.w),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="outputs/snapshots")
    parser.add_argument("--name", default="")
    parser.add_argument("--pattern-cols", type=int, default=9, help="棋盘格横向内角点数")
    parser.add_argument("--pattern-rows", type=int, default=7, help="棋盘格纵向内角点数")
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    args = parser.parse_args()

    camera_ids = list(CAMERAS)
    pattern_size = (args.pattern_cols, args.pattern_rows)
    sample_name = args.name.strip() or time.strftime("sample_%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir) / sample_name
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = CalibrationCapture()
    deadline = time.monotonic() + max(args.timeout_sec, 1.0)
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.complete(camera_ids):
            rclpy.spin_once(node, timeout_sec=0.05)

        missing = {
            cid: {
                "rgb": cid not in node.rgb,
                "depth": cid not in node.depth,
                "camera_info": cid not in node.camera_info,
            }
            for cid in camera_ids
            if cid not in node.rgb or cid not in node.depth or cid not in node.camera_info
        }
        if missing:
            raise RuntimeError(f"missing messages: {missing}")

        manifest: dict[str, Any] = {
            "sample": sample_name,
            "pattern_size": list(pattern_size),
            "created_at": time.time(),
            "cameras": {},
        }

        for camera_id in camera_ids:
            rgb = decode_compressed_rgb(node.rgb[camera_id])
            depth = decode_depth(node.depth[camera_id])
            found, corners, annotated = detect_chessboard(rgb, pattern_size)

            rgb_path = out_dir / f"{camera_id}_rgb.png"
            annotated_path = out_dir / f"{camera_id}_corners.png"
            depth_path = out_dir / f"{camera_id}_depth_m.npy"
            depth_preview_path = out_dir / f"{camera_id}_depth_preview.png"
            cv2.imwrite(str(rgb_path), rgb)
            cv2.imwrite(str(annotated_path), annotated)
            np.save(depth_path, depth)
            cv2.imwrite(str(depth_preview_path), depth_preview(depth))

            corner_path = None
            if found and corners is not None:
                corner_path = out_dir / f"{camera_id}_corners.npy"
                np.save(corner_path, np.asarray(corners, dtype=np.float32))

            valid_depth = depth[np.isfinite(depth) & (depth > 0.0)]
            manifest["cameras"][camera_id] = {
                "rgb_topic": CAMERAS[camera_id]["rgb"],
                "depth_topic": CAMERAS[camera_id]["depth"],
                "camera_info_topic": CAMERAS[camera_id]["camera_info"],
                "rgb_file": str(rgb_path),
                "depth_file": str(depth_path),
                "depth_preview_file": str(depth_preview_path),
                "corners_file": str(corner_path) if corner_path else None,
                "annotated_file": str(annotated_path),
                "chessboard_found": found,
                "corner_count": int(len(corners)) if found and corners is not None else 0,
                "image_shape": list(rgb.shape),
                "depth_shape": list(depth.shape),
                "depth_valid_ratio": float(valid_depth.size / max(depth.size, 1)),
                "depth_min_m": float(valid_depth.min()) if valid_depth.size else None,
                "depth_median_m": float(np.median(valid_depth)) if valid_depth.size else None,
                "depth_max_m": float(valid_depth.max()) if valid_depth.size else None,
                "camera_info": camera_info_payload(node.camera_info[camera_id]),
                "gripper_pose": pose_payload(node.pose.get(camera_id)),
            }

        manifest_path = out_dir / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(manifest, ensure_ascii=False, indent=2))
        print(f"saved: {manifest_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
