#!/usr/bin/env python3
"""Export an eye-in-hand result for the path planning state machine."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def export(result_path: Path, camera: str, output_path: Path) -> None:
    data = json.loads(result_path.read_text(encoding="utf-8"))
    if data.get("mode") != "eye_in_hand":
        raise ValueError("Expected an eye_in_hand calibration result")
    result = data["result"]
    translation = [float(value) for value in result["position"]]
    orientation = [float(value) for value in result["orientation_xyzw"]]
    if len(translation) != 3 or len(orientation) != 4:
        raise ValueError("Expected XYZ translation and XYZW quaternion")
    if not all(math.isfinite(value) for value in translation + orientation):
        raise ValueError("Calibration transform contains a nonfinite value")
    if abs(sum(value * value for value in orientation) - 1.0) > 0.02:
        raise ValueError("Calibration quaternion is not normalized")
    arm = camera.removeprefix("cam_")
    payload = {
        "camera": camera,
        "mode": "eye_in_hand",
        "source_result_json": result_path.name,
        "source_sample_count": int(data["sample_count"]),
        "parent_frame_note": f"/{arm}/gripper_end_pos pose frame",
        "child_frame": f"{camera}_color_optical_frame",
        "transform_convention": "T_gripper_camera",
        "translation_m": translation,
        "orientation_xyzw": orientation,
        "matrix": result["matrix"],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(output_path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_json", type=Path)
    parser.add_argument("--camera", choices=("cam_left", "cam_right"), required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    export(args.result_json, args.camera, args.out)


if __name__ == "__main__":
    main()
