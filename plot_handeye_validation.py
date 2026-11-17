#!/usr/bin/env python3
"""Plot fixed-checkerboard consistency from saved hand-eye results."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from validate_handeye_result import mean_rotation, rotation_angle_deg


def residuals(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = json.loads(path.read_text(encoding="utf-8"))
    handeye = np.asarray(data["result"]["matrix"], dtype=np.float64)
    poses = [
        np.asarray(sample["robot_matrix"]["matrix"], dtype=np.float64)
        @ handeye
        @ np.asarray(sample["target_matrix"]["matrix"], dtype=np.float64)
        for sample in data["samples"]
    ]
    translations = np.asarray([pose[:3, 3] for pose in poses])
    center = translations.mean(axis=0)
    position_mm = np.linalg.norm(translations - center, axis=1) * 1000.0
    rotation_center = mean_rotation([pose[:3, :3] for pose in poses])
    rotation_deg = np.asarray(
        [rotation_angle_deg(rotation_center.T @ pose[:3, :3]) for pose in poses]
    )
    return position_mm, rotation_deg


def main() -> None:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", type=Path, default=root / "docs/examples/left_handeye_result.json")
    parser.add_argument("--right", type=Path, default=root / "docs/examples/right_handeye_result.json")
    parser.add_argument("--out", type=Path, default=root / "docs/images/handeye_validation.png")
    args = parser.parse_args()

    fig, axes = plt.subplots(2, 1, figsize=(10, 6.2), sharex=True)
    colors = {"Left wrist": "#1768ac", "Right wrist": "#db6d28"}
    for label, path in (("Left wrist", args.left), ("Right wrist", args.right)):
        position_mm, rotation_deg = residuals(path)
        sample_index = np.arange(1, len(position_mm) + 1)
        axes[0].plot(sample_index, position_mm, "o-", ms=4, lw=1.7,
                     color=colors[label], label=f"{label} (n={len(position_mm)})")
        axes[1].plot(sample_index, rotation_deg, "o-", ms=4, lw=1.7,
                     color=colors[label], label=label)

    axes[0].set_ylabel("Position residual (mm)")
    axes[1].set_ylabel("Rotation residual (deg)")
    axes[1].set_xlabel("Captured pose pair")
    axes[0].set_title("Eye-in-hand calibration: fixed-board consistency", loc="left", fontsize=15, pad=15)
    axes[0].legend(frameon=False, ncol=2)
    for axis in axes:
        axis.grid(axis="y", color="#d9e0e6", lw=0.7)
        axis.spines[["top", "right"]].set_visible(False)
        axis.set_ylim(bottom=0)
    axes[1].set_xticks(np.arange(1, max(len(residuals(args.left)[0]), len(residuals(args.right)[0])) + 1, 2))
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=180, facecolor="white")
    print(args.out)


if __name__ == "__main__":
    main()
