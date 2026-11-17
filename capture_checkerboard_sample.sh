#!/usr/bin/env bash
set -euo pipefail

# Capture one calibration snapshot from all three cameras.
# Assumes start_calibration_rgb_depth_aligned.sh is running.

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

NAME="${1:-}"

args=(
  --pattern-cols 8
  --pattern-rows 11
  --timeout-sec 10
)

if [[ -n "${NAME}" ]]; then
  args+=(--name "${NAME}")
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${SCRIPT_DIR}/capture_calibration_sample.py" "${args[@]}"
