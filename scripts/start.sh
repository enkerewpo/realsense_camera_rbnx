#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Start the atlas bridge. THIS SCRIPT DOES NOT START THE REALSENSE
# DRIVER — `realsense2_camera`'s `rs_launch.py` is spawned inside
# atlas_bridge's `Driver(CMD_INIT)` handler, AFTER `rbnx boot` calls
# in with config (camera_name, resolutions, align_depth, …).

set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$PKG/rbnx-build/ws/install/setup.bash"
else
    echo "[realsense_camera/start] ERROR: rbnx-build/ws/install missing — run rbnx build first" >&2
    exit 1
fi

export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
if ROBONIX_PY="$(rbnx path robonix-py 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_PY:$PYTHONPATH"
fi

exec python3 -m realsense_camera.atlas_bridge
