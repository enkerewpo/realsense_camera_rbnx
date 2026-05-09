#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""realsense_camera_rbnx — Intel RealSense D435i RGBD primitive
(capability_id=realsense_camera).

Owns `robonix/primitive/camera/*`. The D435i has an internal IMU but
we deliberately do NOT atlas-route it under `primitive/imu/imu`
(MID-360 IMU is canonical for the ranger). Subscribers needing the
camera IMU directly can read /<camera_name>/imu.

Lifecycle:
    on_init      — spawn rs_launch.py with camera_name + profiles → wait
                   for first RGB frame → declare rgb + depth topic_out.
    on_shutdown  — kill realsense subprocess.

Config (from manifest):
    camera_name        default "camera_435i"
    rgb_topic          default "/<camera_name>/color/image_raw"
    depth_topic        default "/<camera_name>/aligned_depth_to_color/image_raw"
    rgb_profile        default "640x480x30"
    depth_profile      default "848x480x30"
    align_depth        default true
    enable_imu         default true   (published, NOT atlas-routed)
    enable_sync        default true
    sentinel_timeout_s default 30.0
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from robonix_api import Capability, Ok, Err

logging.basicConfig(
    level=os.environ.get("REALSENSE_LOG_LEVEL", "INFO"),
    format="[realsense] %(message)s",
)
log = logging.getLogger("realsense")

cap = Capability(id="realsense_camera", namespace="robonix/primitive/camera")

_pkg_root: Path = Path(__file__).resolve().parent.parent
_rs_proc: subprocess.Popen | None = None


def _spawn_realsense(cfg: dict) -> None:
    """Launch ros2 launch realsense2_camera rs_launch.py with config args."""
    global _rs_proc
    cam = cfg.get("camera_name", "camera_435i")
    args = [
        "ros2", "launch", "realsense2_camera", "rs_launch.py",
        "camera_namespace:=/",
        f"camera_name:={cam}",
        f"enable_imu:={'true' if cfg.get('enable_imu', True) else 'false'}",
        f"enable_gyro:={'true' if cfg.get('enable_imu', True) else 'false'}",
        f"enable_accel:={'true' if cfg.get('enable_imu', True) else 'false'}",
        "unite_imu_method:=2",
        f"align_depth.enable:={'true' if cfg.get('align_depth', True) else 'false'}",
        f"enable_sync:={'true' if cfg.get('enable_sync', True) else 'false'}",
        "publish_tf:=true",  # rtabmap consumes camera_link → optical_frame TFs
        "temporal_filter.enable:=true",
        "hole_filling_filter.enable:=true",
        f"rgb_camera.color_profile:={cfg.get('rgb_profile', '640x480x30')}",
        f"depth_module.depth_profile:={cfg.get('depth_profile', '848x480x30')}",
    ]
    log_path = _pkg_root / "rbnx-build" / "data" / "realsense.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fh = open(log_path, "ab", buffering=0)
    log.info("spawning realsense (cam=%s) → %s", cam, log_path)
    log.debug("launch args: %s", " ".join(args))
    _rs_proc = subprocess.Popen(
        args, stdout=log_fh, stderr=log_fh, start_new_session=True,
    )


def _kill_realsense() -> None:
    p = _rs_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def _wait_for_image(topic: str, timeout_s: float) -> bool:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image
    except ImportError as e:
        log.warning("rclpy unavailable (%s); skipping sentinel wait", e)
        return True
    rclpy.init(args=None)
    node = Node("realsense_atlas_sentinel")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen = threading.Event()
    node.create_subscription(Image, topic, lambda _m: seen.set(), qos)
    log.info("waiting for first frame on %s — up to %.1fs", topic, timeout_s)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if seen.is_set():
                break
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return seen.is_set()


@cap.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE: spawn realsense, wait for RGB, declare rgb/depth."""
    cam = cfg.get("camera_name", "camera_435i")
    rgb_topic = cfg.get("rgb_topic", f"/{cam}/color/image_raw")
    depth_topic = cfg.get(
        "depth_topic", f"/{cam}/aligned_depth_to_color/image_raw"
    )
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))

    try:
        _spawn_realsense(cfg)
    except Exception as e:  # noqa: BLE001
        return Err(f"spawn realsense failed: {e}")

    if not _wait_for_image(rgb_topic, sentinel_timeout):
        _kill_realsense()
        return Err(f"no Image on {rgb_topic} within {sentinel_timeout:.1f}s")

    cap.declare_ros2(
        "robonix/primitive/camera/rgb",
        topic=rgb_topic,
        qos="best_effort",
    )
    cap.declare_ros2(
        "robonix/primitive/camera/depth",
        topic=depth_topic,
        qos="best_effort",
    )
    log.info("init complete: rgb=%s depth=%s", rgb_topic, depth_topic)
    return Ok()


@cap.on_shutdown
def shutdown():
    _kill_realsense()
    return Ok()


if __name__ == "__main__":
    cap.run()
