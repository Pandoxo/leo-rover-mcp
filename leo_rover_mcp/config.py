"""Settings, all overridable through environment variables (read once at import)."""

from __future__ import annotations

import os


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name, "").strip()
    return float(raw) if raw else default


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name, "").strip().lower()
    if not raw:
        return default
    return raw in ("1", "true", "yes", "on")


# --- connection -------------------------------------------------------------
# Leo Rover access point: 10.0.0.1, rosbridge (used by the stock leo_ui) on port 9090
URL = os.environ.get("LEO_URL", "ws://10.0.0.1:9090").strip()
NAMESPACE = os.environ.get("LEO_NAMESPACE", "").strip().strip("/")
DRY_RUN = _env_bool("LEO_DRY_RUN", False)
CONNECT_TIMEOUT_S = _env_float("LEO_CONNECT_TIMEOUT", 5.0)

# topics are relative to NAMESPACE (see docs.fictionlab.pl -> Leo Rover -> ROS API)
CMD_VEL_TOPIC = os.environ.get("LEO_CMD_VEL_TOPIC", "cmd_vel")
ODOM_TOPICS = [t.strip() for t in os.environ.get("LEO_ODOM_TOPIC", "merged_odom,wheel_odom").split(",") if t.strip()]
IMU_TOPIC = os.environ.get("LEO_IMU_TOPIC", "imu/data")
BATTERY_TOPIC = os.environ.get("LEO_BATTERY_TOPIC", "firmware/battery_averaged")
CAMERA_TOPIC = os.environ.get("LEO_CAMERA_TOPIC", "").strip()  # empty: auto-discover a CompressedImage topic
RESET_ODOM_SERVICE = "reset_odometry"

# --- motion limits ----------------------------------------------------------
MAX_LINEAR = _env_float("LEO_MAX_LINEAR", 0.4)  # m/s, the rover tops out around 0.4
DEFAULT_LINEAR = min(_env_float("LEO_DEFAULT_LINEAR", 0.2), MAX_LINEAR)
MAX_ANGULAR = _env_float("LEO_MAX_ANGULAR", 1.0)  # rad/s
DEFAULT_ANGULAR = min(_env_float("LEO_DEFAULT_ANGULAR", 0.6), MAX_ANGULAR)
MIN_LINEAR = 0.04  # slowest useful speed near a goal
MIN_ANGULAR = 0.15
MAX_DURATION_S = _env_float("LEO_MAX_DURATION", 10.0)  # per rover_drive call
MAX_DISTANCE_M = _env_float("LEO_MAX_DISTANCE", 3.0)  # per rover_move call
MAX_TURN_DEG = 360.0

# --- safety -----------------------------------------------------------------
CMD_RATE_HZ = 10.0  # firmware stops the wheels after 0.5 s without cmd_vel
ODOM_STALE_S = 0.6  # abort a motion when odometry is older than this
STALL_TIME_S = _env_float("LEO_STALL_TIME", 1.5)  # commanded but not moving this long -> stop
STALL_DIST_M = 0.02
STALL_YAW_DEG = 3.0
BATTERY_MIN_V = _env_float("LEO_BATTERY_MIN", 10.4)  # refuse motion below (3S Li-ion pack)
BATTERY_WARN_V = _env_float("LEO_BATTERY_WARN", 11.0)
TILT_WARN_DEG = 20.0
TILT_MAX_DEG = _env_float("LEO_TILT_MAX", 30.0)  # refuse motion when roll/pitch exceeds this

# --- goal tolerances / gains -------------------------------------------------
DIST_TOL_M = 0.01
YAW_TOL_DEG = 1.0
K_DIST = 1.2  # 1/s, slow down over the last ~0.15 m
K_YAW = 2.0  # 1/s
K_HEADING = 1.5  # heading hold while driving straight

# --- camera -------------------------------------------------------------------
IMAGE_MAX_WIDTH = int(_env_float("LEO_IMAGE_WIDTH", 640))
IMAGE_QUALITY = 80
