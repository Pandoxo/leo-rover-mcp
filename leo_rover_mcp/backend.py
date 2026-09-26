"""Rover backends: the real Leo Rover through rosbridge, and (in sim.py) a simulated one.

A backend keeps the latest telemetry and sends velocity commands; all safety logic lives in rover.py.
"""

from __future__ import annotations

import asyncio
import base64
import logging
import math
import time
from dataclasses import dataclass

from . import config as C
from .rosbridge import RosbridgeClient, RosbridgeError

log = logging.getLogger("leo_rover_mcp.backend")


class BackendError(RuntimeError):
    pass


@dataclass
class Telemetry:
    """Latest readings; *_t are time.monotonic() receive times (None = never received)."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0  # rad, odometry frame
    v: float = 0.0  # m/s measured
    w: float = 0.0  # rad/s measured
    odom_t: float | None = None
    battery_v: float | None = None
    battery_t: float | None = None
    roll: float | None = None  # rad
    pitch: float | None = None
    imu_t: float | None = None


def yaw_from_quat(q: dict) -> float:
    x, y, z, w = (float(q.get(k, 0.0)) for k in ("x", "y", "z", "w"))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def roll_pitch_from_quat(q: dict) -> tuple[float, float]:
    x, y, z, w = (float(q.get(k, 0.0)) for k in ("x", "y", "z", "w"))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


# LeoOS camera topics (image_transport), best first; mono and rectified-mono are a last resort
CAMERA_PREFERENCE = ("image_raw/compressed", "image_color/compressed", "image_rect_color/compressed")


def _pick_camera_topic(topics: list[str]) -> str:
    for suffix in CAMERA_PREFERENCE:
        for t in topics:
            if t.endswith(suffix):
                return t
    colour = [t for t in topics if "mono" not in t]
    return next((t for t in colour if "camera" in t), (colour or topics)[0])


def _topic(name: str) -> str:
    name = name.strip("/")
    return f"/{C.NAMESPACE}/{name}" if C.NAMESPACE else f"/{name}"


class LeoBackend:
    """Real rover over rosbridge (LeoOS runs rosbridge_server on port 9090 for its web UI)."""

    kind = "rosbridge"

    def __init__(self, url: str | None = None):
        self.url = url or C.URL
        self.telemetry = Telemetry()
        self.odom_topic: str | None = None
        self._client = RosbridgeClient(self.url)
        self._odom_subs: list[tuple[str, str]] = []

    @property
    def alive(self) -> bool:
        return self._client.connected

    @property
    def lost_reason(self) -> str | None:
        return self._client.close_reason

    async def connect(self) -> dict:
        try:
            await self._client.connect(C.CONNECT_TIMEOUT_S)
            await self._client.advertise(_topic(C.CMD_VEL_TOPIC), "geometry_msgs/msg/Twist")
            await self._client.subscribe(_topic(C.BATTERY_TOPIC), "std_msgs/msg/Float32", self._on_battery, 200)
            await self._client.subscribe(_topic(C.IMU_TOPIC), "sensor_msgs/msg/Imu", self._on_imu, 50)
            for name in C.ODOM_TOPICS:  # first odometry topic that talks wins
                topic = _topic(name)
                sid = await self._client.subscribe(topic, "nav_msgs/msg/Odometry", self._odom_cb(topic))
                self._odom_subs.append((topic, sid))
            deadline = time.monotonic() + 3.0
            while self.odom_topic is None and time.monotonic() < deadline:
                await asyncio.sleep(0.05)
            await asyncio.sleep(0.3)  # let battery / IMU arrive too
        except RosbridgeError as e:
            await self.close()
            raise BackendError(str(e)) from e
        if self.odom_topic is None:
            await self.close()
            raise BackendError(
                f"connected to {self.url} but no odometry on {[t for t, _ in self._odom_subs] or C.ODOM_TOPICS}; "
                "is leo_bringup running (ros2 topic list on the rover)? Set LEO_NAMESPACE if the rover uses one"
            )
        for topic, sid in self._odom_subs:
            if topic != self.odom_topic:
                await self._client.unsubscribe(topic, sid)
        return {"url": self.url, "odom_topic": self.odom_topic}

    async def close(self) -> None:
        await self._client.close()

    def _odom_cb(self, topic: str):
        def cb(msg: dict) -> None:
            if self.odom_topic is None:
                self.odom_topic = topic
            if topic != self.odom_topic:
                return
            pose = msg.get("pose", {}).get("pose", {})
            twist = msg.get("twist", {}).get("twist", {})
            t = self.telemetry
            t.x = float(pose.get("position", {}).get("x", 0.0))
            t.y = float(pose.get("position", {}).get("y", 0.0))
            t.yaw = yaw_from_quat(pose.get("orientation", {}))
            t.v = float(twist.get("linear", {}).get("x", 0.0))
            t.w = float(twist.get("angular", {}).get("z", 0.0))
            t.odom_t = time.monotonic()

        return cb

    def _on_battery(self, msg: dict) -> None:
        self.telemetry.battery_v = float(msg.get("data", 0.0))
        self.telemetry.battery_t = time.monotonic()

    def _on_imu(self, msg: dict) -> None:
        q = msg.get("orientation") or {}
        if not any(q.get(k) for k in ("x", "y", "z", "w")):
            return  # raw IMU without orientation estimate
        self.telemetry.roll, self.telemetry.pitch = roll_pitch_from_quat(q)
        self.telemetry.imu_t = time.monotonic()

    async def send_cmd_vel(self, v: float, w: float) -> None:
        try:
            await self._client.publish(
                _topic(C.CMD_VEL_TOPIC),
                {"linear": {"x": v, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": w}},
            )
        except RosbridgeError as e:
            raise BackendError(str(e)) from e

    async def camera_topics(self) -> list[str]:
        try:
            res = await self._client.call_service("/rosapi/topics_for_type", {"type": "sensor_msgs/msg/CompressedImage"})
        except RosbridgeError as e:
            raise BackendError(f"cannot list camera topics (rosapi): {e}") from e
        return sorted(res.get("topics", []))

    async def snapshot(self, topic: str | None, timeout: float = 4.0) -> tuple[bytes, str, dict]:
        if not topic:
            if C.CAMERA_TOPIC:
                topic = _topic(C.CAMERA_TOPIC)
            else:
                topics = await self.camera_topics()
                if not topics:
                    raise BackendError("no sensor_msgs/CompressedImage topic on the rover (is the camera running?)")
                topic = _pick_camera_topic(topics)
        fut = asyncio.get_running_loop().create_future()

        def cb(msg: dict) -> None:
            if not fut.done():
                fut.set_result(msg)

        try:
            sid = await self._client.subscribe(topic, "sensor_msgs/msg/CompressedImage", cb, 100)
            try:
                msg = await asyncio.wait_for(fut, timeout)
            finally:
                await self._client.unsubscribe(topic, sid)
        except asyncio.TimeoutError as e:
            raise BackendError(f"no image on {topic} within {timeout:.0f} s") from e
        except RosbridgeError as e:
            raise BackendError(str(e)) from e
        data = msg.get("data", "")
        raw = base64.b64decode(data) if isinstance(data, str) else bytes(data)  # rosbridge sends uint8[] as base64
        fmt = str(msg.get("format", "jpeg")).lower()
        return raw, ("png" if "png" in fmt else "jpeg"), {"topic": topic}

    async def list_topics(self) -> list[dict]:
        try:
            res = await self._client.call_service("/rosapi/topics")
        except RosbridgeError as e:
            raise BackendError(f"cannot list topics (rosapi): {e}") from e
        return [{"name": n, "type": t} for n, t in zip(res.get("topics", []), res.get("types", []))]

    async def trigger(self, service: str) -> dict:
        try:
            return await self._client.call_service(_topic(service))
        except RosbridgeError as e:
            raise BackendError(str(e)) from e
