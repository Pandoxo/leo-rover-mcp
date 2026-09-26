"""Simulated Leo Rover: differential drive with the firmware's acceleration limits and cmd_vel timeout,
a small walled room with boxes (the rover stalls when it hits one) and a top-down 'camera'."""

from __future__ import annotations

import asyncio
import io
import math
import time

from .backend import BackendError, Telemetry

ROOM = (-2.0, -2.0, 4.0, 3.0)  # xmin, ymin, xmax, ymax [m]
OBSTACLES = [  # axis-aligned boxes: xmin, ymin, xmax, ymax [m]
    (1.2, -0.4, 1.6, 0.4),
    (-0.8, 1.2, 0.2, 1.6),
    (2.6, 1.6, 3.2, 2.2),
]
RADIUS = 0.25  # footprint circle of the ~0.45 x 0.43 m rover
LIN_ACC, ANG_ACC = 0.5, 1.0  # firmware defaults [m/s^2], [rad/s^2]
CMD_TIMEOUT = 0.5  # firmware controller.input_timeout
DT = 0.02


def _collides(x: float, y: float) -> bool:
    xmin, ymin, xmax, ymax = ROOM
    if not (xmin + RADIUS <= x <= xmax - RADIUS and ymin + RADIUS <= y <= ymax - RADIUS):
        return True
    for bx0, by0, bx1, by1 in OBSTACLES:
        cx, cy = min(max(x, bx0), bx1), min(max(y, by0), by1)
        if math.hypot(x - cx, y - cy) < RADIUS:
            return True
    return False


class SimBackend:
    kind = "simulation"

    def __init__(self):
        self.telemetry = Telemetry()
        self.odom_topic = "sim"
        self._cmd = (0.0, 0.0)
        self._cmd_t = 0.0
        self._v = self._w = 0.0
        self._x = self._y = self._yaw = 0.0
        self._odom0 = (0.0, 0.0, 0.0)  # odometry origin in world frame
        self._battery = 12.4
        self._task: asyncio.Task | None = None
        self.bumped = False

    @property
    def alive(self) -> bool:
        return self._task is not None and not self._task.done()

    lost_reason = None

    async def connect(self) -> dict:
        self._task = asyncio.create_task(self._physics())
        await asyncio.sleep(0.1)
        return {"url": "simulation", "odom_topic": self.odom_topic}

    async def close(self) -> None:
        if self._task is not None:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
        self._task = None

    async def send_cmd_vel(self, v: float, w: float) -> None:
        if not self.alive:
            raise BackendError("simulation not running")
        self._cmd, self._cmd_t = (v, w), time.monotonic()

    async def _physics(self) -> None:
        last = time.monotonic()
        while True:
            await asyncio.sleep(DT)
            now = time.monotonic()
            dt, last = now - last, now
            cv, cw = self._cmd if now - self._cmd_t < CMD_TIMEOUT else (0.0, 0.0)
            self._v += max(-LIN_ACC * dt, min(LIN_ACC * dt, cv - self._v))
            self._w += max(-ANG_ACC * dt, min(ANG_ACC * dt, cw - self._w))
            yaw = self._yaw + self._w * dt
            x = self._x + self._v * math.cos(yaw) * dt
            y = self._y + self._v * math.sin(yaw) * dt
            if _collides(x, y) and not _collides(self._x, self._y):
                self._v, self.bumped = 0.0, True  # wheels blocked: odometry sees no motion
                x, y = self._x, self._y
            self._x, self._y, self._yaw = x, y, math.atan2(math.sin(yaw), math.cos(yaw))
            self._battery = max(10.0, self._battery - dt * (2e-5 + 2e-4 * (abs(self._v) + abs(self._w))))
            ox, oy, oyaw = self._odom0
            dx, dy = self._x - ox, self._y - oy
            t = self.telemetry
            t.x = dx * math.cos(-oyaw) - dy * math.sin(-oyaw)
            t.y = dx * math.sin(-oyaw) + dy * math.cos(-oyaw)
            t.yaw = math.atan2(math.sin(self._yaw - oyaw), math.cos(self._yaw - oyaw))
            t.v, t.w, t.odom_t = self._v, self._w, now
            t.battery_v, t.battery_t = round(self._battery, 3), now
            t.roll, t.pitch, t.imu_t = 0.0, 0.0, now

    async def camera_topics(self) -> list[str]:
        return ["/sim/top_down/compressed"]

    async def snapshot(self, topic: str | None, timeout: float = 4.0) -> tuple[bytes, str, dict]:
        try:
            from PIL import Image, ImageDraw
        except ImportError as e:  # pragma: no cover
            raise BackendError("the simulated camera needs Pillow") from e
        scale = 100  # px per metre
        xmin, ymin, xmax, ymax = ROOM
        w, h = int((xmax - xmin) * scale), int((ymax - ymin) * scale)

        def px(x: float, y: float) -> tuple[float, float]:
            return (x - xmin) * scale, (ymax - y) * scale

        img = Image.new("RGB", (w, h), (235, 235, 228))
        d = ImageDraw.Draw(img)
        for gx in range(math.ceil(xmin), math.floor(xmax) + 1):
            d.line([px(gx, ymin), px(gx, ymax)], fill=(205, 205, 200))
        for gy in range(math.ceil(ymin), math.floor(ymax) + 1):
            d.line([px(xmin, gy), px(xmax, gy)], fill=(205, 205, 200))
        for bx0, by0, bx1, by1 in OBSTACLES:
            d.rectangle([px(bx0, by1), px(bx1, by0)], fill=(150, 90, 60))
        d.rectangle([0, 0, w - 1, h - 1], outline=(40, 40, 40), width=4)
        cx, cy = px(self._x, self._y)
        r = RADIUS * scale
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(40, 110, 200))
        tip = px(self._x + 0.4 * math.cos(self._yaw), self._y + 0.4 * math.sin(self._yaw))
        d.line([(cx, cy), tip], fill=(230, 60, 40), width=5)
        d.text((8, 8), "SIMULATION top-down view, 1 m grid, red line = rover heading", fill=(0, 0, 0))
        buf = io.BytesIO()
        img.save(buf, "PNG")
        return buf.getvalue(), "png", {"topic": "/sim/top_down/compressed", "note": "simulated overhead map, not a real camera"}

    async def list_topics(self) -> list[dict]:
        return [{"name": "/sim/top_down/compressed", "type": "sensor_msgs/msg/CompressedImage"}]

    async def trigger(self, service: str) -> dict:
        if service == "reset_odometry":
            self._odom0 = (self._x, self._y, self._yaw)
            return {"success": True, "message": "odometry reset"}
        raise BackendError(f"service {service} not simulated")
