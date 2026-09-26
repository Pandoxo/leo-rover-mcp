"""Rover control with safety checks: speed/duration caps, battery and tilt guards, odometry watchdog,
stall detection and a stop that interrupts any running motion.

Every motion is a loop at CMD_RATE_HZ that re-sends cmd_vel (the firmware halts after 0.5 s of silence,
so if this process dies the rover stops by itself) and always ends with zero velocity.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import Callable

from . import config as C
from .backend import BackendError, LeoBackend, Telemetry
from .sim import SimBackend

log = logging.getLogger("leo_rover_mcp.rover")


class RoverError(RuntimeError):
    pass


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x: float, lim: float) -> float:
    return max(-lim, min(lim, x))


def _finite(**values: float | None) -> None:
    # NaN slips through every range check (all comparisons are False) and would make the timeout NaN too
    for name, x in values.items():
        if x is not None and not math.isfinite(x):
            raise RoverError(f"{name} must be a finite number")


class Rover:
    def __init__(self):
        self.backend: LeoBackend | SimBackend | None = None
        self._motion_lock = asyncio.Lock()
        self._stop_requested = False
        self.last_motion: dict | None = None

    # -- connection -------------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self.backend is not None and self.backend.alive

    async def connect(self, simulate: bool | None = None, url: str | None = None) -> dict:
        if self.connected:
            b = self.backend
            is_sim = b.kind == "simulation"
            want_sim = simulate if simulate is not None else (False if url else is_sim)
            if want_sim != is_sim or (url and not is_sim and url != b.url):
                raise RoverError(f"already connected in mode '{b.kind}'; call rover_disconnect before switching")
            return {"ok": True, "already_connected": True, **self.status()}
        if self.backend is not None:  # dead connection left over
            await self.backend.close()
            self.backend = None
        # an explicit url means the real rover, even when LEO_DRY_RUN makes the simulator the default
        sim = (C.DRY_RUN and not url) if simulate is None else simulate
        backend = SimBackend() if sim else LeoBackend(url)
        try:
            info = await backend.connect()
        except BackendError as e:
            raise RoverError(str(e)) from e
        self.backend = backend
        await self._send(0.0, 0.0)
        return {"ok": True, "mode": backend.kind, **info, **self.status()}

    async def disconnect(self) -> dict:
        if self.backend is None:
            return {"ok": True, "note": "was not connected"}
        await self.stop()
        await self.backend.close()
        self.backend = None
        return {"ok": True, "disconnected": True}

    def _require(self) -> LeoBackend | SimBackend:
        if self.backend is None:
            raise RoverError("not connected: call rover_connect first")
        if not self.backend.alive:
            raise RoverError(f"connection to the rover is gone ({self.backend.lost_reason or 'closed'}); call rover_connect")
        return self.backend

    async def _send(self, v: float, w: float) -> None:
        try:
            await self._require().send_cmd_vel(v, w)
        except BackendError as e:
            raise RoverError(str(e)) from e

    # -- state ----------------------------------------------------------------------

    def status(self) -> dict:
        b = self._require()
        t: Telemetry = b.telemetry
        now = time.monotonic()

        def age(ts: float | None) -> float | None:
            return None if ts is None else round(now - ts, 2)

        warnings = []
        if t.battery_v is not None and t.battery_v < C.BATTERY_WARN_V:
            warnings.append(f"battery low ({t.battery_v:.2f} V), motion refused below {C.BATTERY_MIN_V} V")
        if t.roll is not None and max(abs(math.degrees(t.roll)), abs(math.degrees(t.pitch or 0))) > C.TILT_WARN_DEG:
            warnings.append("rover strongly tilted")
        if t.odom_t is None or now - t.odom_t > C.ODOM_STALE_S:
            warnings.append("odometry stale: motion refused")
        return {
            "mode": b.kind,
            "pose": {"x_m": round(t.x, 3), "y_m": round(t.y, 3), "heading_deg": round(math.degrees(t.yaw), 1)},
            "velocity": {"linear_m_s": round(t.v, 3), "angular_deg_s": round(math.degrees(t.w), 1)},
            "battery_v": None if t.battery_v is None else round(t.battery_v, 2),
            "tilt_deg": None if t.roll is None else
            {"roll": round(math.degrees(t.roll), 1), "pitch": round(math.degrees(t.pitch or 0.0), 1)},
            "data_age_s": {"odom": age(t.odom_t), "battery": age(t.battery_t), "imu": age(t.imu_t)},
            "moving": self._motion_lock.locked(),
            "last_motion": self.last_motion,
            "warnings": warnings,
        }

    def _preflight(self) -> Telemetry:
        t = self._require().telemetry
        now = time.monotonic()
        if t.odom_t is None or now - t.odom_t > C.ODOM_STALE_S:
            raise RoverError("odometry is stale or missing: refusing to move (check the rover's ROS nodes)")
        if t.battery_v is not None and t.battery_t is not None and now - t.battery_t < 5 and t.battery_v < C.BATTERY_MIN_V:
            raise RoverError(f"battery at {t.battery_v:.2f} V (< {C.BATTERY_MIN_V} V): charge the rover")
        if t.roll is not None and t.imu_t is not None and now - t.imu_t < 2:
            tilt = max(abs(math.degrees(t.roll)), abs(math.degrees(t.pitch or 0.0)))
            if tilt > C.TILT_MAX_DEG:
                raise RoverError(f"rover tilted {tilt:.0f} deg (> {C.TILT_MAX_DEG}): refusing to move")
        return t

    # -- motion core ------------------------------------------------------------------

    async def _run(self, kind: str, step: Callable[[Telemetry, float], tuple[float, float] | None],
                   timeout: float, report: Callable[[Telemetry], dict]) -> dict:
        if self._motion_lock.locked():
            raise RoverError("another motion is running; call rover_stop first")
        async with self._motion_lock:
            t = self._preflight()
            self._stop_requested = False
            start = (t.x, t.y, t.yaw)
            anchor = (t.x, t.y, t.yaw, time.monotonic())  # stall watchdog reference
            t0 = time.monotonic()
            period = 1.0 / C.CMD_RATE_HZ
            outcome, detail = "done", ""
            try:
                while True:
                    now = time.monotonic()
                    elapsed = now - t0
                    if self._stop_requested:
                        outcome, detail = "stopped", "rover_stop was called"
                        break
                    if not self._require().alive:
                        outcome, detail = "aborted", "connection lost"
                        break
                    if t.odom_t is None or now - t.odom_t > C.ODOM_STALE_S:
                        outcome, detail = "aborted", "odometry stopped updating"
                        break
                    if elapsed > timeout:
                        outcome, detail = "timeout", f"goal not reached within {timeout:.1f} s"
                        break
                    cmd = step(t, elapsed)
                    if cmd is None:
                        break
                    v, w = _clamp(cmd[0], C.MAX_LINEAR), _clamp(cmd[1], C.MAX_ANGULAR)
                    # stall: commanding motion but the pose has not changed for STALL_TIME_S
                    if math.hypot(t.x - anchor[0], t.y - anchor[1]) > C.STALL_DIST_M or \
                            abs(math.degrees(_wrap(t.yaw - anchor[2]))) > C.STALL_YAW_DEG or \
                            (abs(v) < C.MIN_LINEAR / 2 and abs(w) < C.MIN_ANGULAR / 2):
                        anchor = (t.x, t.y, t.yaw, now)
                    elif now - anchor[3] > C.STALL_TIME_S:
                        outcome, detail = "stalled", ("commanded to move but odometry shows no motion for "
                                                      f"{C.STALL_TIME_S:.1f} s: probably blocked by an obstacle or stuck")
                        break
                    await self._send(v, w)
                    await asyncio.sleep(period)
            except asyncio.CancelledError:
                outcome, detail = "aborted", "request cancelled"
                raise
            except RoverError as e:
                outcome, detail = "aborted", str(e)
            finally:
                await self._halt()
                self.last_motion = {"kind": kind, "outcome": outcome, **({"detail": detail} if detail else {})}
            if self.connected:
                await asyncio.sleep(0.4)  # let it settle so the report shows where it really stopped
            moved = math.hypot(t.x - start[0], t.y - start[1])
            fwd = (t.x - start[0]) * math.cos(start[2]) + (t.y - start[1]) * math.sin(start[2])
            result = {
                "ok": outcome == "done",
                "outcome": outcome,
                **({"detail": detail} if detail else {}),
                "elapsed_s": round(time.monotonic() - t0, 2),
                "moved": {"distance_m": round(moved, 3), "forward_m": round(fwd, 3),
                          "turned_deg": round(math.degrees(_wrap(t.yaw - start[2])), 1)},
                **report(t),
                "pose": self.status()["pose"] if self.connected else None,
            }
            self.last_motion.update(result["moved"])
            return result

    async def _halt(self) -> None:
        for _ in range(3):  # zero twice more in case one message gets lost
            try:
                await self._send(0.0, 0.0)
            except RoverError:
                return  # no connection: the firmware timeout stops the wheels
            await asyncio.sleep(0.03)

    # -- motion commands ------------------------------------------------------------

    async def drive(self, linear: float, angular_deg_s: float, duration: float) -> dict:
        _finite(linear_m_s=linear, angular_deg_s=angular_deg_s, duration_s=duration)
        if not 0 < duration <= C.MAX_DURATION_S:
            raise RoverError(f"duration must be in (0, {C.MAX_DURATION_S}] s")
        if abs(linear) > C.MAX_LINEAR:
            raise RoverError(f"|linear| must be <= {C.MAX_LINEAR} m/s")
        w = math.radians(angular_deg_s)
        if abs(w) > C.MAX_ANGULAR:
            raise RoverError(f"|angular| must be <= {math.degrees(C.MAX_ANGULAR):.0f} deg/s")

        def step(_t: Telemetry, elapsed: float):
            return None if elapsed >= duration else (linear, w)

        return await self._run("drive", step, duration + 1.0, lambda _t: {})

    async def move(self, distance: float, speed: float | None = None) -> dict:
        _finite(distance_m=distance, speed_m_s=speed)
        if abs(distance) > C.MAX_DISTANCE_M:
            raise RoverError(f"|distance| must be <= {C.MAX_DISTANCE_M} m per call; split longer drives")
        if abs(distance) < C.DIST_TOL_M:
            raise RoverError(f"distance too small (< {C.DIST_TOL_M} m)")
        speed = C.DEFAULT_LINEAR if speed is None else abs(speed)
        if not C.MIN_LINEAR <= speed <= C.MAX_LINEAR:
            raise RoverError(f"speed must be in [{C.MIN_LINEAR}, {C.MAX_LINEAR}] m/s")
        t = self._preflight()
        x0, y0, yaw0 = t.x, t.y, t.yaw
        sign = 1.0 if distance > 0 else -1.0
        cy, sy = math.cos(yaw0), math.sin(yaw0)

        def step(t: Telemetry, _elapsed: float):
            progress = sign * ((t.x - x0) * cy + (t.y - y0) * sy)
            remaining = abs(distance) - progress
            if remaining <= C.DIST_TOL_M:
                return None
            v = sign * max(C.MIN_LINEAR, min(speed, C.K_DIST * remaining))
            w = _clamp(C.K_HEADING * _wrap(yaw0 - t.yaw), 0.5)  # hold the heading
            return v, w

        def report(t: Telemetry) -> dict:
            progress = sign * ((t.x - x0) * cy + (t.y - y0) * sy)
            return {"requested_m": distance, "error_m": round(abs(distance) - progress, 3)}

        timeout = abs(distance) / speed * 2.0 + 3.0
        return await self._run("move", step, timeout, report)

    async def turn(self, angle_deg: float, speed_deg_s: float | None = None) -> dict:
        _finite(angle_deg=angle_deg, speed_deg_s=speed_deg_s)
        if abs(angle_deg) > C.MAX_TURN_DEG:
            raise RoverError(f"|angle| must be <= {C.MAX_TURN_DEG} deg")
        if abs(angle_deg) < C.YAW_TOL_DEG:
            raise RoverError(f"angle too small (< {C.YAW_TOL_DEG} deg)")
        speed = C.DEFAULT_ANGULAR if speed_deg_s is None else math.radians(abs(speed_deg_s))
        if not C.MIN_ANGULAR <= speed <= C.MAX_ANGULAR:
            raise RoverError(f"speed must be in [{math.degrees(C.MIN_ANGULAR):.0f}, "
                             f"{math.degrees(C.MAX_ANGULAR):.0f}] deg/s")
        t = self._preflight()
        target = math.radians(angle_deg)
        sign = 1.0 if target > 0 else -1.0
        acc = {"turned": 0.0, "last": t.yaw}  # unwrapped yaw so turns beyond 180 deg work

        def turned(t: Telemetry) -> float:
            acc["turned"] += _wrap(t.yaw - acc["last"])
            acc["last"] = t.yaw
            return acc["turned"]

        def step(t: Telemetry, _elapsed: float):
            remaining = abs(target) - sign * turned(t)
            if remaining <= math.radians(C.YAW_TOL_DEG):
                return None
            return 0.0, sign * max(C.MIN_ANGULAR, min(speed, C.K_YAW * remaining))

        def report(t: Telemetry) -> dict:
            return {"requested_deg": angle_deg, "error_deg": round(math.degrees(abs(target) - sign * turned(t)), 1)}

        timeout = abs(target) / speed * 2.0 + 3.0
        return await self._run("turn", step, timeout, report)

    async def stop(self) -> dict:
        was_moving = self._motion_lock.locked()
        self._stop_requested = True
        if self.connected:
            await self._halt()
        # wait for a running motion loop to notice and exit
        for _ in range(20):
            if not self._motion_lock.locked():
                break
            await asyncio.sleep(0.05)
        return {"ok": True, "stopped": True, "was_moving": was_moving, "last_motion": self.last_motion}

    # -- sensors / services -------------------------------------------------------------

    async def snapshot(self, topic: str | None = None) -> tuple[bytes, str, dict]:
        b = self._require()
        try:
            raw, fmt, meta = await b.snapshot(topic)
        except BackendError as e:
            raise RoverError(str(e)) from e
        return _shrink(raw, fmt, meta)

    async def list_topics(self) -> list[dict]:
        try:
            return await self._require().list_topics()
        except BackendError as e:
            raise RoverError(str(e)) from e

    async def reset_odometry(self) -> dict:
        if self._motion_lock.locked():
            raise RoverError("cannot reset odometry while moving")
        try:
            res = await self._require().trigger(C.RESET_ODOM_SERVICE)
        except BackendError as e:
            raise RoverError(str(e)) from e
        await asyncio.sleep(0.3)
        return {"ok": bool(res.get("success", True)), "message": res.get("message", ""), "pose": self.status()["pose"]}


def _shrink(raw: bytes, fmt: str, meta: dict) -> tuple[bytes, str, dict]:
    """Downscale to IMAGE_MAX_WIDTH and re-encode as JPEG to keep the image cheap for the model."""
    try:
        import io

        from PIL import Image
    except ImportError:
        return raw, fmt, meta
    try:
        img = Image.open(io.BytesIO(raw))
        img.load()
    except Exception:
        return raw, fmt, meta  # unknown format: pass through untouched
    meta = {**meta, "original_size": list(img.size)}
    if img.width > C.IMAGE_MAX_WIDTH:
        img = img.resize((C.IMAGE_MAX_WIDTH, round(img.height * C.IMAGE_MAX_WIDTH / img.width)))
    buf = io.BytesIO()
    img.convert("RGB").save(buf, "JPEG", quality=C.IMAGE_QUALITY)
    meta["size"] = list(img.size)
    return buf.getvalue(), "jpeg", meta
