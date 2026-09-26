"""MCP server that lets an AI agent drive a Leo Rover (LeoOS, ROS 2) through rosbridge.

Run (stdio transport):  python -m leo_rover_mcp.mcp_server

Environment:
  LEO_URL=ws://10.0.0.1:9090   rosbridge on the rover (the stock web UI uses the same one)
  LEO_DRY_RUN=1                simulated rover by default (rover_connect(simulate=...) overrides)
  LEO_NAMESPACE                ROS namespace of the rover, if any
  LEO_MAX_LINEAR (0.4 m/s), LEO_MAX_ANGULAR (1.0 rad/s), LEO_MAX_DISTANCE (3 m), LEO_MAX_DURATION (10 s)
  LEO_BATTERY_MIN (10.4 V), LEO_CAMERA_TOPIC (auto), LEO_IMAGE_WIDTH (640)
  see leo_rover_mcp/config.py for the rest.

Tools return {"ok": true, ...} or {"ok": false, "error": "..."}; refusals are normal, read them and adjust.
"""

from __future__ import annotations

import logging
import math
import signal
import sys

from mcp.server.fastmcp import FastMCP, Image

from . import config as C
from .rover import Rover, RoverError

# stdout belongs to the MCP protocol: log to stderr only
logging.basicConfig(stream=sys.stderr, level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
log = logging.getLogger("leo_rover_mcp.mcp")

INSTRUCTIONS = """\
Drives a real Leo Rover (4-wheel skid-steer, ~0.45 x 0.43 m, ~6.5 kg). It can hit people, pets, furniture
and fall down stairs. It has NO obstacle sensors: the only way to see is rover_camera.
Workflow: rover_info -> rover_connect -> rover_status -> rover_camera (look) -> rover_turn / rover_move -> rover_camera ...
Mode: check "mode" in the rover_connect result. "simulation" = simulated room, rover_camera returns a top-down map
(not a camera view); "rosbridge" = the REAL rover. rover_info.dry_run_default says which one rover_connect() picks;
pass simulate=true/false to choose explicitly. To switch modes call rover_disconnect first.
Rules:
- Look before every move with rover_camera and keep moves short (<= ~1 m) in unknown space.
  The camera looks forward; nothing behind or right beside the rover is visible, so reverse only a little.
- Frame: odometry frame, metres, x forward / y left at the last odometry reset; heading in degrees, CCW positive.
  rover_turn angle > 0 turns left, < 0 right. rover_move distance < 0 drives backwards.
- Prefer rover_move / rover_turn (closed loop on odometry). rover_drive is open-loop velocity for a time.
- A motion ending with outcome 'stalled' means the wheels are blocked: look with the camera, back off, go around.
- Odometry drifts (wheel slip, carpets): re-check with the camera instead of trusting the pose blindly.
- If anything looks wrong call rover_stop. When finished call rover_disconnect.
"""

mcp = FastMCP("leo-rover", instructions=INSTRUCTIONS)
_rover = Rover()


async def _call(coro) -> dict:
    try:
        out = await coro
        return out if isinstance(out, dict) else {"ok": True, "result": out}
    except RoverError as e:
        return {"ok": False, "error": str(e)}
    except (ValueError, TypeError) as e:
        return {"ok": False, "error": f"invalid request: {e}"}
    except Exception as e:  # pragma: no cover - unexpected transport failure
        log.exception("unexpected error")
        return {"ok": False, "error": f"unexpected error: {e!r}"}


async def _sync(fn) -> dict:
    async def wrap():
        return fn()

    return await _call(wrap())


@mcp.tool()
def rover_info() -> dict:
    """Static facts about the rover, frames, limits and configuration. No motion."""
    return {
        "robot": "Leo Rover: 4 in-wheel motors, skid-steer (differential) drive, LeoOS on a Raspberry Pi, ROS 2",
        "size": "about 0.45 m long x 0.43 m wide x 0.25 m high; wheel radius 0.0625 m, track 0.358 m",
        "sensors": "wheel odometry + IMU, forward-facing camera; no lidar, bumpers or cliff sensors",
        "frame": "odometry frame: metres, x forward / y left at the last odometry reset, heading deg CCW+",
        "limits": {
            "max_linear_m_s": C.MAX_LINEAR, "default_linear_m_s": C.DEFAULT_LINEAR,
            "max_angular_deg_s": round(math.degrees(C.MAX_ANGULAR)),
            "default_angular_deg_s": round(math.degrees(C.DEFAULT_ANGULAR)),
            "max_distance_per_move_m": C.MAX_DISTANCE_M, "max_drive_duration_s": C.MAX_DURATION_S,
            "battery_min_v": C.BATTERY_MIN_V, "max_tilt_deg": C.TILT_MAX_DEG,
        },
        "safety": [
            "cmd_vel is re-sent at 10 Hz; the rover's firmware stops the wheels 0.5 s after the last command, "
            "so a crashed server or lost Wi-Fi stops the rover",
            f"a motion is aborted when the wheels do not move for {C.STALL_TIME_S} s (blocked) or odometry stops",
            "stall detection only catches blocked wheels, not wheels spinning in place (slip)",
        ],
        "rosbridge_url": C.URL,
        "dry_run_default": C.DRY_RUN,
    }


@mcp.tool()
async def rover_connect(simulate: bool | None = None, url: str | None = None) -> dict:
    """Connect to the rover's rosbridge (default LEO_URL, ws://10.0.0.1:9090) and start reading telemetry.

    simulate: True = simulated rover in a 6 x 5 m room with boxes (its 'camera' is a top-down map),
    False = the real rover; omitted = LEO_DRY_RUN decides (see rover_info.dry_run_default), a given url means real.
    The result's "mode" is "simulation" or "rosbridge" (real). Already connected: returns the current state, or an
    error if a different mode / url was asked for (rover_disconnect first).
    Leaves the rover standing still. Fails if no odometry arrives.
    """
    return await _call(_rover.connect(simulate, url))


@mcp.tool()
async def rover_status() -> dict:
    """Pose (odometry), measured velocity, battery voltage, tilt, data freshness, last motion outcome, warnings."""
    return await _sync(lambda: {"ok": True, **_rover.status()})


@mcp.tool(structured_output=False)
async def rover_camera(topic: str | None = None) -> list:
    """Take one picture with the rover's camera (downscaled JPEG) plus the current status.

    topic: a sensor_msgs/CompressedImage topic; default LEO_CAMERA_TOPIC or the first camera topic found.
    """
    try:
        data, fmt, meta = await _rover.snapshot(topic)
    except RoverError as e:
        return [{"ok": False, "error": str(e)}]
    status = await _sync(lambda: {"ok": True, **_rover.status()})
    return [Image(data=data, format=fmt), {**status, "camera": meta}]


@mcp.tool()
async def rover_move(distance_m: float, speed_m_s: float | None = None) -> dict:
    """Drive straight by distance_m (negative = backwards) holding the current heading, closed loop on odometry.

    Blocks until done and returns what really happened: outcome done / stalled / stopped / timeout / aborted,
    moved (distance, forward, turned), error_m (remaining distance) and the new pose. ok is true only for done.
    """
    return await _call(_rover.move(distance_m, speed_m_s))


@mcp.tool()
async def rover_turn(angle_deg: float, speed_deg_s: float | None = None) -> dict:
    """Turn in place by angle_deg (positive = left / counter-clockwise, negative = right), up to 360 deg.

    Closed loop on odometry; returns outcome, moved.turned_deg, error_deg and the new pose like rover_move.
    """
    return await _call(_rover.turn(angle_deg, speed_deg_s))


@mcp.tool()
async def rover_drive(linear_m_s: float, angular_deg_s: float, duration_s: float) -> dict:
    """Open-loop: hold a velocity (forward m/s, turn rate deg/s, left positive) for duration_s, then stop.

    Use for arcs; for exact distances or angles prefer rover_move / rover_turn.
    """
    return await _call(_rover.drive(linear_m_s, angular_deg_s, duration_s))


@mcp.tool()
async def rover_stop() -> dict:
    """Stop immediately: interrupts any running motion and commands zero velocity. Safe to call any time."""
    return await _call(_rover.stop())


@mcp.tool()
async def rover_reset_odometry() -> dict:
    """Make the current pose the origin (x=0, y=0, heading=0) of the odometry frame."""
    return await _call(_rover.reset_odometry())


@mcp.tool()
async def rover_list_topics(contains: str | None = None) -> dict:
    """List ROS topics on the rover (via rosapi), optionally filtered by a substring. For diagnostics."""

    async def run():
        topics = await _rover.list_topics()
        if contains:
            topics = [t for t in topics if contains in t["name"] or contains in t["type"]]
        return {"ok": True, "topics": topics}

    return await _call(run())


@mcp.tool()
async def rover_disconnect() -> dict:
    """Finish: stop the rover and close the connection."""
    return await _call(_rover.disconnect())


# --------------------------------------------------------------------------


def main() -> None:
    # no cleanup needed on exit: without cmd_vel the firmware stops the wheels after 0.5 s
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    mcp.run(transport="stdio")


if __name__ == "__main__":
    main()
