"""Rover logic against the simulated backend (real time, a few seconds per test)."""

import asyncio
import math

import pytest

from leo_rover_mcp.rover import Rover, RoverError


def run(coro):
    return asyncio.run(coro)


async def _connected() -> Rover:
    r = Rover()
    res = await r.connect(simulate=True)
    assert res["ok"] and res["mode"] == "simulation"
    return r


def test_move_forward_and_back():
    async def go():
        r = await _connected()
        res = await r.move(0.5, 0.3)
        assert res["outcome"] == "done", res
        assert abs(res["moved"]["forward_m"] - 0.5) < 0.04
        res = await r.move(-0.3)
        assert res["outcome"] == "done", res
        assert abs(r.status()["pose"]["x_m"] - 0.2) < 0.05
        await r.disconnect()

    run(go())


def test_turn_left_right_and_beyond_180():
    async def go():
        r = await _connected()
        res = await r.turn(90)
        assert res["ok"], res
        assert abs(r.status()["pose"]["heading_deg"] - 90) < 4
        res = await r.turn(-200, 57)
        assert res["ok"], res
        assert abs(res["moved"]["turned_deg"] - 160) < 4  # -200 wraps to +160
        await r.disconnect()

    run(go())


def test_drive_open_loop_arc():
    async def go():
        r = await _connected()
        res = await r.drive(0.2, 30, 2.0)
        assert res["ok"], res
        assert res["moved"]["distance_m"] > 0.2 and res["moved"]["turned_deg"] > 20
        assert abs(r.backend.telemetry.v) < 0.05  # stopped afterwards
        await r.disconnect()

    run(go())


def test_stall_against_box():
    async def go():
        r = await _connected()
        res = await r.move(2.0, 0.3)  # box starts at x = 1.2, rover radius 0.25
        assert res["outcome"] == "stalled", res
        assert not res["ok"]
        assert 0.85 < r.status()["pose"]["x_m"] < 0.96
        res = await r.move(-0.3)  # backing off works
        assert res["ok"], res
        await r.disconnect()

    run(go())


def test_stop_interrupts_motion():
    async def go():
        r = await _connected()
        task = asyncio.create_task(r.move(2.0, 0.2))
        await asyncio.sleep(1.5)
        await r.stop()
        res = await task
        assert res["outcome"] == "stopped"
        assert res["moved"]["distance_m"] < 0.5
        assert r.last_motion["outcome"] == "stopped"
        await r.disconnect()

    run(go())


def test_refusals():
    async def go():
        r = Rover()
        with pytest.raises(RoverError, match="not connected"):
            await r.move(0.5)
        r = await _connected()
        for call in (r.move(10), r.move(0.5, 5.0), r.turn(720), r.drive(0.2, 0, 60), r.drive(2.0, 0, 1),
                     r.drive(0, 500, 1)):
            with pytest.raises(RoverError):
                await call
        task = asyncio.create_task(r.move(0.5))
        await asyncio.sleep(0.3)
        with pytest.raises(RoverError, match="another motion"):
            await r.turn(30)
        await task
        r.backend.telemetry.battery_v = 9.9
        r.backend._battery = 9.9
        with pytest.raises(RoverError, match="battery"):
            await r.move(0.5)
        await r.disconnect()

    run(go())


def test_reset_odometry_and_camera():
    async def go():
        r = await _connected()
        await r.turn(45)
        await r.move(0.4)
        res = await r.reset_odometry()
        pose = res["pose"]
        assert abs(pose["x_m"]) < 0.01 and abs(pose["y_m"]) < 0.01 and abs(pose["heading_deg"]) < 1
        await r.move(0.3)
        p = r.status()["pose"]
        assert abs(p["x_m"] - 0.3) < 0.04 and abs(p["y_m"]) < 0.03  # new frame is aligned with the rover
        data, fmt, meta = await r.snapshot()
        assert fmt == "jpeg" and data[:2] == b"\xff\xd8" and meta["size"][0] <= 640
        await r.disconnect()

    run(go())


def test_heading_hold():
    async def go():
        r = await _connected()
        await r.turn(-30)
        res = await r.move(0.8, 0.4)
        assert res["ok"], res
        assert abs(res["moved"]["turned_deg"]) < 3
        p = r.status()["pose"]
        assert abs(math.atan2(p["y_m"], p["x_m"]) - math.radians(-30)) < math.radians(4)
        await r.disconnect()

    run(go())


def test_non_finite_arguments_are_refused():
    async def go():
        r = await _connected()
        for call in (r.move(float("nan")), r.move(0.5, float("inf")), r.turn(float("nan")),
                     r.drive(0.1, 0, float("nan"))):
            with pytest.raises(RoverError, match="finite"):
                await call
        assert r.last_motion is None  # nothing moved
        await r.disconnect()

    run(go())


def test_connect_refuses_silent_mode_switch():
    async def go():
        r = await _connected()
        assert (await r.connect())["already_connected"]
        assert (await r.connect(simulate=True))["already_connected"]
        with pytest.raises(RoverError, match="rover_disconnect"):
            await r.connect(simulate=False)
        with pytest.raises(RoverError, match="rover_disconnect"):
            await r.connect(url="ws://192.0.2.1:9090")
        await r.disconnect()

    run(go())


def test_camera_topic_preference():
    from leo_rover_mcp.backend import _pick_camera_topic

    leo = ["/camera/image_color/compressed", "/camera/image_mono/compressed", "/camera/image_rect/compressed",
           "/camera/image_rect_color/compressed"]
    assert _pick_camera_topic(leo) == "/camera/image_color/compressed"
    assert _pick_camera_topic(leo + ["/camera/image_raw/compressed"]) == "/camera/image_raw/compressed"
    assert _pick_camera_topic(["/camera/image_mono/compressed", "/usb_cam/x/compressed"]) == "/usb_cam/x/compressed"
    assert _pick_camera_topic(["/camera/image_mono/compressed"]) == "/camera/image_mono/compressed"
