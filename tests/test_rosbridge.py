"""LeoBackend + Rover against a fake rosbridge server that speaks the real JSON protocol and
behaves like LeoOS (odometry integrating cmd_vel, battery, IMU, camera, rosapi, reset_odometry)."""

import asyncio
import base64
import io
import json
import math

import pytest
import websockets
from PIL import Image

from leo_rover_mcp import config as C
from leo_rover_mcp.rover import Rover, RoverError


def _jpeg(w=1280, h=720) -> bytes:
    buf = io.BytesIO()
    Image.new("RGB", (w, h), (10, 120, 30)).save(buf, "JPEG")
    return buf.getvalue()


class FakeLeo:
    def __init__(self, odom_topic="/merged_odom"):
        self.odom_topic = odom_topic
        self.x = self.y = self.yaw = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_count = 0
        self.advertised = {}
        self.subs = {}  # topic -> sub id
        self.server = None
        self.conns = set()

    async def start(self):
        self.server = await websockets.serve(self.handler, "127.0.0.1", 0)
        port = self.server.sockets[0].getsockname()[1]
        return f"ws://127.0.0.1:{port}"

    async def stop(self):
        self.server.close()
        await self.server.wait_closed()

    async def drop_clients(self):
        for ws in list(self.conns):
            await ws.close()

    async def handler(self, ws):
        self.conns.add(ws)
        pump = asyncio.create_task(self.pump(ws))
        try:
            async for raw in ws:
                m = json.loads(raw)
                op = m["op"]
                if op == "advertise":
                    self.advertised[m["topic"]] = m["type"]
                elif op == "publish" and m["topic"] == "/cmd_vel":
                    self.cmd = (m["msg"]["linear"]["x"], m["msg"]["angular"]["z"])
                    self.cmd_count += 1
                elif op == "subscribe":
                    self.subs[m["topic"]] = m["id"]
                    if m["topic"] == "/camera/image_raw/compressed":
                        await ws.send(json.dumps({"op": "publish", "topic": m["topic"], "msg": {
                            "format": "jpeg", "data": base64.b64encode(_jpeg()).decode()}}))
                elif op == "unsubscribe":
                    self.subs.pop(m["topic"], None)
                elif op == "call_service":
                    await ws.send(json.dumps(self.service(m)))
        except websockets.ConnectionClosed:
            pass
        finally:
            pump.cancel()
            self.conns.discard(ws)

    def service(self, m):
        name, values, ok = m["service"], {}, True
        if name == "/rosapi/topics_for_type":
            values = {"topics": ["/camera/image_color/compressed", "/camera/image_raw/compressed"]}
        elif name == "/rosapi/topics":
            values = {"topics": ["/cmd_vel", self.odom_topic], "types": ["geometry_msgs/msg/Twist", "nav_msgs/msg/Odometry"]}
        elif name == "/reset_odometry":
            self.x = self.y = self.yaw = 0.0
            values = {"success": True, "message": ""}
        else:
            ok = False
        return {"op": "service_response", "id": m["id"], "service": name, "values": values, "result": ok}

    async def pump(self, ws):
        dt = 0.05
        while True:
            await asyncio.sleep(dt)
            v, w = self.cmd  # (no firmware timeout here; the client must send zeros itself)
            self.yaw += w * dt
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            out = []
            if self.odom_topic in self.subs:
                out.append((self.odom_topic, {
                    "pose": {"pose": {"position": {"x": self.x, "y": self.y, "z": 0.0},
                                      "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(self.yaw / 2), "w": math.cos(self.yaw / 2)}}},
                    "twist": {"twist": {"linear": {"x": v}, "angular": {"z": w}}}}))
            if "/firmware/battery_averaged" in self.subs:
                out.append(("/firmware/battery_averaged", {"data": 12.1}))
            if "/imu/data" in self.subs:
                s = math.sin(math.radians(5) / 2)
                out.append(("/imu/data", {"orientation": {"x": s, "y": 0.0, "z": 0.0, "w": math.cos(math.radians(5) / 2)}}))
            for topic, msg in out:
                await ws.send(json.dumps({"op": "publish", "topic": topic, "msg": msg}))


def run(coro):
    return asyncio.run(coro)


def test_full_session_over_rosbridge():
    async def go():
        fake = FakeLeo()
        url = await fake.start()
        r = Rover()
        res = await r.connect(simulate=False, url=url)
        assert res["ok"] and res["mode"] == "rosbridge" and res["odom_topic"] == "/merged_odom"
        assert fake.advertised["/cmd_vel"] == "geometry_msgs/msg/Twist"
        await asyncio.sleep(0.1)  # let the fake process the unsubscribe
        assert "/wheel_odom" not in fake.subs  # fallback subscription dropped
        st = r.status()
        assert st["battery_v"] == 12.1 and abs(st["tilt_deg"]["roll"] - 5) < 0.1

        res = await r.move(0.3, 0.2)
        assert res["ok"], res
        assert abs(fake.x - 0.3) < 0.03
        assert fake.cmd == (0.0, 0.0)  # always ends with a zero command
        res = await r.turn(-45)
        assert res["ok"], res
        assert abs(math.degrees(fake.yaw) + 45) < 3

        data, fmt, meta = await r.snapshot()
        assert fmt == "jpeg" and meta["topic"] == "/camera/image_raw/compressed"
        assert meta["original_size"] == [1280, 720] and meta["size"] == [640, 360]
        await asyncio.sleep(0.1)
        assert "/camera/image_raw/compressed" not in fake.subs  # unsubscribed after one frame

        topics = await r.list_topics()
        assert {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist"} in topics
        res = await r.reset_odometry()
        assert res["ok"] and res["pose"]["x_m"] == 0.0
        await r.disconnect()
        await fake.stop()

    run(go())


def test_wheel_odom_fallback():
    async def go():
        fake = FakeLeo(odom_topic="/wheel_odom")
        url = await fake.start()
        r = Rover()
        res = await r.connect(simulate=False, url=url)
        assert res["odom_topic"] == "/wheel_odom"
        await r.disconnect()
        await fake.stop()

    run(go())


def test_connection_loss_aborts_motion():
    async def go():
        fake = FakeLeo()
        url = await fake.start()
        r = Rover()
        await r.connect(simulate=False, url=url)
        task = asyncio.create_task(r.move(1.0, 0.2))
        await asyncio.sleep(0.8)
        await fake.drop_clients()
        res = await task
        assert res["outcome"] == "aborted" and not res["ok"]
        with pytest.raises(RoverError, match="connection"):
            await r.move(0.2)
        # reconnect works
        res = await r.connect(simulate=False, url=url)
        assert res["ok"]
        await r.disconnect()
        await fake.stop()

    run(go())


def test_unreachable_rover():
    async def go():
        r = Rover()
        with pytest.raises(RoverError, match="cannot reach"):
            await r.connect(simulate=False, url="ws://127.0.0.1:1")

    run(go())


def test_no_odometry_is_refused(monkeypatch):
    monkeypatch.setattr(C, "ODOM_TOPICS", ["nonexistent_odom"])

    async def go():
        fake = FakeLeo()
        url = await fake.start()
        r = Rover()
        with pytest.raises(RoverError, match="no odometry"):
            await r.connect(simulate=False, url=url)
        assert not r.connected
        await fake.stop()

    run(go())
