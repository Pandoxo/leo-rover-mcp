"""Minimal asyncio client for the rosbridge v2 JSON protocol (no ROS needed on this machine).

Only what the rover needs: advertise/publish, subscribe/unsubscribe and call_service.
"""

from __future__ import annotations

import asyncio
import itertools
import json
import logging
from collections.abc import Callable
from typing import Any

import websockets

log = logging.getLogger("leo_rover_mcp.rosbridge")


class RosbridgeError(RuntimeError):
    pass


class RosbridgeClient:
    def __init__(self, url: str):
        self.url = url
        self._ws: Any = None
        self._reader: asyncio.Task | None = None
        self._ids = itertools.count(1)
        self._subs: dict[str, dict[str, Callable[[dict], None]]] = {}  # topic -> {sub id: callback}
        self._calls: dict[str, asyncio.Future] = {}
        self._advertised: set[str] = set()
        self._send_lock = asyncio.Lock()
        self.close_reason: str | None = None

    @property
    def connected(self) -> bool:
        return self._ws is not None and self._reader is not None and not self._reader.done()

    async def connect(self, timeout: float = 5.0) -> None:
        try:
            self._ws = await asyncio.wait_for(
                websockets.connect(self.url, max_size=32 * 1024 * 1024, ping_interval=5, ping_timeout=5),
                timeout,
            )
        except (OSError, asyncio.TimeoutError, websockets.InvalidURI, websockets.InvalidHandshake) as e:
            raise RosbridgeError(f"cannot reach rosbridge at {self.url}: {e!r}") from e
        self.close_reason = None
        self._reader = asyncio.create_task(self._read_loop())

    async def close(self) -> None:
        if self._ws is not None:
            await self._ws.close()
        if self._reader is not None:
            await asyncio.gather(self._reader, return_exceptions=True)
        self._ws = None
        self._reader = None
        self._subs.clear()
        self._advertised.clear()

    # -- protocol ---------------------------------------------------------------

    async def _send(self, msg: dict) -> None:
        if not self.connected:
            raise RosbridgeError(f"not connected to rosbridge ({self.close_reason or 'closed'})")
        async with self._send_lock:
            await self._ws.send(json.dumps(msg))

    async def advertise(self, topic: str, msg_type: str) -> None:
        if topic in self._advertised:
            return
        await self._send({"op": "advertise", "topic": topic, "type": msg_type, "queue_size": 1})
        self._advertised.add(topic)

    async def publish(self, topic: str, msg: dict) -> None:
        await self._send({"op": "publish", "topic": topic, "msg": msg})

    async def subscribe(
        self, topic: str, msg_type: str, callback: Callable[[dict], None], throttle_ms: int = 0
    ) -> str:
        sub_id = f"sub:{topic}:{next(self._ids)}"
        self._subs.setdefault(topic, {})[sub_id] = callback
        await self._send(
            {"op": "subscribe", "id": sub_id, "topic": topic, "type": msg_type,
             "throttle_rate": throttle_ms, "queue_length": 1}
        )
        return sub_id

    async def unsubscribe(self, topic: str, sub_id: str) -> None:
        self._subs.get(topic, {}).pop(sub_id, None)
        if self.connected:
            await self._send({"op": "unsubscribe", "id": sub_id, "topic": topic})

    async def call_service(self, service: str, args: dict | None = None, timeout: float = 5.0) -> dict:
        call_id = f"call:{service}:{next(self._ids)}"
        fut = asyncio.get_running_loop().create_future()
        self._calls[call_id] = fut
        try:
            await self._send({"op": "call_service", "id": call_id, "service": service, "args": args or {}})
            resp = await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError as e:
            raise RosbridgeError(f"service {service} did not answer within {timeout:.0f} s") from e
        finally:
            self._calls.pop(call_id, None)
        if not resp.get("result", False):
            raise RosbridgeError(f"service {service} failed: {resp.get('values')}")
        return resp.get("values") or {}

    async def _read_loop(self) -> None:
        try:
            async for raw in self._ws:
                try:
                    msg = json.loads(raw)
                except (TypeError, ValueError):
                    continue
                op = msg.get("op")
                if op == "publish":
                    for cb in list(self._subs.get(msg.get("topic"), {}).values()):
                        try:
                            cb(msg.get("msg") or {})
                        except Exception:  # a bad callback must not kill the connection
                            log.exception("subscriber callback failed")
                elif op == "service_response":
                    fut = self._calls.get(msg.get("id"))
                    if fut is not None and not fut.done():
                        fut.set_result(msg)
                elif op == "status" and msg.get("level") in ("error", "warning"):
                    log.warning("rosbridge %s: %s", msg.get("level"), msg.get("msg"))
            self.close_reason = "connection closed by rosbridge"
        except websockets.ConnectionClosed as e:
            self.close_reason = f"connection lost: {e}"
        finally:
            err = RosbridgeError(f"rosbridge {self.close_reason or 'closed'}")
            for fut in self._calls.values():
                if not fut.done():
                    fut.set_exception(err)
