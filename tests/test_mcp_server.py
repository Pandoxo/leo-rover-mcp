"""End to end: spawn the MCP server over stdio (simulated rover) and drive it like an agent would."""

import asyncio
import json
import os
import sys
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

ROOT = Path(__file__).resolve().parents[1]


def _params() -> StdioServerParameters:
    env = {**os.environ, "LEO_DRY_RUN": "1", "PYTHONPATH": str(ROOT), "PYTHONNOUSERSITE": "1"}
    return StdioServerParameters(command=sys.executable, args=["-m", "leo_rover_mcp.mcp_server"], env=env)


def _json(res) -> dict:
    return json.loads(next(c.text for c in res.content if c.type == "text"))


def test_agent_session():
    async def go():
        async with stdio_client(_params()) as (read, write), ClientSession(read, write) as s:
            init = await s.initialize()
            assert "rover_camera" in init.instructions
            names = {t.name for t in (await s.list_tools()).tools}
            assert names == {"rover_info", "rover_connect", "rover_status", "rover_camera", "rover_move", "rover_turn",
                             "rover_drive", "rover_stop", "rover_reset_odometry", "rover_list_topics", "rover_disconnect"}

            assert _json(await s.call_tool("rover_move", {"distance_m": 0.5}))["ok"] is False  # not connected yet
            assert _json(await s.call_tool("rover_connect", {}))["mode"] == "simulation"

            cam = await s.call_tool("rover_camera", {})
            kinds = [c.type for c in cam.content]
            assert kinds == ["image", "text"] and cam.content[0].mimeType == "image/jpeg"

            res = _json(await s.call_tool("rover_turn", {"angle_deg": 90}))
            assert res["ok"] and abs(res["pose"]["heading_deg"] - 90) < 4

            # stop arrives while a move is running and interrupts it
            move = asyncio.create_task(s.call_tool("rover_move", {"distance_m": 1.0, "speed_m_s": 0.2}))
            await asyncio.sleep(1.5)
            assert _json(await s.call_tool("rover_stop", {}))["ok"]
            res = _json(await move)
            assert res["outcome"] == "stopped", res

            bad = _json(await s.call_tool("rover_move", {"distance_m": 50}))
            assert bad["ok"] is False and "distance" in bad["error"]
            assert _json(await s.call_tool("rover_disconnect", {}))["ok"]

    asyncio.run(go())
