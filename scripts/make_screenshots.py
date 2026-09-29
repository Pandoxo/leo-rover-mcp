"""Drive the simulated rover along a short route and save screenshots + an animated GIF to docs/.

    .venv/bin/python scripts/make_screenshots.py
"""

import asyncio
import io
from pathlib import Path

from PIL import Image, ImageDraw

from leo_rover_mcp.rover import Rover
from leo_rover_mcp.sim import ROOM

OUT = Path(__file__).resolve().parent.parent / "docs"
SCALE = 100


def px(x: float, y: float) -> tuple[float, float]:
    return (x - ROOM[0]) * SCALE, (ROOM[3] - y) * SCALE


async def frame(rover: Rover, trail: list, draw_trail: bool) -> Image.Image:
    data, _, _ = await rover.snapshot()
    img = Image.open(io.BytesIO(data)).convert("RGB")
    if draw_trail and len(trail) > 1:
        # redraw the rover on top of its own trail
        sim = rover._require()
        d = ImageDraw.Draw(img, "RGBA")
        d.line([px(*p) for p in trail], fill=(40, 110, 200, 170), width=4, joint="curve")
        cx, cy = px(sim._x, sim._y)
        r = 25
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(40, 110, 200))
        import math
        d.line([(cx, cy), (cx + 40 * math.cos(sim._yaw), cy - 40 * math.sin(sim._yaw))], fill=(230, 60, 40), width=5)
    return img


async def main() -> None:
    OUT.mkdir(exist_ok=True)
    rover = Rover()
    await rover.connect(simulate=True)
    sim = rover._require()
    trail: list[tuple[float, float]] = [(sim._x, sim._y)]
    frames: list[Image.Image] = []

    (await frame(rover, trail, False)).save(OUT / "sim-start.png")

    async def run(coro):
        task = asyncio.create_task(coro)
        while not task.done():
            trail.append((sim._x, sim._y))
            frames.append((await frame(rover, trail, True)).resize((600, 500)))
            await asyncio.sleep(0.12)
        return task.result()

    route = [("turn", -60), ("move", 1.0), ("turn", 60), ("move", 1.6), ("turn", 90), ("move", 1.6), ("turn", 90),
             ("move", 1.4), ("turn", 100)]
    for i, (kind, val) in enumerate(route):
        res = await run(rover.turn(val) if kind == "turn" else rover.move(val, 0.3))
        print(kind, val, res.get("outcome"))
        if i == 3:
            (await frame(rover, trail, True)).save(OUT / "sim-mid-route.png")

    (await frame(rover, trail, True)).save(OUT / "sim-route.png")

    await rover.disconnect()

    # fresh run straight into the first box: stall detection
    rover = Rover()
    await rover.connect(simulate=True)
    res = await rover.move(2.0, 0.3)
    print("bump:", res.get("outcome"))
    (await frame(rover, [], False)).save(OUT / "sim-stalled.png")
    await rover.disconnect()

    frames[0].save(OUT / "sim-demo.gif", save_all=True, append_images=frames[3::3], duration=100, loop=0, optimize=True)
    print(len(frames), "frames")


asyncio.run(main())
