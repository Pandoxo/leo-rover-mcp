# Driving the Leo Rover through MCP — manual for AI agents

This file is for an agent that has the `leo-rover` MCP server connected (tools named `rover_*`, in Claude Code
`mcp__leo-rover__rover_*`). It covers how to drive, how to read results, and what to do when things go wrong.
For setup see [README.md](README.md).

## 1. What you are controlling

* A **real** 6.5 kg, 0.45 × 0.43 m four-wheel skid-steer robot (unless the mode is `simulation`, see §2). It can
  hit people, pets and furniture and it can fall down stairs.
* It has **no obstacle, bumper or cliff sensors**. Your only way to perceive the world is `rover_camera`, which
  looks **forward only**. You cannot see behind or directly beside the rover.
* Position comes from wheel odometry + IMU. It drifts (wheel slip, carpets, bumps). Treat the pose as an
  estimate; verify with the camera.

## 2. Simulation or real — always check

`rover_connect` returns `"mode"`:

| `mode` | Meaning | What `rover_camera` returns |
|---|---|---|
| `simulation` | simulated 6 × 5 m room with 3 boxes, nothing physical moves | a **top-down map**: grey floor, 1 m grid, brown boxes, blue circle = rover, red line = heading |
| `rosbridge` | the **real** rover | a real forward-facing camera photo |

* `rover_info().dry_run_default` tells you which mode `rover_connect()` without arguments will pick.
* Force a mode with `rover_connect(simulate=true)` / `rover_connect(simulate=false)`.
* Passing `url` implies the real rover.
* If already connected, `rover_connect` with a *different* mode or url returns an error; call `rover_disconnect`
  first. Called with no arguments, it just returns the current state.
* **Do not drive the real rover unless the user asked for the real rover.** When in doubt, ask.

## 3. Standard loop

```
rover_info                         # limits, frame, default mode (no motion)
rover_connect(simulate=...)        # check "mode" and "odom_topic" in the result
rover_status                       # battery_v, warnings, data_age_s
loop:
    rover_camera                   # LOOK first, every time
    decide: is the path ahead clear for the distance I want?
    rover_turn(angle_deg)          # face the direction
    rover_camera                   # look again after turning
    rover_move(distance_m)         # short step, <= ~1 m in unknown space
    read the result: outcome, moved, error_m, pose
rover_disconnect                   # always, when finished
```

Rules of thumb:

* **Look before every move.** After a turn the view changes — look again before driving.
* **Short steps.** ≤ 1 m in unknown space, ≤ 0.3 m near obstacles or people. Max per call is 3 m anyway.
* **Reverse only a little** (≤ 0.3 m): you cannot see behind.
* **Prefer `rover_move` / `rover_turn`** (closed loop on odometry). Use `rover_drive` only for arcs or when you
  need a raw velocity; it is open loop — it holds a velocity for `duration_s` regardless of where it ends up.
* Use slow speeds near obstacles: `rover_move(0.3, speed_m_s=0.1)`, `rover_turn(45, speed_deg_s=20)`.
* If anything looks wrong, call **`rover_stop`**. It interrupts a running motion from another call too.

## 4. Frames and sign conventions

* Odometry frame, metres. At the last odometry reset: **x forward, y left**. Heading in degrees, **CCW
  positive**, range (−180, 180].
* `rover_turn(angle_deg)`: `> 0` = turn **left**, `< 0` = turn **right**. Up to ±360°.
* `rover_move(distance_m)`: `> 0` forward, `< 0` backward. Holds the heading it started with.
* `rover_drive(linear_m_s, angular_deg_s, duration_s)`: `angular_deg_s > 0` curves left.
* `rover_reset_odometry` makes the current pose (0, 0, 0°). Handy before a task so coordinates are intuitive.

To reach a point (gx, gy) from pose (x, y, h): `bearing = atan2(gy − y, gx − x)` in degrees, turn by
`wrap(bearing − h)` into (−180, 180], look, then move `hypot(gx − x, gy − y)` in short steps.

## 5. Reading results

Every tool returns `{"ok": true, ...}` or `{"ok": false, "error": "..."}`. A refusal is normal: read the error
and adjust (e.g. "|distance| must be <= 3.0 m per call; split longer drives").

Motion results (`rover_move`, `rover_turn`, `rover_drive`):

```json
{"ok": true, "outcome": "done", "elapsed_s": 3.1,
 "moved": {"distance_m": 0.5, "forward_m": 0.5, "turned_deg": 0.3},
 "requested_m": 0.5, "error_m": 0.004,
 "pose": {"x_m": 0.5, "y_m": 0.0, "heading_deg": 0.3}}
```

`ok` is true only when `outcome` is `done`.

| `outcome` | Meaning | What to do |
|---|---|---|
| `done` | goal reached (within 1 cm / 1°) | continue |
| `stalled` | commanded to move but odometry showed no motion for 1.5 s: **blocked** | `rover_camera`, back off a little (`rover_move(-0.2)`), turn, go around |
| `stopped` | `rover_stop` was called | intended |
| `timeout` | goal not reached in time (slipping, heavy load, very slow speed) | look, check `moved`, retry with a shorter step |
| `aborted` | connection lost or odometry stopped updating | `rover_status`; if disconnected, `rover_connect` again |

Stall detection catches blocked wheels, **not** spinning wheels: if `moved` looks fine but the camera shows you
did not go anywhere, the wheels are slipping — stop and rethink.

`rover_status` fields: `pose`, `velocity`, `battery_v`, `tilt_deg` (roll/pitch), `data_age_s` (seconds since the
last odom/battery/IMU message), `moving`, `last_motion`, `warnings`.

## 6. Refusals and warnings you may see

| Message | Cause | Action |
|---|---|---|
| `not connected: call rover_connect first` | — | `rover_connect` |
| `odometry is stale or missing` | no odom for > 0.6 s | `rover_status`; rover's ROS nodes may be down; tell the user |
| `battery at X V (< 10.4 V)` | battery flat | stop, tell the user to charge |
| `rover tilted N deg` | > 30° roll/pitch | stop, tell the user — it may be stuck on something or about to tip |
| `another motion is running` | a motion is still executing | wait or `rover_stop` |
| `already connected in mode ...` | mode/url switch requested | `rover_disconnect`, then `rover_connect` |
| `no image on ... within 4 s` / `no CompressedImage topic` | camera not running | `rover_list_topics(contains="camera")`, try `rover_camera(topic=...)`; tell the user |
| `cannot reach rosbridge at ...` | not on the rover's Wi-Fi / wrong IP | tell the user to join `LeoRover-XXXX` and check `http://10.0.0.1` |

## 7. Limits (defaults)

| | |
|---|---|
| linear speed | default 0.2, max 0.4 m/s (min 0.04 for `rover_move`) |
| turn rate | default ~34, max ~57 °/s (min ~9 for `rover_turn`) |
| `rover_move` | ≤ 3 m per call |
| `rover_drive` | ≤ 10 s per call |
| `rover_turn` | ≤ 360° per call |

Exact values for the running server: `rover_info().limits`.

## 8. Example (simulation)

```
rover_connect(simulate=true)          -> mode "simulation", pose (0, 0, 0°)
rover_camera                          -> map: box ahead at x 1.2–1.6 m
rover_move(0.8)                       -> done, x 0.8
rover_turn(90)                        -> done, heading 90°
rover_camera                          -> path to the left is clear
rover_move(0.8)                       -> done, (0.8, 0.8)
rover_turn(-90); rover_move(1.2)      -> passes left of the box
rover_disconnect
```

In the simulator, driving into a box or wall ends with `stalled` — good practice for recovery.

## 9. Never

* Never drive the real rover without the user's request, or keep driving after the user says stop.
* Never do long moves or long `rover_drive` in space you have not seen.
* Never assume the pose is exact after many moves; re-check with the camera.
* Never leave the session connected: finish with `rover_disconnect`.
