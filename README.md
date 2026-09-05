# Human-Robot Interaction

Maze navigation with OMPL path planning and obstacle avoidance in
[CoppeliaSim](https://www.coppeliarobotics.com/), plus the accompanying
coursework reports on human-robot interaction in industrial automation.

A mobile robot is placed in a walled maze seeded with obstacles. It builds a
collision model of the scene, plans a collision-free route to a goal dummy,
follows that route with a differential-drive controller, and re-plans on the
fly whenever the goal is moved or the direct route is blocked.

---

## Contents

| Path | What it is |
| --- | --- |
| `scenes/Human_Robot_Interaction_Assignment_Simulation1.ttt` | The CoppeliaSim scene: maze, boundaries, obstacles, robot and goal dummy. |
| `scripts/path_following_obstacle_avoidance.lua` | Threaded child script — planning, path following and obstacle avoidance. |
| `tools/run_simulation.py` | Optional Python driver: runs the scene over the ZMQ Remote API and logs the trajectory to CSV/PNG. |
| `tests/smoke_test.lua` | Offline test that exercises the Lua logic against a mock `sim` API — no CoppeliaSim needed. |
| `docs/Human_Robot_Interaction_Assignment_Simulation_Report.pdf` | Report on the simulation: scenario, approach and algorithm walkthrough. |
| `docs/Human_Robot_Interaction_Report_On_Industrial_Automation.pdf` | Report on HRI in industrial automation: findings, recommendations, references. |
| `requirements.txt` | Python dependencies for `tools/`. |

> **Note on the scene file.** CoppeliaSim stores `.ttt` scenes in a compressed
> binary format, so the embedded child script is not readable or diffable in
> Git. `scripts/path_following_obstacle_avoidance.lua` is the reviewable source
> of truth for that logic — it implements the algorithm documented in the
> simulation report. Paste it into the robot's child script, or load it from the
> scene, and keep the two in step when either changes.

---

## Requirements

* **CoppeliaSim 4.6 or newer** (Edu or Pro), with the bundled **OMPL** plugin
  (`simExtOMPL`) enabled — it ships with the standard installation.
* **Lua 5.4** — only for running the offline test suite.
* **Python 3.10+** — only for `tools/run_simulation.py`.

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

---

## Running the simulation

### In the CoppeliaSim GUI

1. Open `scenes/Human_Robot_Interaction_Assignment_Simulation1.ttt`.
2. Confirm the robot carries a **threaded child script** containing
   `scripts/path_following_obstacle_avoidance.lua`.
3. Press **Play**. The robot plans a route and drives to the goal dummy.
4. Drag the goal dummy while the simulation runs — the robot re-plans and
   re-routes towards the new position.

The planned path is drawn in green, the accepted collision-free states in
orange, and the trajectory actually driven in blue.

### From Python

With CoppeliaSim already open (its ZMQ Remote API server listens on port 23000
by default):

```bash
python tools/run_simulation.py --duration 60 --plot
```

This loads the scene, steps the simulation, and writes
`output/trajectory.csv` plus `output/trajectory.png` with the path length
printed to the console. Useful flags:

| Flag | Purpose |
| --- | --- |
| `--robot /robot` | Object path of the robot to log, when the scene renames it. |
| `--no-load-scene` | Use the scene already open instead of loading one. |
| `--host` / `--port` | Point at a CoppeliaSim instance elsewhere. |
| `--output PATH` | Where to write the CSV (the PNG sits beside it). |

`output/` is git-ignored — simulation results are reproducible artefacts, not
source.

---

## How it works

The pipeline mirrors the five-step approach set out in the simulation report:
sense the environment, plan the best route, navigate while avoiding obstacles,
stay efficient, and adapt as things change.

### 1. Setup — `sysCall_init`

Resolves the robot, its reference frame, both wheel motors, the convex
collision proxy and the goal dummy. Every object in the scene is added to an
obstacle collection, then the robot's own tree and the goal dummy are removed
from it, so the robot never treats itself or its target as something to avoid.
The goal dummy is detached from the robot so it can be dragged freely at
runtime, and the drawing containers for the three visualisations are created.

### 2. Collision queries — `checkCollidesAt`

The collision proxy is teleported to a candidate position, tested against the
obstacle collection, and restored. Because the proxy is a kinematic stand-in
rather than the dynamic body, the query never perturbs the physics.

`findFreePositionNear` builds on this, spiralling outwards in 16 directions per
ring until it finds free space. It is used twice: to nudge the robot out of a
start pose that already collides, and to relocate a goal that has been dropped
inside an obstacle — an unreachable goal is repaired rather than rejected.

### 3. Goal acquisition — `getTargetPosition`

Goal samples are buffered and read back one `goalLatency` behind the present,
modelling perception delay, then low-pass filtered. This keeps sensor jitter
from triggering a re-plan on every step; only a displacement larger than
`goalMoveThreshold` counts as the goal genuinely having moved.

### 4. Planning — `planPath`

An OMPL task is built over a 2D position state space bounded to
`searchRange` around the robot, with the collision proxy and obstacle
collection registered as the collision pair. `RRTConnect` runs for up to
`planningTime` seconds per attempt and retries `planningAttempts` times before
reporting failure, returning the path as a flat `{x1, y1, x2, y2, ...}` table.

### 5. Execution — `sysCall_thread`

The main loop reads the perceived goal, repairs it if needed, re-plans when the
goal has drifted, then follows the path waypoint by waypoint. Each waypoint is
retired once the robot is within `waypointTolerance`; the final approach uses
the tighter `goalTolerance`. Heading error drives a proportional differential
term while forward speed is scaled by `cos(heading error)`, so the robot slows
into turns and accelerates on straights instead of stalling one wheel. Wheel
speeds are saturated symmetrically, which preserves the turn ratio at full
throttle.

### 6. Teardown — `sysCall_cleanup`

Motors are zeroed, the goal dummy is re-parented to the robot, and the drawing
objects are removed, leaving the scene exactly as it was found.

### Tuning

All parameters live in the `P` table at the top of the script:

| Parameter | Default | Effect |
| --- | --- | --- |
| `maxWheelSpeed` | `6.0` rad/s | Wheel saturation limit. |
| `cruiseSpeed` | `4.0` rad/s | Nominal forward speed. |
| `headingGain` | `2.5` | Higher turns harder, lower tracks more smoothly. |
| `waypointTolerance` | `0.06` m | Smaller hugs the path; larger cuts corners. |
| `goalTolerance` | `0.08` m | Arrival radius at the goal. |
| `plannerName` | `RRTConnect` | Any algorithm in `simOMPL.Algorithm`. |
| `planningTime` | `4.0` s | Per-attempt planning budget. |
| `searchRange` | `3.0` m | Half-extent of the planning state space — raise for larger mazes. |
| `goalLatency` | `0.25` s | Simulated perception delay. |
| `goalMoveThreshold` | `0.15` m | Goal displacement that forces a re-plan. |

If planning fails on a large maze, raise `searchRange` and `planningTime`
first. If the robot oscillates along the path, lower `headingGain` or raise
`waypointTolerance`.

---

## Testing

CoppeliaSim cannot run headless in CI, so the Lua logic is qualified against a
mock `sim` / `simOMPL` API backed by a small 2D world and a differential-drive
integrator:

```bash
luac5.4 -p scripts/path_following_obstacle_avoidance.lua   # syntax check
lua5.4 tests/smoke_test.lua                                # behavioural check
```

Two scenarios run: a reachable goal with an obstacle across the direct line,
and a goal buried inside an obstacle. Both assert that the planner is invoked,
the robot converges, it does not finish inside an obstacle, the motors stop and
the scene hierarchy is restored.

```
scenario: goal in free space, obstacle on the direct line
  ... 290 step(s) simulated, final distance to goal 0.050 m
scenario: goal inside an obstacle
  ... 275 step(s) simulated, final distance to goal 0.196 m
all checks passed
```

The second scenario ending ~0.20 m from the nominal goal is the expected
result: the goal sits inside the box, so it is relocated to the nearest free
cell just outside it.

These are logic checks against a mock, not a substitute for running the real
scene — dynamics, wheel slip and true OMPL sampling only appear in CoppeliaSim.

---

## Reports

**Simulation report** (`docs/Human_Robot_Interaction_Assignment_Simulation_Report.pdf`)
covers the maze scenario and the approach taken: mapping the environment from
sensor data, planning an optimal route, avoiding obstacles while staying on
that route, keeping motion fast and efficient, and adapting when the world
changes. It then walks through each function of the control script.

**Industrial automation report** (`docs/Human_Robot_Interaction_Report_On_Industrial_Automation.pdf`)
surveys HRI in Industry 4.0 — collaboration, safety and ergonomics,
productivity, intuitive interaction, data-driven optimisation, workforce
transformation and trust — with recommendations on collaborative system design,
reskilling, safety standards and ethics, and a review of articulated and SCARA
robot hardware.

---

## License

Coursework submission. See the repository owner for reuse terms.
