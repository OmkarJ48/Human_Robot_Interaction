#!/usr/bin/env python3
"""Drive the HRI maze scene from Python and record the robot's trajectory.

Connects to a running CoppeliaSim instance over the ZeroMQ Remote API,
loads ``scenes/Human_Robot_Interaction_Assignment_Simulation1.ttt``, runs the
simulation while sampling the robot pose, and writes the trajectory to CSV
(optionally plotting it).

Start CoppeliaSim first -- the ZMQ Remote API server is enabled by default on
port 23000 in CoppeliaSim 4.6+ -- then:

    python tools/run_simulation.py --duration 60 --plot
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SCENE = REPO_ROOT / "scenes" / "Human_Robot_Interaction_Assignment_Simulation1.ttt"
DEFAULT_OUTPUT = REPO_ROOT / "output" / "trajectory.csv"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="localhost", help="CoppeliaSim host")
    parser.add_argument("--port", type=int, default=23000, help="ZMQ Remote API port")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE, help="scene to load")
    parser.add_argument(
        "--robot",
        default="/robot",
        help="object path of the robot whose pose is logged",
    )
    parser.add_argument(
        "--duration", type=float, default=120.0, help="max simulated seconds to run"
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="CSV output path")
    parser.add_argument("--plot", action="store_true", help="save a PNG of the trajectory")
    parser.add_argument(
        "--no-load-scene",
        action="store_true",
        help="use the scene already open in CoppeliaSim instead of loading one",
    )
    return parser.parse_args(argv)


def connect(host: str, port: int):
    """Return a (client, sim) pair, or exit with a readable message."""
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    except ImportError:
        sys.exit(
            "coppeliasim-zmqremoteapi-client is not installed.\n"
            "Install the dependencies with:  pip install -r requirements.txt"
        )

    try:
        client = RemoteAPIClient(host=host, port=port)
        return client, client.require("sim")
    except Exception as exc:  # noqa: BLE001 - surface any transport failure plainly
        sys.exit(
            f"could not reach CoppeliaSim at {host}:{port} ({exc}).\n"
            "Start CoppeliaSim and make sure the ZMQ Remote API server is running."
        )


def resolve_robot(sim, preferred: str) -> int:
    """Find the robot handle, tolerating a few common scene naming choices."""
    candidates = [preferred, "/robot", "/Robot", "/PioneerP3DX", "/youBot"]
    for path in candidates:
        try:
            return sim.getObject(path)
        except Exception:  # noqa: BLE001 - getObject raises when the path is absent
            continue
    sys.exit(
        f"no robot found at {preferred!r} (also tried: {', '.join(candidates[1:])}).\n"
        "Pass the correct object path with --robot."
    )


def run(sim, robot: int, duration: float) -> list[tuple[float, float, float, float]]:
    """Step the simulation to completion, sampling (t, x, y, yaw)."""
    samples: list[tuple[float, float, float, float]] = []
    sim.setStepping(True)
    sim.startSimulation()
    try:
        while (t := sim.getSimulationTime()) < duration:
            x, y, _ = sim.getObjectPosition(robot, -1)
            yaw = sim.getObjectOrientation(robot, -1)[2]
            samples.append((t, x, y, yaw))
            sim.step()
            if sim.getSimulationState() == sim.simulation_stopped:
                break
    finally:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass
    return samples


def path_length(samples) -> float:
    return sum(
        math.dist(a[1:3], b[1:3]) for a, b in zip(samples, samples[1:])
    )


def write_csv(samples, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "x_m", "y_m", "yaw_rad"])
        writer.writerows(samples)


def write_plot(samples, output: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed; skipping the plot", file=sys.stderr)
        return

    xs = [s[1] for s in samples]
    ys = [s[2] for s in samples]
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(xs, ys, linewidth=1.6, label="trajectory")
    ax.plot(xs[0], ys[0], "o", label="start")
    ax.plot(xs[-1], ys[-1], "*", markersize=12, label="end")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Robot trajectory through the maze")
    ax.grid(alpha=0.3)
    ax.legend()
    target = output.with_suffix(".png")
    fig.savefig(target, dpi=150, bbox_inches="tight")
    print(f"plot written to {target}")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    _client, sim = connect(args.host, args.port)

    if not args.no_load_scene:
        if not args.scene.is_file():
            sys.exit(f"scene not found: {args.scene}")
        sim.loadScene(str(args.scene.resolve()))

    robot = resolve_robot(sim, args.robot)
    samples = run(sim, robot, args.duration)
    if not samples:
        sys.exit("the simulation produced no samples")

    write_csv(samples, args.output)
    print(f"{len(samples)} samples over {samples[-1][0]:.2f} s -> {args.output}")
    print(f"path length: {path_length(samples):.3f} m")
    if args.plot:
        write_plot(samples, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
