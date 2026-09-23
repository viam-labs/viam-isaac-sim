"""Do the arms stay unwound over a realistic number of moves?

A fresh boot shows neither the bug nor the fix. The UR SVA allows every revolute joint
+-360 degrees and viam's planner may pick any solution in that range, so the cell winds
itself up gradually: zero after a reset, a joint at 206 degrees after one pass over the
stations, an arm at 270 after two. Approaching 360 the cell fails three different-looking
ways - no plan found, an arm that cannot settle and times out after 30 s, or a move
rejected outright as out of range.

So this has to run long enough to accumulate. It resets, then makes several passes over
every station, and reports the largest absolute angle each joint has reached. With
`joint_limit_deg` set those numbers should sit inside the limit and stop growing; without
it they climb until something jams.

The per-joint maxima are printed whether or not the check passes, because the useful
signal is drift long before it becomes a jam.

    .venv/bin/python probes/windup_check.py [passes]
"""

import asyncio
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from obstacle_check import box, tool_transform  # noqa: E402
from viam.components.arm import Arm  # noqa: E402
from viam.components.generic import Generic  # noqa: E402
from viam.proto.common import (  # noqa: E402
    GeometriesInFrame, Pose, PoseInFrame, WorldState,
)
from viam.robot.client import RobotClient  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
STATIONS = json.loads((Path(__file__).resolve().parent / "stations.json").read_text())
LIMIT_DEG = 180.0
# The served limit needs headroom greater than the 0.5 deg settle tolerance: an arm that
# settles at 180.3 after a goal of 179.8 fails its NEXT move's start check, before
# planning. Peaking this close to the limit is not yet a failure, so it is reported as a
# warning - the point is to see drift while it is still drift.
MARGIN_DEG = 5.0
HOMES = {"arm-a": (-300.0, -380.0, 850.0), "arm-b": (-300.0, 380.0, 850.0)}


async def main():
    passes = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    robot = await RobotClient.at_address(
        FQDN,
        RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]
        ),
    )
    try:
        motion = MotionClient.from_robot(robot, "builtin")
        world = Generic.from_robot(robot, "sim-world")
        arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")}

        await world.do_command({"command": "reset"})
        await asyncio.sleep(3)

        props = list((await world.do_command({"command": "obstacles"}))["obstacles"])
        state = WorldState(
            obstacles=[GeometriesInFrame(
                reference_frame="world",
                geometries=[box(p["label"], p["center_mm"], p["dims_mm"]) for p in props],
            )],
            transforms=[tool_transform("arm-a"), tool_transform("arm-b")],
        )

        async def go(arm, where):
            try:
                await motion.move(
                    component_name=arm,
                    destination=PoseInFrame(reference_frame="world", pose=Pose(
                        x=where[0], y=where[1], z=where[2],
                        o_x=0, o_y=0, o_z=-1, theta=0)),
                    world_state=state,
                )
                return None
            except Exception as exc:  # noqa: BLE001
                return str(exc)

        worst = {name: [0.0] * 6 for name in arms}
        stalls, plan_failures = [], []

        for index in range(1, passes + 1):
            for station, (arm, position) in STATIONS["stations"].items():
                if station.startswith("FORBIDDEN"):
                    continue
                moving = arm if arm.startswith("arm") else f"arm-{arm}"
                other = "arm-b" if moving == "arm-a" else "arm-a"
                await go(other, HOMES[other])
                error = await go(moving, position)
                if error:
                    where = f"pass {index} {station}"
                    (stalls if "stall" in error.lower() else plan_failures).append(where)
                for name, arm_client in arms.items():
                    values = (await arm_client.get_joint_positions()).values
                    worst[name] = [max(w, abs(v)) for w, v in zip(worst[name], values)]
            live = {n: round(max(w)) for n, w in worst.items()}
            print(f"  after pass {index}: max |angle| so far {live}")

        print(f"\n{'arm':>7}  per-joint max |angle| (deg)")
        exceeded = []
        for name, values in worst.items():
            print(f"{name:>7}  {[round(v) for v in values]}")
            over = [f"j{i}={v:.0f}" for i, v in enumerate(values) if v > LIMIT_DEG + 1]
            if over:
                exceeded.append(f"{name}: {', '.join(over)}")

        tight = []
        for name, values in worst.items():
            for i, v in enumerate(values):
                if LIMIT_DEG - MARGIN_DEG < v <= LIMIT_DEG + 1:
                    tight.append(f"{name} j{i}={v:.0f}")
        if tight:
            print(f"  WARNING: within {MARGIN_DEG:.0f} deg of the limit: {', '.join(tight)}")
            print("           a joint that settles past the served limit fails its next "
                  "move's start check")

        print()
        for label, items in (("stalled", stalls), ("no plan", plan_failures)):
            if items:
                print(f"{len(items)} {label}: {', '.join(items[:6])}")
        if exceeded:
            print(f"joints past +-{LIMIT_DEG:.0f}: {'; '.join(exceeded)}")
        if exceeded or stalls or plan_failures:
            return 1
        print(f"{passes} passes, no joint past +-{LIMIT_DEG:.0f} deg, "
              "no stalls, no planning failures")
        return 0
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
