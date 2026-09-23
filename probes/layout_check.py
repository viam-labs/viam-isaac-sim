"""Verify the cell layout against the real motion service, not against arithmetic.

Two things are being checked, and the second is the one that matters:

  * every station an arm must reach, it can reach;
  * neither tray can be reached by arm-a.

That second one is the whole reason the cell has two arms. It is a property of the layout,
not a rule the referee enforces, so if the geometry stops guaranteeing it the handoff
silently becomes optional and the exercise loses its point.

It is checked here rather than in `probes/scale_study.py` because the arithmetic version
got it wrong: that probe used the UR5e's datasheet reach of 850 mm, and the bound the
motion service actually enforces on a flange pose is 1016.7 mm. Under the wrong bound the
trays looked safely out of reach at scale 0.62 while the planner reached one of them 5/5.
The planner is the authority, so ask the planner.
"""

import asyncio
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path

from viam.components.arm import Arm
from viam.components.generic import Generic
from viam.proto.common import (
    GeometriesInFrame, Pose, PoseInFrame, RectangularPrism, Vector3, WorldState,
)
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

sys.path.insert(0, str(Path(__file__).resolve().parent))
from obstacle_check import box, tool_transform  # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
STATIONS = json.loads((Path(__file__).resolve().parent / "stations.json").read_text())


async def main():
    robot = await RobotClient.at_address(
        FQDN,
        RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]
        ),
    )
    try:
        motion = MotionClient.from_robot(robot, "builtin")
        bases = STATIONS["bases_mm"]

        # Now that the planner can see them, every station has to survive the scenery
        # AND the tool - the stations were computed around a 0.12 m tool the planner
        # previously knew nothing about, so this is where that arithmetic gets tested.
        world = Generic.from_robot(robot, "sim-world")
        props = list((await world.do_command({"command": "obstacles"}))["obstacles"])
        state = WorldState(
            obstacles=[GeometriesInFrame(
                reference_frame="world",
                geometries=[box(p["label"], p["center_mm"], p["dims_mm"]) for p in props],
            )],
            # Both arms carry a tool; each must hang off its own arm.
            transforms=[tool_transform("arm-a"), tool_transform("arm-b")],
        )
        print(f"planning against {len(props)} props + a tool on each arm\n")

        # Retreat the arm that is NOT being tested first. Without this the test measures
        # the wrong thing: arm-a is left parked at the inspection point by the previous
        # station, so arm-b is refused for the entirely correct reason that arm-a is
        # standing in the spot - which reads as a layout failure and is not one. A round
        # never has both arms at the inspection point either.
        HOMES = {"arm-a": (-300.0, -380.0, 850.0), "arm-b": (-300.0, 380.0, 850.0)}

        async def retreat(arm):
            x, y, z = HOMES[arm]
            try:
                await motion.move(
                    component_name=arm,
                    destination=PoseInFrame(reference_frame="world", pose=Pose(
                        x=x, y=y, z=z, o_x=0, o_y=0, o_z=-1, theta=0)),
                    world_state=state,
                )
                return True
            except Exception:  # noqa: BLE001
                return False

        failures, timings = [], []

        print(f"scale {STATIONS['scale']}\n")
        print(f"{'station':>18} {'arm':>4} {'dist mm':>8} {'want':>10} {'got':>10} {'ms':>7}")
        for name, (arm, position) in STATIONS["stations"].items():
            forbidden = name.startswith("FORBIDDEN")
            # Orientation feasibility is a phase-1 question; this asks only whether the
            # planner will accept the position, which is what the reach bound governs.
            pose = Pose(x=position[0], y=position[1], z=position[2],
                        o_x=0, o_y=0, o_z=-1, theta=0)
            moving = arm if arm.startswith("arm") else f"arm-{arm}"
            other = "arm-b" if moving == "arm-a" else "arm-a"
            if not await retreat(other):
                print(f"  (could not retreat {other} before {name})")
            start = time.perf_counter()
            try:
                await motion.move(
                    component_name=moving,
                    destination=PoseInFrame(reference_frame="world", pose=pose),
                    world_state=state,
                )
                got = "reached"
            except Exception as exc:  # noqa: BLE001
                text = str(exc).lower()
                got = ("too far" if "too far" in text
                       else "collision" if "collision" in text or "obstacle" in text
                       else "no plan")
            elapsed = (time.perf_counter() - start) * 1000
            timings.append(elapsed)

            want = "unreachable" if forbidden else "reached"
            ok = (got != "reached") if forbidden else (got == "reached")
            if not ok:
                failures.append(f"{name}: wanted {want}, got {got}")
            distance = math.dist(position, bases[arm])
            flag = "" if ok else "   <-- WRONG"
            print(f"{name:>18} {arm:>4} {distance:>8.1f} {want:>10} {got:>10} "
                  f"{elapsed:>7.1f}{flag}")

        print(f"\nplanning: median {statistics.median(timings):.1f} ms, "
              f"max {max(timings):.1f} ms over {len(timings)} calls")
        if failures:
            print(f"\n{len(failures)} layout failures:")
            for failure in failures:
                print(f"  - {failure}")
            return 1
        print("\nlayout holds: every station reachable, both trays out of arm-a's reach")
        return 0
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
