"""Split motion.Move into the part that thinks and the part that moves.

The round budget has been unmeasurable because `motion.Move()` plans *and* executes and
only returns when the arm has arrived, and the SDK has no plan-only call - `get_plan`
only retrieves a plan already in flight. An earlier attempt to time it reported 3-5 ms and
concluded planning was free; that was an artifact of repeating a move the arm had already
made, which returns immediately having done nothing.

Running the move as a concurrent task instead of awaiting it straight through makes the
split observable: poll the joints while it runs, and the instant they first budge is the
boundary. Everything before it is planning; everything after is the arm actually driving.

That matters because the two scale differently. Planning is per-call and would be spent
even on a real robot; execution is the simulated arm moving at its own rate. A round
issues about fifty moves, so a course that fits depends on which half dominates.

    .venv/bin/python probes/latency_check.py
"""

import asyncio
import json
import os
import statistics
import sys
import time
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
HOMES = {"arm-a": (300.0, -380.0, 900.0), "arm-b": (300.0, 380.0, 900.0)}
MOVED_DEG = 0.5      # a joint has "started" once it leaves its start by this much
POLL_S = 0.02


async def timed_move(motion, arms, name, position, state):
    """(plan_ms, exec_ms) for one move, or (total_ms, None) if it never moved."""
    arm = arms[name]
    start = [v for v in (await arm.get_joint_positions()).values]

    t0 = time.perf_counter()
    task = asyncio.create_task(motion.move(
        component_name=name,
        destination=PoseInFrame(reference_frame="world", pose=Pose(
            x=position[0], y=position[1], z=position[2], o_x=0, o_y=0, o_z=-1, theta=0)),
        world_state=state,
    ))

    first_motion = None
    while not task.done():
        values = [v for v in (await arm.get_joint_positions()).values]
        if first_motion is None and any(
            abs(a - b) > MOVED_DEG for a, b in zip(values, start)
        ):
            first_motion = time.perf_counter()
            break
        await asyncio.sleep(POLL_S)

    try:
        await task
    except Exception as exc:  # noqa: BLE001
        return None, str(exc)[:60]
    done = time.perf_counter()
    if first_motion is None:
        # Already at the goal: nothing to plan around and nothing to drive.
        return ((done - t0) * 1000, None)
    return ((first_motion - t0) * 1000, (done - first_motion) * 1000)


async def main():
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

        plans, execs, noop = [], [], 0
        print(f"{'station':>18} {'plan ms':>9} {'exec ms':>9}")
        for station, (arm, position) in STATIONS["stations"].items():
            if station.startswith("FORBIDDEN"):
                continue
            moving = arm if arm.startswith("arm") else f"arm-{arm}"
            other = "arm-b" if moving == "arm-a" else "arm-a"
            await timed_move(motion, arms, other, HOMES[other], state)
            plan, execution = await timed_move(motion, arms, moving, position, state)
            if plan is None:
                print(f"{station:>18}  FAILED {execution}")
                continue
            if execution is None:
                noop += 1
                print(f"{station:>18} {plan:>9.0f} {'(no motion)':>9}")
                continue
            plans.append(plan)
            execs.append(execution)
            print(f"{station:>18} {plan:>9.0f} {execution:>9.0f}")

        if not plans:
            print("\nno moves produced motion")
            return 1
        print(f"\n{'':>18} {'median':>9} {'max':>9}   total over {len(plans)} moves")
        print(f"{'planning':>18} {statistics.median(plans):>9.0f} {max(plans):>9.0f}"
              f"   {sum(plans)/1000:>6.1f} s")
        print(f"{'execution':>18} {statistics.median(execs):>9.0f} {max(execs):>9.0f}"
              f"   {sum(execs)/1000:>6.1f} s")
        per_move = (sum(plans) + sum(execs)) / len(plans)
        print(f"\nper move {per_move:.0f} ms; a 50-move round would be "
              f"{per_move * 50 / 1000:.0f} s of arm time"
              + (f" ({noop} moves did not need to move)" if noop else ""))
        return 0
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
