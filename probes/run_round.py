"""Drive one QC round against the live machine and print what happened.

    .venv/bin/python probes/run_round.py [good|bad]
"""

import asyncio
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from qc.layout import Cell  # noqa: E402
from qc.round import Clients, Trace, run_round  # noqa: E402
from viam.components.arm import Arm  # noqa: E402
from viam.components.generic import Generic  # noqa: E402
from viam.components.gripper import Gripper  # noqa: E402
from viam.robot.client import RobotClient  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")


async def connect() -> tuple:
    robot = await RobotClient.at_address(
        FQDN,
        RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    clients = Clients(
        motion=MotionClient.from_robot(robot, "builtin"),
        world=Generic.from_robot(robot, "sim-world"),
        arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
        grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                  "arm-b": Gripper.from_robot(robot, "suction-b")},
    )
    return robot, clients


async def main() -> int:
    tray = sys.argv[1] if len(sys.argv) > 1 else "good"
    robot, clients = await connect()
    trace = Trace()
    failure = None
    try:
        await run_round(clients, Cell.from_fragment(), verdict_tray=tray, trace=trace)
    except Exception as exc:  # noqa: BLE001
        # Print the trace anyway. A round that dies mid-way is exactly when the record
        # of what it managed is worth most, and an exception that eats it costs a run.
        failure = str(exc)
    finally:
        await robot.close()

    for stage in trace.stages:
        name = stage["stage"]
        detail = {k: v for k, v in stage.items() if k != "stage"}
        if "part" in detail:
            detail["part"] = [round(v) for v in detail["part"]]
        if "flange" in detail:
            detail["flange"] = [round(v) for v in detail["flange"]]
        if "offset" in detail:
            detail["offset"] = [round(v) for v in detail["offset"]]
        print(f"  {name:>24}  {detail}")
    if failure:
        print(f"\n  FAILED: {failure[:400]}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
