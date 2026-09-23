"""How often does a grasp actually hold, and does the gripper know when it does not?

A single successful pick proves the mechanism works; it does not prove the cell can run a
round. The recording showed the difference - `holding=True` with the carton still on the
belt - so this repeats the pick and scores it two ways that must agree:

  * what the gripper SAYS   - is_holding_something / status / gripped
  * what the simulator DID  - whether the part rose with the flange

A disagreement is worse than a failure. A round that trusts `holding` will carry nothing
to the inspection camera and score it.

    .venv/bin/python probes/grasp_reliability.py [attempts]
"""

import asyncio
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from obstacle_check import box, tool_transform  # noqa: E402
from viam.components.arm import Arm  # noqa: E402
from viam.components.generic import Generic  # noqa: E402
from viam.components.gripper import Gripper  # noqa: E402
from viam.proto.common import (  # noqa: E402
    GeometriesInFrame, Pose, PoseInFrame, WorldState,
)
from viam.robot.client import RobotClient  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
LIFT_MM = 250.0
CARRIED_TOLERANCE_MM = 30.0


async def main():
    attempts = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    robot = await RobotClient.at_address(
        FQDN,
        RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]
        ),
    )
    try:
        motion = MotionClient.from_robot(robot, "builtin")
        world = Generic.from_robot(robot, "sim-world")
        gripper = Gripper.from_robot(robot, "suction-a")
        arm = Arm.from_robot(robot, "arm-a")

        held, claimed, disagreed = 0, 0, 0
        print(f"{'try':>4} {'says':>6} {'status':>8} {'part rose':>10} {'verdict':>24}")
        for attempt in range(1, attempts + 1):
            await world.do_command({"command": "reset"})
            await asyncio.sleep(3.5)
            props = list((await world.do_command({"command": "obstacles"}))["obstacles"])

            def state(with_part: bool):
                chosen = props if with_part else [
                    p for p in props if p["label"] != "part"]
                return WorldState(
                    obstacles=[GeometriesInFrame(
                        reference_frame="world",
                        geometries=[box(p["label"], p["center_mm"], p["dims_mm"])
                                    for p in chosen])],
                    transforms=[tool_transform("arm-a"), tool_transform("arm-b")],
                )

            part = (await world.do_command(
                {"command": "prop_poses", "names": ["part"]}))["props"]["part"]
            px, py, pz = part["position_mm"]
            contact = pz + 40 + 120

            async def move(z, with_part):
                await motion.move(
                    component_name="arm-a",
                    destination=PoseInFrame(reference_frame="world", pose=Pose(
                        x=px, y=py, z=z, o_x=0, o_y=0, o_z=-1, theta=0)),
                    world_state=state(with_part))

            try:
                await move(contact + 150, True)
                await move(contact, False)
                await gripper.grab()
                await asyncio.sleep(0.4)
                before = (await world.do_command(
                    {"command": "prop_poses", "names": ["part"]}))["props"]["part"]
                await move(contact + LIFT_MM, False)
                # A vertical lift only loads the cup along its axis. Carrying loads it
                # sideways, which is where a suction cup actually lets go - and it is what
                # the recorded take did when it dropped the carton.
                mid = (await world.do_command(
                    {"command": "prop_poses", "names": ["part"]}))["props"]["part"]
                lifted = abs(mid["position_mm"][2] - before["position_mm"][2]
                             - LIFT_MM) < CARRIED_TOLERANCE_MM
                await motion.move(
                    component_name="arm-a",
                    destination=PoseInFrame(reference_frame="world", pose=Pose(
                        x=px - 120, y=py + 220, z=contact + LIFT_MM,
                        o_x=0, o_y=0, o_z=-1, theta=0)),
                    world_state=state(False))
            except Exception as exc:  # noqa: BLE001
                print(f"{attempt:>4} {'-':>6} {'-':>8} {'-':>10} "
                      f"{'move failed':>22}  {str(exc)[:40]}")
                continue

            after = (await world.do_command(
                {"command": "prop_poses", "names": ["part"]}))["props"]["part"]
            rose = after["position_mm"][2] - before["position_mm"][2]
            travelled = abs(after["position_mm"][0] - (px - 120)) < 120
            says = (await gripper.is_holding_something()).is_holding_something
            status = (await gripper.do_command({"command": "status"}))["status"]
            carried = lifted and travelled

            held += carried
            claimed += says
            mismatch = says != carried
            disagreed += mismatch
            verdict = ("carried" if carried else
                       ("dropped in the carry" if lifted else "never lifted"))
            if mismatch:
                verdict += "  <-- GRIPPER LIED"
            print(f"{attempt:>4} {str(says):>6} {status:>8} {rose:>9.1f}mm {verdict:>24}")
            await gripper.open()

        print(f"\nactually carried {held}/{attempts}; "
              f"claimed to be holding {claimed}/{attempts}; "
              f"disagreed {disagreed}/{attempts}")
        if disagreed:
            print("a round cannot trust is_holding_something until this is zero")
        return 0 if held == attempts and not disagreed else 1
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
