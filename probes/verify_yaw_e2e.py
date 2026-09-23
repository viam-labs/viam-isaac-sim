"""Do Isaac and the motion service agree about where the arms are - through the module?

probes/yaw_probe.py proved the correction against a bare Isaac stage. This proves it
through the running machine, which is the thing that actually has to be right: the frame
system, the kinematics the motion service planned with, and the simulated arm all have to
describe the same robot.

Both arms are commanded to all-zero joints, then two independent answers are compared:

  * `arm.get_end_position()` - the module reading Isaac's flange prim, i.e. the truth of
    the simulation, in WORLD coordinates;
  * `motion.get_pose()` - the frame system's answer, computed from the SVA.

If the base rotation is wrong these disagree by a mirror about the origin: x and y flip
sign while z stays. If the spawn pose is being dropped, arm-b reads the same as arm-a.
"""

import asyncio
import math
import os
import sys

from viam.components.arm import Arm
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
ARMS = ("arm-a", "arm-b")
# SVA forward kinematics at all-zero joints, relative to the arm's base (mm). The arms
# sit at y = -+380, z = 391.4, so in world terms the flange should land at
# (-817.2, -612.9, 454.2) for arm-a and (-817.2, 147.1, 454.2) for arm-b - and crucially
# NOT at the same place as each other, which is what a dropped spawn pose would produce.
SVA_ZERO_MM = (-817.2, -232.9, 62.8)
ARM_BASE_MM = {"arm-a": (0.0, -380.0, 391.4), "arm-b": (0.0, 380.0, 391.4)}
TOLERANCE_MM = 25.0


async def main():
    robot = await RobotClient.at_address(
        FQDN,
        RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]
        ),
    )
    failures = []
    try:
        motion = MotionClient.from_robot(robot, "builtin")
        print(f"resources: {sorted({r.name for r in robot.resource_names})}\n")

        for name in ARMS:
            arm = Arm.from_robot(robot, name)
            await arm.move_to_joint_positions(
                __import__("viam.components.arm", fromlist=["JointPositions"])
                .JointPositions(values=[0.0] * 6)
            )
            await asyncio.sleep(2.0)

            isaac = await arm.get_end_position()
            frame = await motion.get_pose(component_name=name, destination_frame="world")
            f = frame.pose
            print(f"{name}:")
            print(f"  isaac flange (module)   ({isaac.x:8.1f}, {isaac.y:8.1f}, {isaac.z:7.1f})")
            print(f"  frame system (motion)   ({f.x:8.1f}, {f.y:8.1f}, {f.z:7.1f})")
            base = ARM_BASE_MM[name]
            want = tuple(base[i] + SVA_ZERO_MM[i] for i in range(3))
            print(f"  expected in world       ({want[0]:8.1f}, {want[1]:8.1f}, {want[2]:7.1f})")
            off = math.dist((isaac.x, isaac.y, isaac.z), want)
            print(f"  isaac vs expected:      {off:.1f} mm")
            if off > TOLERANCE_MM:
                failures.append(f"{name}: isaac flange {off:.1f} mm from the SVA pose")

            gap = math.dist((isaac.x, isaac.y, isaac.z), (f.x, f.y, f.z))
            mirrored = math.dist((-isaac.x, -isaac.y, isaac.z), (f.x, f.y, f.z))
            print(f"  isaac vs frame system:  {gap:.1f} mm"
                  f"   (mirrored: {mirrored:.1f} mm)")
            if gap > TOLERANCE_MM:
                verdict = ("MIRRORED - base rotation still wrong"
                           if mirrored < gap else "DISAGREE")
                failures.append(f"{name}: {verdict} ({gap:.1f} mm apart)")
                print(f"  -> {verdict}\n")
            else:
                print("  -> agree\n")

        print("\n" + ("\n".join(f"FAIL {f}" for f in failures) if failures
                      else "isaac and the motion service agree on both arms"))
        return 1 if failures else 0
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
