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
from viam.components.generic import Generic
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
# Both arms carry a 180 deg base yaw so their workspace does not straddle the pan wrap
# point. Read it from the fragment rather than restating it: the expected flange position
# rotates with the base, and a probe that hardcodes one orientation silently checks the
# wrong thing the moment the config changes.
def _base_yaw_deg(name):
    import json
    from pathlib import Path as _P
    cfg = json.loads((_P(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text())
    comp = next(c for c in cfg["components"] if c["name"] == name)
    o = comp.get("frame", {}).get("orientation")
    return float(o["value"]["th"]) if o else 0.0
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

        # Zero the joints with a sim reset, not a joint command: move_to_joint_positions
        # is an unchecked joint-space sweep through the cell, and from an arbitrary start
        # it drives the arm through the floor or the belt on the way. reset teleports.
        await Generic.from_robot(robot, "sim-world").do_command({"command": "reset"})
        await asyncio.sleep(3.0)

        for name in ARMS:
            arm = Arm.from_robot(robot, name)

            joints = list((await arm.get_joint_positions()).values)
            at_zero = max(abs(v) for v in joints) < 1.0
            isaac = await arm.get_end_position()
            frame = await motion.get_pose(component_name=name, destination_frame="world")
            f = frame.pose
            print(f"{name}:")
            print(f"  isaac flange (module)   ({isaac.x:8.1f}, {isaac.y:8.1f}, {isaac.z:7.1f})")
            print(f"  frame system (motion)   ({f.x:8.1f}, {f.y:8.1f}, {f.z:7.1f})")
            base = ARM_BASE_MM[name]
            yaw = math.radians(_base_yaw_deg(name))
            c, s_ = math.cos(yaw), math.sin(yaw)
            rx = SVA_ZERO_MM[0] * c - SVA_ZERO_MM[1] * s_
            ry = SVA_ZERO_MM[0] * s_ + SVA_ZERO_MM[1] * c
            want = (base[0] + rx, base[1] + ry, base[2] + SVA_ZERO_MM[2])
            print(f"  expected in world       ({want[0]:8.1f}, {want[1]:8.1f}, {want[2]:7.1f})")
            off = math.dist((isaac.x, isaac.y, isaac.z), want)
            print(f"  joints                  {[round(v, 1) for v in joints]}")
            # The SVA-pose comparison is only meaningful at zero joints. reset() restores
            # the articulation's default state, which is not guaranteed to be all-zero,
            # so check rather than assume - otherwise a drooping arm reads as a frame bug.
            if at_zero:
                print(f"  isaac vs expected:      {off:.1f} mm")
                if off > TOLERANCE_MM:
                    failures.append(f"{name}: isaac flange {off:.1f} mm from the SVA pose")
            else:
                print(f"  (not at zero joints, so the SVA-pose check is skipped; "
                      f"isaac vs frame system below is the real invariant)")

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
