"""Can the motion service see the cell - the scenery, and the tool on the end of the arm?

Two separate blindnesses, with very different failure modes.

The scenery lives in isaac and nowhere in viam's frame system. Asked to put a flange
inside the belt, the planner plans straight there and the arm drives in until the isaac
collider stalls it. That at least fails loudly.

The tool is worse. The 0.12 m vacuum tool exists in neither the SVA nor the isaac stage,
so the planner will clear the flange past the belt by a millimetre and put the cup right
through it - and nothing stalls, because there is no collider there to stall against. Yet
every station pose in the cell is computed around that tool length.

The tool has to be a `Transform` parented to the arm's end-effector frame, not an entry in
`obstacles`: obstacles are resolved to world once when planning starts, so an obstacle
"on" the arm would just sit wherever the arm happened to be. A transform with a
physical_object is carried by its parent frame and moves with the arm.

Which way the tool points is not a guess. The SVA's ee_link carries ov_degrees
(0, -1, 0, 90), and a viam orientation vector names the frame's z-axis, so the
end-effector frame's +z is the direction the final (0, -99.6, 0) translation continues -
out of the flange. The tool runs along +z of the arm frame. (cell.py's EE_FROM_TOOL
assumes +x, the URDF ee_link convention, which is why it must not be ported as-is.)

Three probes, and the middle one is the control:

    1. flange inside the belt, scenery passed      -> must refuse
    2. flange clear of the belt, no tool declared  -> must reach   (else the test is void)
    3. same pose, tool declared                    -> must refuse

    .venv/bin/python probes/obstacle_check.py
"""

import asyncio
import os
import sys

from viam.components.generic import Generic
from viam.proto.common import (
    Geometry,
    GeometriesInFrame,
    Pose,
    PoseInFrame,
    RectangularPrism,
    Transform,
    Vector3,
    WorldState,
)
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")

ARM = os.environ.get("QC_TOOL_ARM", "arm-a")
TOOL_LENGTH_MM = 120.0
TOOL_RADIUS_MM = 25.0

# Somewhere with nothing near it, to start each attempt from a clean state.
HOME = Pose(x=300.0, y=-380.0, z=900.0, o_x=0, o_y=0, o_z=-1, theta=0)

IDENTITY = dict(o_x=0.0, o_y=0.0, o_z=1.0, theta=0.0)


def box(label, centre_mm, dims_mm):
    return Geometry(
        center=Pose(x=centre_mm[0], y=centre_mm[1], z=centre_mm[2], **IDENTITY),
        box=RectangularPrism(dims_mm=Vector3(x=dims_mm[0], y=dims_mm[1], z=dims_mm[2])),
        label=label,
    )


def tool_transform(arm=None):
    """The tool, as a frame hanging off one arm's flange so it travels with that arm.

    `arm` is not decoration. A transform is carried by its parent, so a tool parented to
    arm-a while arm-b is being planned is just a static box parked wherever arm-a happens
    to be standing - and in this cell that is right beside the inspection point, which
    made all four of arm-b's presenting stations look unreachable. Every arm that carries
    a tool needs its own transform, named distinctly.
    """
    arm = arm or ARM
    return Transform(
        reference_frame=f"tool-{arm}",
        pose_in_observer_frame=PoseInFrame(
            reference_frame=arm,
            pose=Pose(x=0.0, y=0.0, z=TOOL_LENGTH_MM / 2.0, **IDENTITY),
        ),
        physical_object=Geometry(
            center=Pose(x=0.0, y=0.0, z=0.0, **IDENTITY),
            box=RectangularPrism(dims_mm=Vector3(
                x=TOOL_RADIUS_MM * 2, y=TOOL_RADIUS_MM * 2, z=TOOL_LENGTH_MM)),
            label=f"tool-{arm}",
        ),
    )


async def scenery(robot):
    """Ask the sim what it spawned, rather than restating it here and drifting."""
    world = Generic.from_robot(robot, "sim-world")
    reply = await world.do_command({"command": "obstacles"})
    return list(reply["obstacles"])


async def attempt(motion, destination, world_state):
    try:
        await motion.move(
            component_name=ARM,
            destination=PoseInFrame(reference_frame="world", pose=destination),
            world_state=world_state,
        )
        return "reached"
    except Exception as exc:  # noqa: BLE001
        text = str(exc).lower()
        if "collision" in text or "obstacle" in text:
            return "refused"
        if "stall" in text or "timeout" in text:
            return "drove into it"
        return f"other: {str(exc)[:50]}"


async def main():
    robot = await RobotClient.at_address(
        FQDN,
        RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]
        ),
    )
    try:
        motion = MotionClient.from_robot(robot, "builtin")
        props = await scenery(robot)
        print("scenery reported by the sim:")
        for p in props:
            print(f"  {p['label']:>10}  centre {[round(v) for v in p['center_mm']]}"
                  f"  dims {[round(v) for v in p['dims_mm']]}")

        obstacles = WorldState(obstacles=[GeometriesInFrame(
            reference_frame="world",
            geometries=[box(p["label"], p["center_mm"], p["dims_mm"]) for p in props],
        )])
        with_tool = WorldState(obstacles=obstacles.obstacles,
                               transforms=[tool_transform()])

        belt = next(p for p in props if p["label"] == "belt")
        cx, cy, cz = belt["center_mm"]
        top = cz + belt["dims_mm"][2] / 2.0
        # Flange 60 mm above the belt: the flange itself is clear, but a 120 mm tool
        # pointing down reaches 60 mm *below* the top face, i.e. into the belt.
        grazing = Pose(x=cx, y=cy, z=top + 60.0, o_x=0, o_y=0, o_z=-1, theta=0)
        inside = Pose(x=cx, y=cy, z=cz, o_x=0, o_y=0, o_z=-1, theta=0)

        checks = [
            ("flange inside the belt, scenery known", inside, obstacles, "refused"),
            ("flange clear of the belt, no tool", grazing, obstacles, "reached"),
            ("same pose, tool declared", grazing, with_tool, "refused"),
        ]

        failures = []
        print(f"\n{'check':>40} {'want':>9} {'got':>16}")
        for label, destination, state, want in checks:
            await attempt(motion, HOME, obstacles)  # retreat first
            got = await attempt(motion, destination, state)
            ok = got == want
            if not ok:
                failures.append(f"{label}: wanted {want}, got {got}")
            print(f"{label:>40} {want:>9} {got:>16}   {'ok' if ok else '<-- WRONG'}")

        print()
        if failures:
            for f in failures:
                print(f"FAIL {f}")
            return 1
        print("the planner sees the scenery and the tool")
        return 0
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
