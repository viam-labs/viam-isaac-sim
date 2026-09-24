"""A QC round: pick, check, hand over, check again, place.

Importable on purpose. The tests drive these functions rather than re-implementing the
sequence, and the eventual move into `qc:cell` verbs should be a rename.

Everything takes its clients as arguments. `record_cell.py` grew as closures over
module-level clients and nothing in it could be imported, which meant the only way to
test the round was to run the script and read its output.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from viam.components.arm import JointPositions
from viam.proto.common import (
    Capsule, GeometriesInFrame, Geometry, Pose, PoseInFrame, RectangularPrism,
    Transform, Vector3, WorldState,
)
from viam.proto.service.motion import (
    Constraints, LinearConstraint, OrientationConstraint,
)

from .layout import TOOL_MM, Cell, Vec3

IDENTITY = {"o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 0.0}
PART_LABEL = "part"


@dataclass
class Clients:
    motion: Any
    world: Any
    arms: Dict[str, Any]
    grippers: Dict[str, Any]


@dataclass
class Held:
    """What an arm is carrying, and where it sits relative to that arm's flange.

    Measured at the moment of the grasp rather than derived. The cup lands where its
    raycast hits, which on a curved face is not where the arithmetic would put it, and
    the difference is what the planner needs to know about.
    """

    arm: str
    offset_mm: Vec3
    dims_mm: Vec3
    orientation: Dict[str, float] = field(
        default_factory=lambda: dict(IDENTITY))


@dataclass
class Trace:
    """What happened, for the tests to assert over."""

    stages: List[Dict[str, Any]] = field(default_factory=list)

    def record(self, stage: str, **facts: Any) -> None:
        self.stages.append({"stage": stage, **facts})

    def at(self, stage: str) -> Dict[str, Any]:
        return next(s for s in self.stages if s["stage"] == stage)

    @property
    def names(self) -> List[str]:
        return [s["stage"] for s in self.stages]


# ---- world state -------------------------------------------------------------


def _box(label: str, centre: Sequence[float], dims: Sequence[float]) -> Geometry:
    return Geometry(
        center=Pose(x=centre[0], y=centre[1], z=centre[2], **IDENTITY),
        box=RectangularPrism(dims_mm=Vector3(x=dims[0], y=dims[1], z=dims[2])),
        label=label,
    )


def tool_transform(arm: str, radius_mm: float = 25.0) -> Transform:
    """The cup, carried by the arm holding it."""
    return Transform(
        reference_frame=f"tool-{arm}",
        pose_in_observer_frame=PoseInFrame(
            reference_frame=arm,
            pose=Pose(x=0.0, y=0.0, z=TOOL_MM / 2.0, **IDENTITY)),
        physical_object=Geometry(
            center=Pose(x=0.0, y=0.0, z=0.0, **IDENTITY),
            capsule=Capsule(radius_mm=radius_mm, length_mm=TOOL_MM),
            label=f"tool-{arm}"),
    )


def world_state(props: Sequence[Dict[str, Any]], *, held: Optional[Held] = None,
                ignore: Sequence[str] = ()) -> WorldState:
    """The scene as the planner should see it at this moment.

    A carried part is not scenery and is not absent: it is part of the moving arm. Left
    in the obstacle list it blocks the arm carrying it; left out entirely it sweeps
    through everything on the way. It goes in as a transform on the holding arm's frame,
    which is the only shape that travels.
    """
    skip = set(ignore)
    if held is not None:
        skip.add(PART_LABEL)
    obstacles = [_box(p["label"], p["center_mm"], p["dims_mm"])
                 for p in props if p["label"] not in skip]
    transforms = [tool_transform("arm-a"), tool_transform("arm-b")]
    if held is not None:
        transforms.append(Transform(
            reference_frame=f"carried-{held.arm}",
            pose_in_observer_frame=PoseInFrame(
                reference_frame=held.arm,
                pose=Pose(x=held.offset_mm[0], y=held.offset_mm[1],
                          z=held.offset_mm[2], **IDENTITY)),
            physical_object=Geometry(
                center=Pose(x=0.0, y=0.0, z=0.0, **held.orientation),
                box=RectangularPrism(dims_mm=Vector3(
                    x=held.dims_mm[0], y=held.dims_mm[1], z=held.dims_mm[2])),
                label=PART_LABEL),
        ))
    return WorldState(
        obstacles=[GeometriesInFrame(reference_frame="world", geometries=obstacles)],
        transforms=transforms,
    )


# ---- reading the world -------------------------------------------------------


async def props_of(clients: Clients) -> List[Dict[str, Any]]:
    return list((await clients.world.do_command({"command": "obstacles"}))["obstacles"])


async def part_pose(clients: Clients) -> Dict[str, Any]:
    reply = await clients.world.do_command(
        {"command": "prop_poses", "names": [PART_LABEL]})
    return reply["props"][PART_LABEL]


async def flange_of(clients: Clients, arm: str) -> Vec3:
    pose = await clients.arms[arm].get_end_position()
    return (pose.x, pose.y, pose.z)


def _ov_to_quat(o_x: float, o_y: float, o_z: float, theta_deg: float):
    """Viam orientation vector -> (w, x, y, z).

    The vector names where the frame's +z points; theta is the roll about it.
    """
    length = math.sqrt(o_x * o_x + o_y * o_y + o_z * o_z) or 1.0
    ox, oy, oz = o_x / length, o_y / length, o_z / length
    # rotation taking +z onto the vector, then the roll about it
    dot = max(-1.0, min(1.0, oz))
    if dot > 1.0 - 1e-9:
        tilt = (1.0, 0.0, 0.0, 0.0)
    elif dot < -1.0 + 1e-9:
        tilt = (0.0, 1.0, 0.0, 0.0)
    else:
        axis = (-oy, ox, 0.0)
        norm = math.sqrt(sum(v * v for v in axis)) or 1.0
        half = math.acos(dot) / 2.0
        s_ = math.sin(half)
        tilt = (math.cos(half), axis[0] / norm * s_, axis[1] / norm * s_, 0.0)
    half = math.radians(theta_deg) / 2.0
    roll = (math.cos(half), 0.0, 0.0, math.sin(half))
    aw, ax, ay, az = tilt
    bw, bx, by, bz = roll
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def _quat_to_ov(quat) -> Dict[str, float]:
    """(w, x, y, z) -> the orientation vector viam's Pose carries."""
    w, x, y, z = quat
    # the frame's +z axis, rotated
    ox = 2.0 * (x * z + y * w)
    oy = 2.0 * (y * z - x * w)
    oz = 1.0 - 2.0 * (x * x + y * y)
    length = math.sqrt(ox * ox + oy * oy + oz * oz) or 1.0
    ox, oy, oz = ox / length, oy / length, oz / length
    # roll about it, recovered from where +x landed
    xx = 1.0 - 2.0 * (y * y + z * z)
    xy = 2.0 * (x * y + z * w)
    xz = 2.0 * (x * z - y * w)
    if abs(oz) > 0.9999:
        theta = math.degrees(math.atan2(xy, xx))
    else:
        ref = (-oy, ox, 0.0)
        norm = math.sqrt(ref[0] ** 2 + ref[1] ** 2) or 1.0
        ref = (ref[0] / norm, ref[1] / norm, 0.0)
        cross = (oy * ref[2] - oz * ref[1], oz * ref[0] - ox * ref[2],
                 ox * ref[1] - oy * ref[0])
        theta = math.degrees(math.atan2(
            xx * cross[0] + xy * cross[1] + xz * cross[2],
            xx * ref[0] + xy * ref[1] + xz * ref[2]))
    return {"o_x": ox, "o_y": oy, "o_z": oz, "theta": theta}


def _rotate_into_frame(quat, vector: Vec3) -> Vec3:
    """Express a world-frame vector in the frame that quaternion describes."""
    w, x, y, z = quat
    conj = (w, -x, -y, -z)
    cw, cx, cy, cz = conj
    vx, vy, vz = vector
    tw = -cx * vx - cy * vy - cz * vz
    tx = cw * vx + cy * vz - cz * vy
    ty = cw * vy - cx * vz + cz * vx
    tz = cw * vz + cx * vy - cy * vx
    return (tw * -cx + tx * cw + ty * -cz - tz * -cy,
            tw * -cy - tx * -cz + ty * cw + tz * -cx,
            tw * -cz + tx * -cy - ty * -cx + tz * cw)


async def measure_hold(clients: Clients, arm: str, cell: Cell) -> Held:
    """Where the part sits relative to the arm's frame, measured now.

    The pose comes from the motion service, not from `get_end_position`. Those are two
    different frames: GetEndPosition reports the isaac flange prim, whose tool axis is
    +x, while a `Transform` parented to `arm-a` is read in the frame system's
    end-effector frame, whose tool axis is +z. Measuring in one and declaring in the
    other puts the carried box a quarter turn out - which shows up as the arm colliding
    with the part it is holding.
    """
    frame = (await clients.motion.get_pose(
        component_name=arm, destination_frame="world")).pose
    part = (await part_pose(clients))["position_mm"]
    delta = tuple(part[i] - (frame.x, frame.y, frame.z)[i] for i in range(3))
    quat = _ov_to_quat(frame.o_x, frame.o_y, frame.o_z, frame.theta)
    # The box's ORIENTATION in that frame too. Declaring world-axis dimensions in the
    # arm's frame describes a box that is the right size and the wrong shape, so the
    # planner clears a path the real part does not fit through - which reads as the arm
    # stalling against nothing.
    part_quat = tuple(float(v) for v in
                      (await part_pose(clients))["orientation_wxyz"])
    relative = _quat_mul((quat[0], -quat[1], -quat[2], -quat[3]), part_quat)
    return Held(arm=arm, offset_mm=_rotate_into_frame(quat, delta),
                dims_mm=cell.part.dims_mm, orientation=_quat_to_ov(relative))


# ---- moves -------------------------------------------------------------------


# Holding the part, the tool's orientation has to stay put along the way. Without this
# the planner is free to pick a solution that flips the wrist - a 164 degree roll on what
# should be a straight lift - and a welded part swings through a wide arc and hits the
# cell. The pose at each END was always the same; it was the path between them that span.
HOLDING = Constraints(orientation_constraint=[
    OrientationConstraint(orientation_tolerance_degs=15.0)])

# Kept for reference, not used. Measured: with the carried box declared *and* its
# orientation, a strict path fails where either alone succeeds - the box needs the
# planner's freedom to adjust the wrist, and a straight line removes it. The box turns
# out to prevent the wrist flip these were added for, by making the sweep a collision.
#
# Short moves to and from a surface go in a straight line. An orientation constraint
# alone only fixes the tool angle, not the route: asked to lift 240 mm it kept the angle
# and swung the arm round the back of the cell to do it, pan heading for 140 degrees.
# A lift is a lift.
STRAIGHT = Constraints(linear_constraint=[
    LinearConstraint(line_tolerance_mm=25.0, orientation_tolerance_degs=15.0)])

# Only moves made WHILE HOLDING are constrained. An empty arm can take whatever route it
# likes - it has nothing to swing - and constraining those just removed the freedom the
# planner needed to get into a tight approach at all.


async def move(clients: Clients, arm: str, pose, state: WorldState,
               constraints: Constraints = None, *,
               clients_for_diag: Optional[Clients] = None) -> None:
    position, orientation = pose
    try:
        await clients.motion.move(
            component_name=arm,
            destination=PoseInFrame(reference_frame="world", pose=Pose(
                x=position[0], y=position[1], z=position[2], **orientation)),
            world_state=state,
            constraints=constraints,
        )
    except Exception as exc:  # noqa: BLE001
        # Say where the part and the arm actually were. A stall reports joint angles,
        # which do not tell you what the arm was leaning on.
        try:
            where = (await part_pose(clients))["position_mm"]
            flange = await flange_of(clients, arm)
            raise RuntimeError(
                f"{arm} -> {[round(v) for v in position]} failed with the part at "
                f"{[round(v) for v in where]} and the flange at "
                f"{[round(v) for v in flange]}: {exc}") from exc
        except RuntimeError:
            raise
        except Exception:
            raise exc


async def lift_by_shoulder(clients: Clients, arm: str, degrees: float) -> None:
    """Raise the arm by moving one joint, leaving every other angle alone.

    A Cartesian lift goes through IK, and IK is free to hand back a different wrist for
    the same tool pose. Moving the shoulder cannot: whatever the wrist was holding, it
    still holds it the same way at the top.
    """
    joints = list((await clients.arms[arm].get_joint_positions()).values)
    joints[1] += degrees
    await clients.arms[arm].move_to_joint_positions(JointPositions(values=joints))


async def move_in_steps(clients: Clients, arm: str, start, pose, state: WorldState,
                        step_mm: float = 60.0) -> None:
    """Walk to a pose in short hops rather than one jump.

    RDK seeds IK from the current configuration and scores by joint distance, so a small
    move stays on the branch the arm is already in. A big one does not: asked to lift
    240 mm in one go it repeatedly picked the wrist-flipped solution, a 158 degree roll
    of j5 - which is free to choose when the arm is empty and jams the moment a part is
    welded to the cup, because the part sweeps a wide arc the planner cleared for a box
    it had no reason to think would be swung.
    """
    (sx, sy, sz), _ = start
    (tx, ty, tz), orientation = pose
    span = max(abs(tx - sx), abs(ty - sy), abs(tz - sz))
    steps = max(1, int(span / step_mm))
    for index in range(1, steps + 1):
        fraction = index / steps
        waypoint = ((sx + (tx - sx) * fraction, sy + (ty - sy) * fraction,
                     sz + (tz - sz) * fraction), orientation)
        await move(clients, arm, waypoint, state)


def lifted(pose, by_mm: float):
    position, orientation = pose
    return (position[0], position[1], position[2] + by_mm), orientation


# ---- the round ---------------------------------------------------------------

CLEAR_MM = 240.0
SETTLE_S = 0.6


async def park(clients: Clients, arm: str, props, state: WorldState = None) -> None:
    """Get an arm out of the way, in front of its own base.

    In front, not behind: with the 180 degree base yaw a pose behind the arm sits on the
    pan wrap point, and parking there between every move is what wound the joints up.
    """
    # Tool pointing outward, away from the cell and away from the arm's own body. A
    # tucked park folds the flange back toward the upper arm, and since both tools are
    # declared to the planner at all times, an idle arm intersecting its own tool makes
    # every plan for the OTHER arm fail - which reads as the moving arm being stuck.
    home = {"arm-a": ((120.0, -700.0, 700.0), {"o_x": 0.0, "o_y": -1.0, "o_z": 0.0,
                                               "theta": 0.0}),
            "arm-b": ((120.0, 700.0, 700.0), {"o_x": 0.0, "o_y": 1.0, "o_z": 0.0,
                                              "theta": 0.0})}[arm]
    await move(clients, arm, home, state or world_state(props))


async def pick(clients: Clients, cell: Cell, props, trace: Trace,
               arm: str = "arm-a", side: str = "+y") -> Held:
    """Take the part off the belt by a flat side.

    The part is an obstacle on the way in and not on the last move: approaching with it
    declared keeps the arm from swiping it off the belt, and the final move is a contact
    by intention rather than a collision.
    """
    grip = cell.pick(side)
    trace.record("before_pick", part=(await part_pose(clients))["position_mm"])
    await move(clients, arm, lifted(grip, CLEAR_MM), world_state(props))
    await move(clients, arm, grip, world_state(props, ignore=[PART_LABEL]))
    trace.record("at_contact", part=(await part_pose(clients))["position_mm"],
                 flange=await flange_of(clients, arm))

    caught = await clients.grippers[arm].grab()
    await asyncio.sleep(SETTLE_S)
    held = await measure_hold(clients, arm, cell)
    trace.record("after_grab", claimed=caught,
                 part=(await part_pose(clients))["position_mm"],
                 flange=await flange_of(clients, arm), offset=held.offset_mm)

    # Lift in JOINT space, by raising the shoulder, before anything asks for a roll.
    #
    # Both halves of that matter. The part has to come up first: at the grasp its
    # underside is level with the belt, so turning it about its long axis drives a corner
    # straight into the belt - which is why the wrist would not move at all while holding,
    # measured, at any commanded angle. And the lift has to avoid IK, because asking for a
    # pose 240 mm up let the planner pick the wrist-flipped branch, a 158 degree roll the
    # loaded arm could not complete. Nudging one joint changes no other.
    await lift_by_shoulder(clients, arm, degrees=-16.0)
    trace.record("after_lift", part=(await part_pose(clients))["position_mm"],
                 flange=await flange_of(clients, arm))
    return held


async def lift_from(clients: Clients, cell: Cell, props, held: Held, target,
                    surface: str) -> None:
    """Take the part up off a surface it is resting on."""
    await move_in_steps(clients, held.arm, target, lifted(target, CLEAR_MM),
                        world_state(props, held=held, ignore=[surface]))


async def set_down(clients: Clients, cell: Cell, props, trace: Trace, held: Held,
                   target, stage: str, surface: str = "") -> None:
    """Put the part down on a surface and let go.

    The surface drops out of the obstacle set for the descent. Setting a part down is
    touching the thing you set it on, so with the surface declared the planner refuses -
    it is right to, and the caller is the one who knows the contact is intended.
    """
    arm = held.arm
    ignore = [surface] if surface else []
    await move(clients, arm, lifted(target, CLEAR_MM), world_state(props, held=held),
               HOLDING)
    await move_in_steps(clients, arm, lifted(target, CLEAR_MM), target,
                        world_state(props, held=held, ignore=ignore))
    await clients.grippers[arm].open()
    await asyncio.sleep(SETTLE_S * 2)
    trace.record(stage, part=(await part_pose(clients))["position_mm"],
                 flange=await flange_of(clients, arm))
    await move(clients, arm, lifted(target, CLEAR_MM), world_state(props))


async def carry_neutral(clients: Clients, cell: Cell, props, held: Held) -> None:
    """Bring the part back to a plain pose before the next leg.

    Planning from the end of a presentation - wrist rolled, arm folded across itself -
    is where the planner times out. A neutral pose in between costs one move and gives
    every following plan a sane start.
    """
    neutral = ((420.0, -260.0 if held.arm == "arm-a" else 260.0, 800.0),
               {"o_x": 0.0, "o_y": -1.0 if held.arm == "arm-a" else 1.0,
                "o_z": 0.0, "theta": 0.0})
    await move(clients, held.arm, neutral, world_state(props, held=held))


async def handoff(clients: Clients, cell: Cell, props, trace: Trace, held: Held,
                  to: str = "arm-b", side: str = "-y") -> Held:
    """Pass the part between the arms by setting it down.

    Not mid-air. Both cups lock all six degrees of freedom, so with both closed on one
    rigid body any drive mismatch between the arms is reported as joint load - and load
    is what breaks the shear limit. Setting it down means the two never hold it at once.
    """
    try:
        await carry_neutral(clients, cell, props, held)
    except Exception:  # noqa: BLE001 - a convenience, never a reason to fail the round
        pass
    await set_down(clients, cell, props, trace, held,
                   cell.handoff_grip("+y", cell.RELEASE_GAP_MM), "after_set_down",
                   surface="handoff_stand")
    await park(clients, held.arm, props)
    trace.record("giver_parked", part=(await part_pose(clients))["position_mm"],
                 flange=await flange_of(clients, held.arm))

    grip = cell.handoff_grip(side)
    await move(clients, to, lifted(grip, CLEAR_MM), world_state(props))
    await move(clients, to, grip,
               world_state(props, ignore=[PART_LABEL, "handoff_stand"]))
    caught = await clients.grippers[to].grab()
    await asyncio.sleep(SETTLE_S)
    taken = await measure_hold(clients, to, cell)
    await lift_from(clients, cell, props, taken, grip, "handoff_stand")
    trace.record("after_handoff", claimed=caught,
                 part=(await part_pose(clients))["position_mm"],
                 flange=await flange_of(clients, to), offset=taken.offset_mm)
    return taken


async def place(clients: Clients, cell: Cell, props, trace: Trace, held: Held,
                tray: str, side: str = "-y") -> None:
    await set_down(clients, cell, props, trace, held,
                   cell.place(tray, side, cell.RELEASE_GAP_MM), f"after_place_{tray}",
                   surface=f"{tray}_tray")


async def roll_to(clients: Clients, cell: Cell, props, held: Held, pose,
                  state: WorldState, step_deg: float = 45.0) -> None:
    """Turn the part to a new roll in stages.

    A presentation turns the part about the tool axis without moving it, which swings
    its far corner through an arc more than 200 mm across. Asked for 90 degrees in one
    move the planner has to clear that whole arc at once and gives up; asked for 45 it
    does not.
    """
    position, orientation = pose
    current = await clients.motion.get_pose(
        component_name=held.arm, destination_frame="world")
    start = current.pose.theta
    target = orientation["theta"]
    delta = (target - start + 180.0) % 360.0 - 180.0
    steps = max(1, int(abs(delta) / step_deg))
    for index in range(1, steps + 1):
        theta = start + delta * index / steps
        await move(clients, held.arm, (position, {**orientation, "theta": theta}), state)


async def present(clients: Clients, cell: Cell, props, trace: Trace, held: Held,
                  side: str, stage: str) -> List[str]:
    """Show the camera every face the cup is not covering.

    Returns the faces actually reached. A face the arm could not get to is not a face
    the check saw, and a round that silently skips one is a round that passes a defect.
    """
    shown: List[str] = []
    for face, pose in cell.present_poses(side):
        state = world_state(props, held=held)
        try:
            position, orientation = pose
            current = await clients.motion.get_pose(
                component_name=held.arm, destination_frame="world")
            same_place = all(abs(a - b) < 1.0 for a, b in zip(
                position, (current.pose.x, current.pose.y, current.pose.z)))
            if same_place:
                await roll_to(clients, cell, props, held, pose, state)
            else:
                await move(clients, held.arm, pose, state)
        except Exception as exc:  # noqa: BLE001
            trace.record(f"{stage}_missed_{face}", error=str(exc)[:400])
            continue
        shown.append(face)
        trace.record(f"{stage}_{face}",
                     part=(await part_pose(clients))["position_mm"],
                     flange=await flange_of(clients, held.arm))
    trace.record(stage, shown=shown)
    return shown


async def run_round(clients: Clients, cell: Cell, *, verdict_tray: str = "good",
                    trace: Optional[Trace] = None) -> Trace:
    """One full round, from the belt to a tray."""
    trace = trace or Trace()
    await clients.world.do_command({"command": "reset"})
    await asyncio.sleep(3.5)
    # A reset snaps the part back but leaves the cups closed, so a round that failed
    # while holding would weld this one to an arm before it started.
    for gripper in clients.grippers.values():
        await gripper.open()
    props = await props_of(clients)

    await park(clients, "arm-b", props)
    held = await pick(clients, cell, props, trace, arm="arm-a", side="+y")
    await present(clients, cell, props, trace, held, "+y", "first_check")

    taken = await handoff(clients, cell, props, trace, held, to="arm-b", side="-y")
    await present(clients, cell, props, trace, taken, "-y", "second_check")

    try:
        await carry_neutral(clients, cell, props, taken)
    except Exception:  # noqa: BLE001
        pass
    await place(clients, cell, props, trace, taken, verdict_tray, side="-y")
    return trace
