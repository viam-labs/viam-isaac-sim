"""Isaac Sim's surface gripper, reached from this module.

A surface gripper is Isaac's model of a suction cup. It is a
``IsaacSurfaceGripper`` prim that owns a list of attachment points, each a
plain D6 ``PhysicsJoint`` whose body0 is the gripper's own body. On close the
gripper raycasts from each joint along its forward axis, up to its max grip
distance, and re-points the joint's second body at whatever the ray hits, so
the joint's locked axes become the hold. A load past the coaxial or shear
force limit breaks the joint and the status falls back to Open. Everything
here that touches USD takes the USD modules as parameters, so the tests can
drive it with fakes the way the suction weld's tests do.

``author_attachment_rig`` is the EPick's own rig: an anchor and one
attachment joint per cup, each joint compliant along the cup axis so a
carried box can pitch and swing the way four suction cups on a bellows do.

The smoke rig at the bottom builds one such gripper over the vacuum's mount
link on a live stage, closes it on whatever is under the cup and reports what
the gripper says. It exists to answer, on hardware: whether a gripper
authored while the sim is playing is picked up at all, whether its raycast
finds one of this module's scaled box props, and whether attachment points
offset from their body's origin are accepted.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from viam.logging import getLogger

from . import compat
from .asset_catalog import EPICK
from .spatial import Quat, Vec3, compose_pose, quat_rotate

LOGGER = getLogger(__name__)

# the schema's own attribute and relationship names, from RobotSchema.usda
ATTR_FORWARD_AXIS = "isaac:forwardAxis"
ATTR_CLEARANCE_OFFSET = "isaac:clearanceOffset"
ATTR_MAX_GRIP_DISTANCE = "isaac:maxGripDistance"
ATTR_COAXIAL_FORCE_LIMIT = "isaac:coaxialForceLimit"
ATTR_SHEAR_FORCE_LIMIT = "isaac:shearForceLimit"
ATTR_RETRY_INTERVAL = "isaac:retryInterval"
ATTR_STATUS = "isaac:status"
REL_ATTACHMENT_POINTS = "isaac:attachmentPoints"
REL_GRIPPED_OBJECTS = "isaac:grippedObjects"

# a D6 axis is locked when its lower limit sits above its upper one
LOCKED_AXES = ("transX", "transY", "transZ", "rotX", "rotY", "rotZ")

# the cup's axis is the mount link's +Z, the direction the tool hangs
FORWARD_AXIS = "Z"

SMOKE_SCOPE = "/World/SurfaceGripperSmoke"
SMOKE_ANCHOR_PRIM = f"{SMOKE_SCOPE}/Anchor"
SMOKE_GRIPPER_PRIM = f"{SMOKE_SCOPE}/SurfaceGripper"
SMOKE_STEPS = ("author", "wire", "restart", "close", "open", "status", "cleanup")

SMOKE_MAX_GRIP_DISTANCE_M = 0.02
SMOKE_FORCE_LIMIT_N = 100.0
SMOKE_RETRY_INTERVAL_S = 1.0
# the ghost body the joints hang from while open: light enough to follow the
# wrist through a locked joint without the arm noticing
GHOST_MASS_KG = 0.1
GHOST_INERTIA_KG_M2 = 1e-4


# -- the EPick model's gripper, from the manual's ratings ---------------------

STANDARD_GRAVITY_M_S2 = 9.80665


def cup_vacuum_force_n(
    radius_mm: float = float(EPICK["cups"]["radius_mm"]),
    vacuum_pct: float = float(EPICK["max_vacuum_pct"]),
    kpa_per_vacuum_pct: float = float(EPICK["kpa_per_vacuum_pct"]),
) -> float:
    """The holding force one cup develops: its inside area times the vacuum
    it pulls, the manual's section 6.2.1 formula. At the 80 % maximum a
    49 mm cup develops 153 N, and that is the load past which an attachment
    breaks. The manual's 4.5 kg per air node is a pump rating, not a holding
    force: with the break force set there, every carry on the GPU machine
    on 2026-09-22 lost the box the moment the arm moved."""
    area_m2 = math.pi * (radius_mm / 1000.0) ** 2
    return area_m2 * vacuum_pct * kpa_per_vacuum_pct * 1000.0


# The coaxial limit is per attachment point. The shear limit is what all the
# cups resist together, since only one joint of a set locks the lateral axes
# (see author_attachment_joint) and so reads every bit of the box's sideways
# load: a cup's friction takes about half its normal force, times the cups. A
# single cup's share, 76 N, released a 2 kg box at every stop of the arm on
# 2026-09-22, where the arm's stiff drives decelerate it in a step or two
CUP_FRICTION_OF_NORMAL = 0.5
DEFAULT_COAXIAL_FORCE_LIMIT_N = cup_vacuum_force_n()
DEFAULT_SHEAR_FORCE_LIMIT_N = (
    DEFAULT_COAXIAL_FORCE_LIMIT_N * CUP_FRICTION_OF_NORMAL * len(EPICK["cups"]["names"])
)
# how far past the clearance offset the raycast looks for a payload. It has to
# exceed the approach gap the clearance is set to (CUP_APPROACH_GAP_MM), or the
# ray has no length left when it starts
DEFAULT_MAX_GRIP_DISTANCE_MM = 15.0
# the automatic mode tries for vacuum for at most 2 s before it gives up,
# manual section 7
DEFAULT_RETRY_INTERVAL_S = 2.0
# The bellows: each cup's attachment joint is a spring along the cup axis, so
# a carried box can pitch by stretching one pair of cups and compressing the
# other, and can swing within the lateral rotation limit. Four joints locked
# in translation would hold the box rigid, which is the weld this replaces.
# The spring is a PhysX soft limit and not a drive on purpose: PhysX reports a
# joint's force from its locked axes and limits, never from a drive, and the
# gripper plugin releases on that reported force, so a spring authored as a
# drive carries the box's whole weight where the plugin cannot see it (a 98 kg
# box was lifted and held on the GPU machine 2026-09-22). The soft limit's
# dead band is where the spring starts; there is no hard stop, the spring is
# the whole travel. The manual gives no spring rate for a 1.5-bellows cup;
# these are attributes and the GPU checklist's swing item measures them.
# 5000 N/m sags a 2 kg box 1 mm at rest. The damping stays low because the
# plugin reads the damper's force too: at a motion onset the damper puts
# (damping times the arm's speed) through every cup before the box catches
# up, and 100 N s/m at 0.5 m/s was 50 N per cup
DEFAULT_CUP_STIFFNESS_N_PER_M = 5000.0
DEFAULT_CUP_DAMPING_N_S_PER_M = 20.0
CUP_BELLOWS_DEAD_BAND_M = 0.0005
CUP_LATERAL_LIMIT_DEG = 15.0


@dataclass(frozen=True)
class GripperLimits:
    """The IsaacSurfaceGripper prim's own attributes, in the units the prim
    takes (metres, newtons, seconds)."""

    max_grip_distance_m: float = DEFAULT_MAX_GRIP_DISTANCE_MM / 1000.0
    coaxial_force_limit_n: float = DEFAULT_COAXIAL_FORCE_LIMIT_N
    shear_force_limit_n: float = DEFAULT_SHEAR_FORCE_LIMIT_N
    retry_interval_s: float = DEFAULT_RETRY_INTERVAL_S


@dataclass(frozen=True)
class CupCompliance:
    """One cup's bellows, on its attachment joint: the joint's axis along the
    cup (the forward axis) is a soft limit, free within +-dead_band_m and
    pulled back by this stiffness and damping beyond it, its two lateral
    rotations are free within +-lateral_limit_deg, and its other axes are
    locked on the one joint that carries the shear and free on the rest."""

    stiffness_n_per_m: float = DEFAULT_CUP_STIFFNESS_N_PER_M
    damping_n_s_per_m: float = DEFAULT_CUP_DAMPING_N_S_PER_M
    dead_band_m: float = CUP_BELLOWS_DEAD_BAND_M
    lateral_limit_deg: float = CUP_LATERAL_LIMIT_DEG


# The plugin's own coaxial check compares a single physics step's reported
# force with its limit, with no window, and the arm follows a linear move as
# waypoints a couple of millimetres apart, each a step into stiff drives that
# stretches the bellows for a step or two: at least 15 N per cup on a 2 kg box
# (GPU machine, 2026-09-23). The module reads the bellows' stretch itself and
# averages it over COAXIAL_LOAD_WINDOW_S (handles/vacuum.py), so the plugin's
# coaxial limit is authored as 0, which turns its check off. Shear stays with
# the plugin, whose locked-axis reading measurably works.
PLUGIN_COAXIAL_CHECK_OFF_N = 0.0
# long enough to ride out a waypoint onset of a few physics steps, short next
# to the EPick's own 150 ms grip time
COAXIAL_LOAD_WINDOW_S = 0.1


def cup_pull_load_n(extension_m: float, compliance: CupCompliance) -> float:
    """The pull one cup's bellows carries at ``extension_m`` of stretch along
    the cup axis, positive away from the tool: the soft limit's spring past
    its dead band, 0 inside the dead band or in compression. The damper's
    share is left out on purpose, since it is the onset transient the window
    exists to ride out."""
    stretch = extension_m - compliance.dead_band_m
    if stretch <= 0.0:
        return 0.0
    return compliance.stiffness_n_per_m * stretch


class LoadWindow:
    """A running mean of per-step loads over a fixed span of physics steps.
    ``mean_n`` is 0 until the window has filled once, so a hold cannot be
    released on its first samples."""

    def __init__(self, window_s: float, physics_dt: float) -> None:
        self.steps = max(1, round(window_s / physics_dt))
        self._samples: deque[float] = deque(maxlen=self.steps)

    @property
    def filled(self) -> bool:
        return len(self._samples) >= self.steps

    @property
    def mean_n(self) -> float:
        if not self.filled:
            return 0.0
        return sum(self._samples) / len(self._samples)

    def push(self, load_n: float) -> float:
        """Add one step's load and return the window's mean."""
        self._samples.append(load_n)
        return self.mean_n

    def reset(self) -> None:
        self._samples.clear()


@dataclass(frozen=True)
class HoldLoad:
    """What the module's coaxial monitor reports, carried in the vacuum
    model's ``is_holding_something`` meta under the same three names: the
    windowed mean pull on the most loaded cup right now, the highest
    single-step pull on any cup since the last grab, and the windowed pull
    that opened the gripper, None while nothing has been released, and the
    monitor's own state: ``off`` (no limit, compliance, joints or reader),
    ``idle`` (on, nothing held or no cup attached to what is held),
    ``armed`` (reading a hold) or ``stopped`` (it raised and logged once)."""

    coaxial_load_n: float = 0.0
    peak_coaxial_load_n: float = 0.0
    released_load_n: float | None = None
    monitor: str = "off"


@dataclass(frozen=True)
class AttachmentRig:
    """What author_attachment_rig put on the stage."""

    scope_path: str
    anchor_path: str
    joint_paths: tuple[str, ...]
    gripper_path: str


def author_attachment_rig(
    modules: Mapping[str, Any],
    stage: Any,
    *,
    scope_path: str,
    body0_path: str,
    body0_world_pose: tuple[Vec3, Quat],
    tool_pose_in_body0: tuple[Vec3, Quat],
    points_tool_m: Sequence[Vec3],
    clearance_offset_m: float,
    limits: GripperLimits,
    compliance: CupCompliance | None,
    forward_axis: str = FORWARD_AXIS,
) -> AttachmentRig:
    """A whole surface gripper in one pass: the ghost anchor, one attachment
    joint per point and the gripper prim pointed at them, all under
    ``scope_path`` (a world-level scope, never inside the articulation the
    joints hang from, since a rigid body nested under a link is an error).

    ``body0_path`` is the arm link the joints hang from, at
    ``body0_world_pose`` right now. ``tool_pose_in_body0`` is the tool frame
    (the TCP at its origin, +Z the cup axis) in that link's frame, and
    ``points_tool_m`` are the attachment points in the tool frame. So each
    joint's body0 side sits at the tool pose composed with its point, with
    the tool's rotation, and its forward axis is the tool's +Z. The anchor
    is authored at the first point's world pose in the tool's orientation,
    and each joint's body1 side is that point's offset from the first, in
    the tool frame. ``compliance`` None locks every axis (the smoke's rig).
    With a compliance the first joint carries the shear and the rest are
    springs along the cup axis only.

    The caller resets the world afterwards: the plugin only looks for
    grippers on the first physics frame after play."""
    identity: Quat = (1.0, 0.0, 0.0, 0.0)
    body0_pos, body0_quat = body0_world_pose
    tool_pos, tool_quat = tool_pose_in_body0
    tool_world_pos, tool_world_quat = compose_pose(body0_pos, body0_quat, tool_pos, tool_quat)
    points = list(points_tool_m)
    anchor_pos, anchor_quat = compose_pose(tool_world_pos, tool_world_quat, points[0], identity)

    stage.DefinePrim(scope_path, "Scope")
    anchor_path = f"{scope_path}/Anchor"
    author_ghost_anchor(
        modules["UsdGeom"],
        modules["UsdPhysics"],
        modules["PhysxSchema"],
        modules["Gf"],
        stage,
        anchor_path,
        anchor_pos,
        anchor_quat,
    )

    joint_paths = attachment_joint_paths(scope_path, len(points))
    offsets = anchor_relative_offsets(points)
    for index, (path, point, offset) in enumerate(zip(joint_paths, points, offsets, strict=True)):
        local_pos0 = quat_rotate(tool_quat, point)
        local_pos0 = (
            tool_pos[0] + local_pos0[0],
            tool_pos[1] + local_pos0[1],
            tool_pos[2] + local_pos0[2],
        )
        author_attachment_joint(
            modules["UsdPhysics"],
            modules["Sdf"],
            modules["Gf"],
            modules["robot_schema"],
            stage,
            path,
            body0_path,
            anchor_path,
            local_pos0,
            offset,
            forward_axis,
            clearance_offset_m,
            local_rot0=tool_quat,
            local_rot1=identity,
            compliance=compliance,
            carries_shear=index == 0,
            physx_schema=modules["PhysxSchema"],
        )

    gripper_path = f"{scope_path}/SurfaceGripper"
    author_surface_gripper(
        modules["robot_schema"],
        modules["Sdf"],
        stage,
        gripper_path,
        joint_paths,
        limits.max_grip_distance_m,
        PLUGIN_COAXIAL_CHECK_OFF_N,
        limits.shear_force_limit_n,
        limits.retry_interval_s,
    )

    return AttachmentRig(
        scope_path=scope_path,
        anchor_path=anchor_path,
        joint_paths=tuple(joint_paths),
        gripper_path=gripper_path,
    )


@dataclass(frozen=True)
class SmokeSpec:
    """One smoke run's gripper: where its attachment points sit in the mount
    link's frame, and the gripper's own limits."""

    gripper: str
    points_m: tuple[Vec3, ...]
    clearance_offset_m: float = 0.0
    max_grip_distance_m: float = SMOKE_MAX_GRIP_DISTANCE_M
    coaxial_force_limit_n: float = SMOKE_FORCE_LIMIT_N
    shear_force_limit_n: float = SMOKE_FORCE_LIMIT_N
    retry_interval_s: float = SMOKE_RETRY_INTERVAL_S


def _number(command: Mapping[str, Any], key: str, default: float, *, minimum: float) -> float:
    value = command.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value < minimum:
        raise ValueError(
            f"surface_gripper_smoke: {key!r} must be a number >= {minimum}, got {value!r}"
        )
    return float(value)


def parse_smoke_spec(command: Mapping[str, Any]) -> SmokeSpec:
    """The smoke command's own arguments, checked before anything touches
    the stage. ``points_m`` is a list of [x, y, z] offsets in the mount
    link's frame, in metres, and cannot be empty."""
    gripper = str(command.get("name", ""))
    if not gripper:
        raise ValueError("surface_gripper_smoke requires 'name', the vacuum gripper component")
    raw_points = command.get("points_m")
    if not isinstance(raw_points, Sequence) or isinstance(raw_points, str) or not raw_points:
        raise ValueError("surface_gripper_smoke requires 'points_m', a non-empty list of [x, y, z]")
    points: list[Vec3] = []
    for point in raw_points:
        if (
            not isinstance(point, Sequence)
            or isinstance(point, str)
            or len(point) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in point)
        ):
            raise ValueError(
                f"surface_gripper_smoke: every point must be [x, y, z] in metres, got {point!r}"
            )
        points.append((float(point[0]), float(point[1]), float(point[2])))
    max_grip = _number(command, "max_grip_distance_m", SMOKE_MAX_GRIP_DISTANCE_M, minimum=1e-6)
    clearance = _number(command, "clearance_offset_m", 0.0, minimum=0.0)
    if clearance >= max_grip:
        raise ValueError(
            "surface_gripper_smoke: the raycast runs from the clearance offset to the max grip "
            f"distance, so the clearance ({clearance}) must be below it ({max_grip})"
        )
    return SmokeSpec(
        gripper=gripper,
        points_m=tuple(points),
        clearance_offset_m=clearance,
        max_grip_distance_m=max_grip,
        coaxial_force_limit_n=_number(
            command, "coaxial_force_limit_n", SMOKE_FORCE_LIMIT_N, minimum=0.0
        ),
        shear_force_limit_n=_number(
            command, "shear_force_limit_n", SMOKE_FORCE_LIMIT_N, minimum=0.0
        ),
        retry_interval_s=_number(command, "retry_interval_s", SMOKE_RETRY_INTERVAL_S, minimum=0.0),
    )


def attachment_joint_paths(scope: str, count: int) -> list[str]:
    return [f"{scope}/AttachmentPoint_{index}" for index in range(count)]


def anchor_relative_offsets(points_m: Sequence[Vec3]) -> list[Vec3]:
    """Each point relative to the first. The anchor body sits at the first
    point, so these are the joints' local positions on its side."""
    x0, y0, z0 = points_m[0]
    return [(x - x0, y - y0, z - z0) for x, y, z in points_m]


def _set_attr(prim: Any, sdf: Any, name: str, type_name: str, value: Any) -> None:
    """Set an attribute an applied schema may or may not have authored yet."""
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, getattr(sdf.ValueTypeNames, type_name))
    attr.Set(value)


def author_ghost_anchor(
    usd_geom: Any,
    usd_physics: Any,
    physx_schema: Any,
    gf: Any,
    stage: Any,
    path: str,
    position_m: Vec3,
    orientation_wxyz: Quat,
) -> None:
    """The attachment joints' second body while the gripper is open: a
    dynamic rigid body with no shape, no gravity and almost no mass. The
    gripper disables the joints while open and re-points them at the
    gripped object on close, so this body never carries a load. It is
    dynamic rather than kinematic on purpose. Between authoring and the
    gripper's first frame the locked joints are live, and a kinematic
    anchor would pin the wrist to wherever it was authored, which is what
    stalled every lift on 2026-09-22. A ghost this light is dragged along
    by the wrist instead. Without the PhysX schema it still falls under
    gravity, so ``physx_schema`` may only be None where that is acceptable."""
    xform = usd_geom.Xform.Define(stage, path)
    xformable = usd_geom.Xformable(xform)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(gf.Vec3d(*(float(v) for v in position_m)))
    w, x, y, z = (float(v) for v in orientation_wxyz)
    xformable.AddOrientOp().Set(gf.Quatf(w, gf.Vec3f(x, y, z)))
    prim = xform.GetPrim()
    usd_physics.RigidBodyAPI.Apply(prim)
    mass = usd_physics.MassAPI.Apply(prim)
    mass.CreateMassAttr(GHOST_MASS_KG)
    mass.CreateDiagonalInertiaAttr(
        gf.Vec3f(GHOST_INERTIA_KG_M2, GHOST_INERTIA_KG_M2, GHOST_INERTIA_KG_M2)
    )
    if physx_schema is not None:
        physx_schema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)


_AXIS_LETTERS = ("X", "Y", "Z")


def _quat_to_gf(gf: Any, quat: Quat) -> Any:
    w, x, y, z = (float(v) for v in quat)
    return gf.Quatf(w, gf.Vec3f(x, y, z))


def _lock_axis(usd_physics: Any, prim: Any, axis: str) -> None:
    limit = usd_physics.LimitAPI.Apply(prim, axis)
    limit.CreateLowAttr().Set(1.0)
    limit.CreateHighAttr().Set(-1.0)


def author_attachment_joint(
    usd_physics: Any,
    sdf: Any,
    gf: Any,
    robot_schema: Any,
    stage: Any,
    joint_path: str,
    body0_path: str,
    body1_path: str,
    local_pos0: Vec3,
    local_pos1: Vec3,
    forward_axis: str = FORWARD_AXIS,
    clearance_offset_m: float = 0.0,
    local_rot0: Quat = (1.0, 0.0, 0.0, 0.0),
    local_rot1: Quat = (1.0, 0.0, 0.0, 0.0),
    compliance: CupCompliance | None = None,
    carries_shear: bool = True,
    physx_schema: Any | None = None,
) -> None:
    """One attachment point: a D6 joint between the gripper body and the
    anchor, kept out of the articulation, carrying the attachment-point API
    with the closing direction and the clearance the raycast starts from.

    With ``compliance`` None every axis is locked, which welds the anchor to
    the gripper body: four such joints hold a carried box rigid, since a box
    fixed at four corners cannot translate or rotate without breaking one of
    them. With a ``compliance`` the joint's forward-axis translation becomes
    a spring instead of a weld, standing in for the cup's bellows: it is
    a soft limit, free within +-dead_band_m and pulled back beyond it
    with the given stiffness and damping, which PhysX reports to the
    plugin, and the two lateral rotations are freed within
    +-lateral_limit_deg so the box can pitch as one cup compresses and its
    opposite stretches. The other three axes stay locked on the one joint
    that ``carries_shear``, and are free on the rest, so a set of cups never
    constrains the same freedom twice."""
    joint = usd_physics.Joint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([sdf.Path(body0_path)])
    joint.CreateBody1Rel().SetTargets([sdf.Path(body1_path)])
    joint.CreateLocalPos0Attr().Set(gf.Vec3f(*(float(v) for v in local_pos0)))
    joint.CreateLocalRot0Attr().Set(_quat_to_gf(gf, local_rot0))
    joint.CreateLocalPos1Attr().Set(gf.Vec3f(*(float(v) for v in local_pos1)))
    joint.CreateLocalRot1Attr().Set(_quat_to_gf(gf, local_rot1))
    joint.CreateExcludeFromArticulationAttr().Set(True)
    prim = joint.GetPrim()
    if compliance is None:
        for axis in LOCKED_AXES:
            _lock_axis(usd_physics, prim, axis)
    else:
        lateral_letters = [letter for letter in _AXIS_LETTERS if letter != forward_axis]
        # Only one cup of a set pins the box sideways and against turning. Four
        # cups doing it lock the same three degrees of freedom four times over,
        # and PhysX shares the fight between them as constraint force that the
        # gripper plugin reads as load: on the GPU machine every carry released
        # the box the moment the arm moved sideways. With one cup carrying the
        # shear, every force the plugin reads on a joint is a real load
        if carries_shear:
            for letter in lateral_letters:
                _lock_axis(usd_physics, prim, f"trans{letter}")
            _lock_axis(usd_physics, prim, f"rot{forward_axis}")
        forward_trans = f"trans{forward_axis}"
        band = usd_physics.LimitAPI.Apply(prim, forward_trans)
        band.CreateLowAttr().Set(-compliance.dead_band_m)
        band.CreateHighAttr().Set(compliance.dead_band_m)
        if physx_schema is None:
            raise ValueError(
                "a cup's bellows needs PhysxSchema for its soft limit: a spring authored as a "
                "drive carries the load where the gripper plugin cannot read it"
            )
        soft_limit = physx_schema.PhysxLimitAPI.Apply(prim, forward_trans)
        soft_limit.CreateStiffnessAttr(compliance.stiffness_n_per_m)
        soft_limit.CreateDampingAttr(compliance.damping_n_s_per_m)
        for letter in lateral_letters:
            lateral_limit = usd_physics.LimitAPI.Apply(prim, f"rot{letter}")
            lateral_limit.CreateLowAttr().Set(-compliance.lateral_limit_deg)
            lateral_limit.CreateHighAttr().Set(compliance.lateral_limit_deg)
    robot_schema.ApplyAttachmentPointAPI(prim)
    _set_attr(prim, sdf, ATTR_FORWARD_AXIS, "Token", forward_axis)
    _set_attr(prim, sdf, ATTR_CLEARANCE_OFFSET, "Float", float(clearance_offset_m))


def author_surface_gripper(
    robot_schema: Any,
    sdf: Any,
    stage: Any,
    gripper_path: str,
    attachment_paths: Sequence[str],
    max_grip_distance_m: float,
    coaxial_force_limit_n: float,
    shear_force_limit_n: float,
    retry_interval_s: float,
) -> Any:
    """The gripper prim itself, pointed at its attachment joints. Authored
    LAST, after the joints have been on the stage long enough for PhysX to
    build them, because the gripper caches each joint's PhysX object when it
    starts and a joint it cannot find stays locked to the anchor."""
    prim = robot_schema.CreateSurfaceGripper(stage, gripper_path)
    prim.GetRelationship(REL_ATTACHMENT_POINTS).SetTargets([sdf.Path(p) for p in attachment_paths])
    _set_attr(prim, sdf, ATTR_MAX_GRIP_DISTANCE, "Float", float(max_grip_distance_m))
    _set_attr(prim, sdf, ATTR_COAXIAL_FORCE_LIMIT, "Float", float(coaxial_force_limit_n))
    _set_attr(prim, sdf, ATTR_SHEAR_FORCE_LIMIT, "Float", float(shear_force_limit_n))
    _set_attr(prim, sdf, ATTR_RETRY_INTERVAL, "Float", float(retry_interval_s))
    return prim


def status_name(value: Any) -> str:
    """The gripper's status as one of Open, Closing, Closed, whatever shape
    the interface hands back: a token string, an enum with a name, or the
    enum's integer, which the C++ side numbers Open 0, Closing 1, Closed 2."""
    if isinstance(value, str):
        return value
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    if isinstance(value, int) and not isinstance(value, bool):
        return {0: "Open", 1: "Closing", 2: "Closed"}.get(value, str(value))
    return str(value)


def _floats(values: Any) -> tuple[float, ...]:
    return tuple(float(v) for v in values)


class SurfaceGripperSmokeRig:
    """Builds, drives and tears down a smoke gripper on a live stage. Every
    method runs on the sim thread; the world verb gets them there."""

    def __init__(self, sim: Any) -> None:
        self._sim = sim
        self._spec: SmokeSpec | None = None
        self._mount_link: str | None = None
        self._tool_prim: str | None = None
        self._joint_paths: list[str] = []
        self._wired = False
        self._iface: Any = None
        self._imports: dict[str, Any] | None = None

    # -- Isaac imports, once, reported rather than assumed --

    def _modules(self) -> dict[str, Any]:
        if self._imports is not None:
            return self._imports
        self._imports = compat.import_surface_gripper()
        return self._imports

    def _stage(self) -> Any:
        assert self._mount_link is not None
        return self._sim._isaac.get_prim_at_path(self._mount_link).GetStage()

    def _link_world_pose(self) -> tuple[Vec3, Quat]:
        assert self._mount_link is not None
        pos, quat = self._sim._isaac.SingleXFormPrim(self._mount_link).get_world_pose()
        x, y, z = _floats(pos)
        w, qx, qy, qz = _floats(quat)
        return (x, y, z), (w, qx, qy, qz)

    def _point_world_positions_mm(self) -> list[Vec3]:
        assert self._spec is not None
        pos, quat = self._link_world_pose()
        out: list[Vec3] = []
        for point in self._spec.points_m:
            dx, dy, dz = quat_rotate(quat, point)
            out.append(((pos[0] + dx) * 1000.0, (pos[1] + dy) * 1000.0, (pos[2] + dz) * 1000.0))
        return out

    # -- the steps --

    def author(
        self, spec: SmokeSpec, mount_link_path: str, tool_prim_path: str | None
    ) -> dict[str, Any]:
        """The ghost anchor and the attachment joints, nothing else yet. The
        gripper prim follows in wire(), once PhysX has had a few steps to
        build these joints, and restart() then makes Isaac notice it."""
        if self._spec is not None:
            self.cleanup()
        modules = self._modules()
        self._spec = spec
        self._mount_link = mount_link_path
        self._tool_prim = tool_prim_path
        stage = self._stage()
        link_pos, link_quat = self._link_world_pose()
        first = quat_rotate(link_quat, spec.points_m[0])
        anchor_pos = (link_pos[0] + first[0], link_pos[1] + first[1], link_pos[2] + first[2])
        stage.DefinePrim(SMOKE_SCOPE, "Scope")
        author_ghost_anchor(
            modules["UsdGeom"],
            modules["UsdPhysics"],
            modules["PhysxSchema"],
            modules["Gf"],
            stage,
            SMOKE_ANCHOR_PRIM,
            anchor_pos,
            link_quat,
        )
        self._joint_paths = attachment_joint_paths(SMOKE_SCOPE, len(spec.points_m))
        for path, point, offset in zip(
            self._joint_paths, spec.points_m, anchor_relative_offsets(spec.points_m), strict=True
        ):
            author_attachment_joint(
                modules["UsdPhysics"],
                modules["Sdf"],
                modules["Gf"],
                modules["robot_schema"],
                stage,
                path,
                mount_link_path,
                SMOKE_ANCHOR_PRIM,
                point,
                offset,
                FORWARD_AXIS,
                spec.clearance_offset_m,
            )
        LOGGER.info(
            "surface gripper smoke: authored anchor %s and %d attachment joint(s) on %s",
            SMOKE_ANCHOR_PRIM,
            len(self._joint_paths),
            mount_link_path,
        )
        return {
            "ok": True,
            "imports": dict(modules["report"]),
            "mount_link": mount_link_path,
            "tool_prim": tool_prim_path,
            "anchor": SMOKE_ANCHOR_PRIM,
            "joints": list(self._joint_paths),
            "link_world_mm": [v * 1000.0 for v in link_pos],
            "points_world_mm": [list(p) for p in self._point_world_positions_mm()],
        }

    def wire(self) -> dict[str, Any]:
        """The gripper prim, pointed at the joints author() made."""
        spec = self._require_spec()
        modules = self._modules()
        author_surface_gripper(
            modules["robot_schema"],
            modules["Sdf"],
            self._stage(),
            SMOKE_GRIPPER_PRIM,
            self._joint_paths,
            spec.max_grip_distance_m,
            spec.coaxial_force_limit_n,
            spec.shear_force_limit_n,
            spec.retry_interval_s,
        )
        self._iface = modules["surface_gripper"].acquire_surface_gripper_interface()
        self._wired = True
        LOGGER.info("surface gripper smoke: authored %s", SMOKE_GRIPPER_PRIM)
        return {"ok": True, "gripper": SMOKE_GRIPPER_PRIM, **self.status()}

    def restart(self) -> dict[str, Any]:
        """A world reset, so the gripper exists as far as Isaac is concerned.
        The surface gripper plugin finds its prims once, on the first physics
        frame after play, by asking the stage for every prim of its type. A
        gripper authored while the sim is playing is invisible to it until
        the next stop and play, which is what the 18:15 smoke run of
        2026-09-22 showed: every status read came back empty. The arm's own
        post-reset hook puts it back on its last targets."""
        self._require_wired()
        self._sim._reset_world()
        LOGGER.info("surface gripper smoke: world reset so the gripper prim is discovered")
        return {"ok": True, **self.status()}

    def close(self) -> dict[str, Any]:
        self._require_wired()
        accepted = self._iface.close_gripper(SMOKE_GRIPPER_PRIM)
        return {"accepted": bool(accepted), **self.status()}

    def open(self) -> dict[str, Any]:
        self._require_wired()
        accepted = self._iface.open_gripper(SMOKE_GRIPPER_PRIM)
        return {"accepted": bool(accepted), **self.status()}

    def status(self) -> dict[str, Any]:
        """What the gripper says through its interface, and what it wrote to
        the prim, side by side. Where the points are in the world is here so
        a reading can be set against the box's top face."""
        self._require_spec()
        out: dict[str, Any] = {
            "points_world_mm": [list(p) for p in self._point_world_positions_mm()]
        }
        if not self._wired:
            out["status"] = "not wired"
            return out
        out["status"] = status_name(self._iface.get_gripper_status(SMOKE_GRIPPER_PRIM))
        # the interface answers an empty string for a path it has no
        # component for, which is what a gripper authored mid-play reads as
        # until a restart
        out["registered"] = bool(out["status"])
        try:
            out["gripped_objects"] = [
                str(p) for p in self._iface.get_gripped_objects(SMOKE_GRIPPER_PRIM)
            ]
        except Exception as error:  # noqa: BLE001 - the reading is the point
            out["gripped_objects_error"] = f"{type(error).__name__}: {error}"
        prim = self._stage().GetPrimAtPath(SMOKE_GRIPPER_PRIM)
        attr = prim.GetAttribute(ATTR_STATUS) if prim.IsValid() else None
        out["status_on_prim"] = str(attr.Get()) if attr else None
        return out

    def cleanup(self) -> dict[str, Any]:
        """Open, then remove everything under the smoke scope. Safe to call
        with nothing authored."""
        removed: list[str] = []
        if self._mount_link is not None:
            stage = self._stage()
            if self._wired and self._iface is not None:
                try:
                    self._iface.open_gripper(SMOKE_GRIPPER_PRIM)
                except Exception as error:  # noqa: BLE001 - cleanup keeps going
                    LOGGER.warning("surface gripper smoke: open before cleanup failed: %s", error)
            if stage.GetPrimAtPath(SMOKE_SCOPE).IsValid():
                stage.RemovePrim(SMOKE_SCOPE)
                removed.append(SMOKE_SCOPE)
        self._spec = None
        self._mount_link = None
        self._tool_prim = None
        self._joint_paths = []
        self._wired = False
        self._iface = None
        return {"ok": True, "removed": removed}

    def step(self, name: str) -> Callable[[], dict[str, Any]]:
        steps: dict[str, Callable[[], dict[str, Any]]] = {
            "wire": self.wire,
            "restart": self.restart,
            "close": self.close,
            "open": self.open,
            "status": self.status,
            "cleanup": self.cleanup,
        }
        if name not in steps:
            expected = ", ".join(SMOKE_STEPS)
            raise ValueError(
                f"surface_gripper_smoke: unknown step {name!r}, expected one of {expected}"
            )
        return steps[name]

    def _require_spec(self) -> SmokeSpec:
        if self._spec is None:
            raise ValueError(
                "surface_gripper_smoke: nothing authored yet, run the 'author' step first"
            )
        return self._spec

    def _require_wired(self) -> None:
        self._require_spec()
        if not self._wired:
            raise ValueError(
                "surface_gripper_smoke: the gripper prim is not wired yet, run the 'wire' step"
            )
