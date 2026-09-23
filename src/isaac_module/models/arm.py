"""erh:isaac-sim:arm - a simulated arm.

Attributes:
  world (string, required)   - name of the erh:isaac-sim:world component
  asset (string)             - known robot, e.g. "ur20", "ur10", "franka"
  usd_path (string)          - explicit USD to spawn instead of a known asset
  prim_path (string)         - where to place it (default /World/<name>), or
                               an existing articulation in the stage
  position ([x,y,z] meters)  - spawn position
  end_effector_prim (string) - prim path whose world pose is reported by
                               GetEndPosition
  ignore_base_rotation (bool)- skip the known asset's fixed base rotation. UR assets
                               in isaac are URDF imports rooted at ROS base_link while
                               viam's kinematics use the controller's base frame, a half
                               turn apart, so the module composes that offset into the
                               spawn. Set this only for a USD you have already authored
                               in the kinematics frame.
  move_timeout_sec (float)   - max time to wait for a move (default 30)
  joint_limit_deg (float)    - NOTE: if an arm is already outside the narrowed range
                               when this is turned on, every motion.Move fails its start
                               check until the arm is driven back in bounds with
                               move_to_joint_positions, which is allowed to head inwards.
                               narrow every revolute joint's range in the kinematics
                               served to the motion service to +-this many degrees.
                               UR arms allow +-360, and the planner will happily pick
                               multi-turn solutions that accumulate until the arm jams;
                               180 covers every orientation and prevents that. SVA only
                               (a urdf is passed through), and it guards the planner,
                               not isaac - a direct move_to_joint_positions can still
                               wind a joint up.
  kinematics_url (string)    - where to fetch the kinematics file served by
                               GetKinematics (.json = SVA, .urdf = URDF;
                               file:// URLs work). Known assets with official
                               viam kinematics (ur3e/ur5e/ur20) fetch them
                               automatically.
"""

import asyncio
import hashlib
import json
import math
import os
import tempfile
import time
import urllib.request
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.arm import Arm, JointPositions, KinematicsFileFormat, Pose
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes

from .. import FAMILY, NAMESPACE
from ..sim_manager import KNOWN_ASSETS, ArmHandle, SimManager
from ..spatial import quat_to_ov
from .utils import apply_frame_to_attrs, get_attrs, validate_sim_component

_TOLERANCE_RAD = math.radians(0.5)


class IsaacArm(Arm, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(NAMESPACE, FAMILY), "arm")

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._handle: Optional[ArmHandle] = None
        self._attrs: Dict[str, Any] = {}
        self._move_timeout = 30.0
        self._kinematics: Optional[Tuple[KinematicsFileFormat.ValueType, bytes]] = None

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        arm = cls(config.name)
        arm.reconfigure(config, dependencies)
        return arm

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        return validate_sim_component(config)

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        attrs = apply_frame_to_attrs(config, get_attrs(config))
        self._move_timeout = float(attrs.get("move_timeout_sec", 30.0))
        self._attrs = attrs
        self._handle = SimManager.get().create_arm(self.name, attrs)

    def _h(self) -> ArmHandle:
        if self._handle is None:
            raise RuntimeError(f"arm {self.name} is not attached to the sim")
        return self._handle

    async def get_end_position(self, **kwargs) -> Pose:
        (x, y, z), quat = await asyncio.to_thread(self._h().get_end_pose)
        ox, oy, oz, theta = quat_to_ov(quat)
        return Pose(
            x=x * 1000.0,
            y=y * 1000.0,
            z=z * 1000.0,
            o_x=ox,
            o_y=oy,
            o_z=oz,
            theta=math.degrees(theta),
        )

    async def move_to_position(self, pose: Pose, **kwargs) -> None:
        raise NotImplementedError(
            "IK and motion planning are Viam's job, not the module's: use the "
            "motion service (needs GetKinematics, on the roadmap) or "
            "move_to_joint_positions"
        )

    async def move_to_joint_positions(
        self, positions: JointPositions, **kwargs
    ) -> None:
        targets = [math.radians(v) for v in positions.values]
        handle = self._h()
        await asyncio.to_thread(handle.set_joint_targets, targets)

        deadline = time.monotonic() + self._move_timeout
        while time.monotonic() < deadline:
            current = await asyncio.to_thread(handle.get_joint_positions)
            if len(current) >= len(targets) and all(
                abs(c - t) <= _TOLERANCE_RAD for c, t in zip(current, targets)
            ):
                return
            await asyncio.sleep(0.05)
        # Stop before raising. The last set_joint_targets stays applied otherwise, so a
        # blocked arm goes on pressing into whatever stopped it - the ground, the belt,
        # the other arm - at full drive force for as long as the sim runs.
        await asyncio.to_thread(handle.stop)
        raise TimeoutError(
            f"arm {self.name} did not reach target within {self._move_timeout}s"
        )

    async def move_through_joint_positions(
        self, positions: Sequence[JointPositions], *args, **kwargs
    ) -> None:
        """Execute a trajectory - this is what the motion service calls to run
        its planned paths. Intermediate waypoints use a loose tolerance so the
        arm flows through them; the final waypoint settles tight."""
        handle = self._h()
        waypoints = list(positions)
        loose = math.radians(2.0)
        for i, wp in enumerate(waypoints):
            targets = [math.radians(v) for v in wp.values]
            await asyncio.to_thread(handle.set_joint_targets, targets)
            last = i == len(waypoints) - 1
            tolerance = _TOLERANCE_RAD if last else loose
            deadline = time.monotonic() + (self._move_timeout if last else 10.0)
            current: List[float] = []
            while time.monotonic() < deadline:
                current = await asyncio.to_thread(handle.get_joint_positions)
                if len(current) >= len(targets) and all(
                    abs(c - t) <= tolerance for c, t in zip(current, targets)
                ):
                    break
                await asyncio.sleep(0.02)
            else:
                detail = ", ".join(
                    f"j{j}: at {math.degrees(c):.1f} want {math.degrees(t):.1f}"
                    for j, (c, t) in enumerate(zip(current, targets))
                    if abs(c - t) > tolerance
                )
                # A missed waypoint fails the move, intermediate ones included.
                #
                # This used to log a warning and drive on to the next waypoint from
                # wherever the arm had actually got to - which is off the trajectory the
                # motion service collision-checked, through a cell it cleared for a
                # different path. That is precisely the unchecked motion this module
                # exists to avoid, and it happened silently.
                #
                # Intermediate waypoints keep the loose 2 degree tolerance, so this only
                # fires when the arm is genuinely stuck rather than merely flowing through
                # a corner, and the message names the waypoint and the offending joints so
                # the failure can be diagnosed instead of just retried.
                await asyncio.to_thread(handle.stop)
                raise TimeoutError(
                    f"arm {self.name} stalled at waypoint {i + 1}/{len(waypoints)} "
                    f"({'final' if last else 'intermediate'}, tolerance "
                    f"{math.degrees(tolerance):.1f} deg; stuck joints: {detail})"
                )

    async def get_joint_positions(self, **kwargs) -> JointPositions:
        radians = await asyncio.to_thread(self._h().get_joint_positions)
        return JointPositions(values=[math.degrees(r) for r in radians])

    async def stop(self, **kwargs) -> None:
        await asyncio.to_thread(self._h().stop)

    async def is_moving(self) -> bool:
        return await asyncio.to_thread(self._h().is_moving)

    def _kinematics_url(self) -> Optional[str]:
        url = self._attrs.get("kinematics_url")
        if url:
            return str(url)
        asset = self._attrs.get("asset")
        if asset and asset in KNOWN_ASSETS:
            return KNOWN_ASSETS[asset].get("kinematics")
        return None

    def _load_kinematics(self) -> Tuple[KinematicsFileFormat.ValueType, bytes]:
        url = self._kinematics_url()
        if not url:
            raise NotImplementedError(
                f"no kinematics file known for arm {self.name}; set the "
                '"kinematics_url" attribute (SVA .json or .urdf)'
            )
        ext = os.path.splitext(url)[1].lower()
        fmt = (
            KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF
            if ext in (".urdf", ".xml", ".xacro")
            else KinematicsFileFormat.KINEMATICS_FILE_FORMAT_SVA
        )

        cache_dir = os.environ.get("VIAM_MODULE_DATA") or tempfile.gettempdir()
        cache = os.path.join(
            cache_dir,
            f"kinematics-{hashlib.sha1(url.encode()).hexdigest()[:12]}{ext}",
        )
        if os.path.exists(cache):
            with open(cache, "rb") as f:
                return fmt, f.read()

        self.logger.info("fetching kinematics for %s from %s", self.name, url)
        with urllib.request.urlopen(url, timeout=30) as resp:
            data = resp.read()
        try:
            os.makedirs(cache_dir, exist_ok=True)
            tmp = cache + ".tmp"
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, cache)
        except OSError:
            pass  # caching is best-effort
        return fmt, data

    def _clamp_joint_limits(
        self, fmt: KinematicsFileFormat.ValueType, data: bytes
    ) -> bytes:
        """Narrow the joint ranges the motion service is told about.

        The UR SVA allows every revolute joint +-360 degrees, which is true of the real
        robot. Viam's planner may pick any solution inside that range and has no reason to
        prefer the one nearest the arm's current pose, so a two-arm cell winds itself up:
        measured over one pass of this cell's stations, a joint went from -26 to 206
        degrees, and after a second pass an arm sat at 270. Approaching 360 the cell fails
        three different-looking ways - the planner cannot find a route, the arm cannot
        settle and times out, or the move is rejected as out of range - which is why the
        symptom looked intermittent.

        Clamping costs no reachable pose: +-180 degrees of a revolute joint already covers
        every orientation. Surveyed across this cell's stations the solutions needed at
        most [162, 174, 104, 157, 90] degrees on the first five joints, and 302 on wrist_3
        - which is the wind-up itself, 302 being the same place as -58.

        This is a guard on what the PLANNER believes, not something isaac enforces:
        set_joint_targets will still drive a joint anywhere it is told, so a direct
        move_to_joint_positions can still wind up. Sending the arms to a known
        configuration between rounds is what covers that path.

        Only SVA json is rewritten. A urdf is passed through untouched.
        """
        limit = self._attrs.get("joint_limit_deg")
        if limit is None:
            return data
        limit = abs(float(limit))
        if fmt != KinematicsFileFormat.KINEMATICS_FILE_FORMAT_SVA:
            self.logger.warning(
                "%s: joint_limit_deg is only applied to SVA kinematics; "
                "this arm serves a urdf, so the limit is being ignored", self.name,
            )
            return data

        model = json.loads(data)
        narrowed = []
        for joint in model.get("joints", []):
            if joint.get("type") != "revolute":
                continue
            before = (joint.get("min"), joint.get("max"))
            joint["min"] = max(float(joint.get("min", -limit)), -limit)
            joint["max"] = min(float(joint.get("max", limit)), limit)
            if (joint["min"], joint["max"]) != before:
                narrowed.append(f"{joint.get('id')} {before} -> "
                                f"({joint['min']}, {joint['max']})")
        if narrowed:
            self.logger.info("%s: narrowed joint limits to +-%g deg: %s",
                             self.name, limit, "; ".join(narrowed))
        return json.dumps(model).encode()

    async def get_kinematics(self, **kwargs) -> Tuple[KinematicsFileFormat.ValueType, bytes]:
        if self._kinematics is None:
            fmt, data = await asyncio.to_thread(self._load_kinematics)
            # Clamp here rather than in _load_kinematics: that caches the fetched file on
            # disk, and the cache must hold what upstream served, not this arm's config.
            self._kinematics = (fmt, self._clamp_joint_limits(fmt, data))
        return self._kinematics

    async def get_geometries(self, **kwargs) -> List[Geometry]:
        return []

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        if command.get("command") == "get_joint_positions_radians":
            return {"values": await asyncio.to_thread(self._h().get_joint_positions)}
        raise ValueError(f"unknown command: {command}")
