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
  move_timeout_sec (float)   - max time to wait for a move (default 30)
  kinematics_url (string)    - where to fetch the kinematics file served by
                               GetKinematics (.json = SVA, .urdf = URDF;
                               file:// URLs work). Known assets with official
                               viam kinematics (ur3e/ur5e/ur20) fetch them
                               automatically.
"""

import asyncio
import hashlib
import math
import os
import tempfile
import time
import urllib.request
from typing import (
    Any,
    AsyncIterator,
    ClassVar,
    Dict,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
)

from typing_extensions import Self
from viam.components.arm import Arm, JointPositions, KinematicsFileFormat, Mesh, Pose
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
                if last:
                    raise TimeoutError(
                        f"arm {self.name} stalled at waypoint {i + 1}/{len(waypoints)} "
                        f"(stuck joints: {detail})"
                    )
                self.logger.warning(
                    "%s: waypoint %d/%d not reached, continuing (%s)",
                    self.name, i + 1, len(waypoints), detail,
                )

    async def move_through_joint_positions_streamed(  # type: ignore
        self,
        batches: AsyncIterator[List[Arm.TrajectoryPoint]],
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> AsyncIterator[Arm.TrajectoryUpdate]:
        raise NotImplementedError(
            f"arm {self.name} does not support move_through_joint_positions_streamed"
        )
        # unreachable, but makes this an async generator so the RPC handler's
        # `async for` surfaces the NotImplementedError instead of a TypeError
        yield Arm.TrajectoryUpdate()

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

    async def get_kinematics(self, **kwargs) -> Tuple[KinematicsFileFormat.ValueType, bytes]:
        if self._kinematics is None:
            self._kinematics = await asyncio.to_thread(self._load_kinematics)
        return self._kinematics

    async def get_3d_models(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, Mesh]:
        raise NotImplementedError(f"arm {self.name} does not support get_3d_models")

    async def set_manual_mode(
        self,
        manual_mode: bool,
        enabled_for: int = 0,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> None:
        raise NotImplementedError(f"arm {self.name} does not support manual mode")

    async def get_manual_mode(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> bool:
        raise NotImplementedError(f"arm {self.name} does not support manual mode")

    async def get_properties(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Arm.Properties:
        raise NotImplementedError(f"arm {self.name} does not support get_properties")

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
