"""erh:isaac-sim:gripper - a simulated suction gripper.

Backed by isaac's surface gripper, which fabricates a physics joint to whatever body is
within reach of its attachment point when it closes. That matters: a grab can fail
because nothing was close enough, and a held part can be pulled off by its own weight or
by driving it into something. A kinematic re-parent would never do either, which is
exactly why a cell built on one cannot teach anything about grasping.

Attributes:
  world (string, required)       - name of the erh:isaac-sim:world component
  parent_prim (string, required) - the prim the cup hangs off, usually an arm's flange,
                                   e.g. "/World/arm_a/wrist_3_link/flange"
  offset ([x,y,z] meters)        - where the cup sits in PARENT_PRIM's own frame, which
                                   is not the frame viam plans in. Measured on isaac's
                                   ur5e: commanding the end-effector frame tool-down puts
                                   the flange prim's local +x pointing down, so the tool
                                   runs along +x there - the urdf ee_link convention -
                                   while viam's SVA puts the same tool along the
                                   end-effector frame's +z. Same cup, two frames, two
                                   axes. Getting this wrong points the suction sideways
                                   and every grasp silently misses.
  max_grip_distance (m)          - how close a body must be to be gripped (default 0.01)
  coaxial_force_limit (N)        - pull-off force along the cup's axis (default 50)
  shear_force_limit (N)          - sideways force before it slips (default 50)
  retry_interval_sec (float)     - how long a close keeps trying (default 0.5)
  grab_settle_sec (float)        - how long grab() waits for the joint to form (default 0.6)
  geometry ({radius_mm,length_mm}) - the cup, reported through GetGeometries so the
                                   motion service can plan around it.
"""

import asyncio
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.gripper import Gripper
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Capsule, Geometry, Pose, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

from .. import FAMILY, NAMESPACE
from ..sim_manager import GripperHandle, SimManager
from .utils import get_attrs, validate_sim_component

_DEFAULT_RADIUS_MM = 25.0
_DEFAULT_LENGTH_MM = 120.0


class IsaacGripper(Gripper, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(NAMESPACE, FAMILY), "gripper")

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._handle: Optional[GripperHandle] = None
        self._attrs: Dict[str, Any] = {}
        self._settle = 0.6

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        gripper = cls(config.name)
        gripper.reconfigure(config, dependencies)
        return gripper

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        required, optional = validate_sim_component(config, needs_source=False)
        attrs = struct_to_dict(config.attributes)
        if not attrs.get("parent_prim"):
            raise ValueError(
                f"gripper {config.name}: needs parent_prim, the prim the cup hangs "
                "off (usually an arm's flange)"
            )
        return required, optional

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        attrs = get_attrs(config)
        self._attrs = attrs
        self._settle = float(attrs.get("grab_settle_sec", 0.6))
        self._handle = SimManager.get().create_gripper(self.name, attrs)

    def _h(self) -> GripperHandle:
        if self._handle is None:
            raise RuntimeError(f"gripper {self.name} is not attached to the sim")
        return self._handle

    async def open(self, **kwargs) -> None:
        await asyncio.to_thread(self._h().open)

    async def grab(self, **kwargs) -> bool:
        """Close, wait for the joint to form, and report whether anything was caught.

        The wait is not politeness. Closing only starts the attempt - isaac keeps trying
        for `retry_interval` - so asking immediately would report failure on a grasp that
        was about to succeed.
        """
        handle = self._h()
        await asyncio.to_thread(handle.close)
        deadline = asyncio.get_running_loop().time() + self._settle
        while asyncio.get_running_loop().time() < deadline:
            if await asyncio.to_thread(handle.gripped_objects):
                return True
            await asyncio.sleep(0.05)
        return bool(await asyncio.to_thread(handle.gripped_objects))

    async def stop(self, **kwargs) -> None:
        """Stop trying to grip, without dropping what is already held.

        A gripper that is Closing is mid-attempt; opening it would be a different
        instruction. One that is Closed is holding something, and stop() must not throw
        the part on the floor.
        """
        if await asyncio.to_thread(self._h().status) == "Closing":
            await asyncio.to_thread(self._h().open)

    async def is_moving(self) -> bool:
        return await asyncio.to_thread(self._h().status) == "Closing"

    async def is_holding_something(self, **kwargs) -> Gripper.HoldingStatus:
        objects = await asyncio.to_thread(self._h().gripped_objects)
        return Gripper.HoldingStatus(
            is_holding_something=bool(objects),
            meta={"gripped": list(objects)},
        )

    async def get_geometries(self, **kwargs) -> List[Geometry]:
        """The cup, so the motion service can plan around it.

        The tool is invisible to the planner otherwise: it is in neither the arm's
        kinematics file nor the isaac articulation, so a path can clear the flange by a
        millimetre and put the cup through the bench.
        """
        # Reported in the gripper's own frame, where +z is the tool axis - the viam
        # convention, NOT the parent prim's +x that `offset` uses. The two are only the
        # same cup seen from two frames.
        spec = self._attrs.get("geometry") or {}
        radius = float(spec.get("radius_mm", _DEFAULT_RADIUS_MM))
        length = float(spec.get("length_mm", _DEFAULT_LENGTH_MM))
        offset = [float(v) * 1000.0 for v in (self._attrs.get("offset") or (0, 0, 0))]
        return [
            Geometry(
                center=Pose(
                    x=offset[0], y=offset[1], z=offset[2] + length / 2.0,
                    o_x=0, o_y=0, o_z=1, theta=0,
                ),
                capsule=Capsule(radius_mm=radius, length_mm=length),
                label=f"{self.name}-cup",
            )
        ]

    async def get_kinematics(self, **kwargs):
        raise NotImplementedError(
            "a suction cup has no joints; it is rigid relative to parent_prim"
        )

    async def get_current_inputs(self, **kwargs) -> List[float]:
        raise NotImplementedError("a suction gripper has no controllable joints")

    async def go_to_inputs(self, values: List[float], **kwargs) -> None:
        raise NotImplementedError("a suction gripper has no controllable joints")

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        cmd = str(command.get("command", ""))
        if cmd == "status":
            return {
                "status": await asyncio.to_thread(self._h().status),
                "gripped": list(await asyncio.to_thread(self._h().gripped_objects)),
            }
        raise ValueError(f"unknown command {cmd!r}; supported: status")
