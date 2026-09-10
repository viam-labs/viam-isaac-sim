"""Patches for gaps in the installed viam-sdk.

Resource creation is synchronous in the SDK's async module handler. Move it
off the event loop because Isaac resource factories can wait for Kit.

MoveThroughJointPositions: RDK's motion service executes its planned arm
trajectories through this RPC, but viam-sdk (through at least 0.80.0) never
implemented the server-side handler - the generated routing exists and falls
through to the UNIMPLEMENTED default, which breaks motion for every python
modular arm. We add the missing handler to ArmRPCService, mirroring the
SDK's own handler conventions; it dispatches to the resource's
move_through_joint_positions method (which IsaacArm implements).

Remove each patch when the SDK provides its equivalent behavior.
"""
import asyncio

from grpclib.metadata import Deadline
from viam.logging import update_log_level
from viam.module.module import Module
from viam.proto.app.robot import ComponentConfig
from viam.proto.module import AddResourceRequest
from viam.resource.registry import Registry
from viam.resource.types import API, Model
from viam.components.arm.service import ArmRPCService
from viam.logging import getLogger
from viam.proto.component.arm import (
    MoveThroughJointPositionsRequest,
    MoveThroughJointPositionsResponse,
)
from viam.utils import struct_to_dict

LOGGER = getLogger("viam-isaac-sim.patches")


async def _move_through_joint_positions(self, stream) -> None:
    request: MoveThroughJointPositionsRequest = await stream.recv_message()
    assert request is not None
    arm = self.get_resource(request.name)
    timeout = stream.deadline.time_remaining() if stream.deadline else None
    await arm.move_through_joint_positions(
        list(request.positions),
        extra=struct_to_dict(request.extra),
        timeout=timeout,
        metadata=stream.metadata,
    )
    await stream.send_message(MoveThroughJointPositionsResponse())

async def _add_resource(self, request: AddResourceRequest, *, deadline: Deadline | None = None):
    """Create resources off the module event loop."""
    dependencies = await self._get_dependencies(request.dependencies)
    config: ComponentConfig = request.config
    api = API.from_string(config.api)
    model = Model.from_string(config.model, ignore_errors=True)
    creator = Registry.lookup_resource_creator(api, model)
    resource = await asyncio.to_thread(creator, config, dependencies)
    if deadline is not None and deadline.time_remaining() <= 0:
        raise TimeoutError("Deadline expired")
    update_log_level(resource.logger, config.log_configuration.level.upper())
    self.server.register(resource)


def apply() -> None:
    Module.add_resource = _add_resource
    LOGGER.info("patched Module.add_resource to offload resource construction")

    existing = getattr(ArmRPCService, "MoveThroughJointPositions", None)
    if existing is not None and not getattr(existing, "__isabstractmethod__", False):
        # only patch if the SDK's handler is the raising placeholder
        qualname = getattr(existing, "__qualname__", "")
        if not qualname.startswith(("ArmServiceBase", "UnimplementedArmServiceBase")):
            LOGGER.info("sdk provides MoveThroughJointPositions; not patching")
            return
    ArmRPCService.MoveThroughJointPositions = _move_through_joint_positions
    LOGGER.info("patched ArmRPCService with MoveThroughJointPositions handler")
