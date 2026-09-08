"""End-to-end test of the module in mock mode: boots the SimManager on a
background thread (standing in for the process main thread) and exercises
the viam component models against it."""

import asyncio
import math
import threading

import pytest
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Vector3
from viam.utils import dict_to_struct

from isaac_module.models.arm import IsaacArm
from isaac_module.models.base import IsaacBase
from isaac_module.models.camera import IsaacCamera
from isaac_module.models.world import IsaacWorld
from isaac_module.sim_manager import SimManager


def _config(name: str, attrs: dict) -> ComponentConfig:
    return ComponentConfig(name=name, attributes=dict_to_struct(attrs))


@pytest.fixture(scope="module")
def sim():
    manager = SimManager.get()
    t = threading.Thread(target=manager.main_loop, daemon=True)
    t.start()
    yield manager
    manager.request_stop()
    t.join(timeout=5)


@pytest.fixture(scope="module")
def world(sim):
    return IsaacWorld.new(_config("sim-world", {"mock": True}), {})


def test_world_boots_and_status(world):
    status = asyncio.run(world.do_command({"command": "status"}))
    assert status["booted"] is True
    assert status["mock"] is True


def test_validate_requires_world():
    with pytest.raises(ValueError, match="world"):
        IsaacArm.validate_config(_config("a", {"asset": "ur20"}))


def test_validate_requires_source():
    with pytest.raises(ValueError, match="asset"):
        IsaacArm.validate_config(_config("a", {"world": "sim-world"}))


def test_validate_ok_returns_dependency():
    deps, _ = IsaacArm.validate_config(
        _config("a", {"world": "sim-world", "asset": "ur20"})
    )
    assert list(deps) == ["sim-world"]


def test_arm_moves(world):
    arm = IsaacArm.new(
        _config("my-arm", {"world": "sim-world", "asset": "ur20", "mock_dof": 6}), {}
    )

    async def scenario():
        start = await arm.get_joint_positions()
        assert start.values == pytest.approx([0.0] * 6)

        from viam.components.arm import JointPositions

        target = JointPositions(values=[10, -20, 30, 0, 5, -5])
        await arm.move_to_joint_positions(target)
        assert not await arm.is_moving()
        end = await arm.get_joint_positions()
        assert end.values == pytest.approx([10, -20, 30, 0, 5, -5], abs=0.5)

        pose = await arm.get_end_position()
        assert pose.x == pytest.approx(300.0)
        assert pose.o_z == pytest.approx(1.0)

        # trajectory execution (what the motion service calls)
        waypoints = [
            JointPositions(values=[5, -10, 15, 0, 2, -2]),
            JointPositions(values=[0, 0, 0, 0, 0, 0]),
        ]
        await arm.move_through_joint_positions(waypoints)
        end = await arm.get_joint_positions()
        assert end.values == pytest.approx([0] * 6, abs=0.5)

    asyncio.run(scenario())


def test_camera_returns_image(world):
    cam = IsaacCamera.new(
        _config("my-cam", {"world": "sim-world", "width": 320, "height": 240}), {}
    )

    async def scenario():
        img = await cam.get_image()
        assert img.mime_type == "image/jpeg"
        assert len(img.data) > 100

        from PIL import Image
        from io import BytesIO

        decoded = Image.open(BytesIO(img.data))
        assert decoded.size == (320, 240)

        images, _meta = await cam.get_images()
        assert len(images) == 1

        props = await cam.get_properties()
        assert props.supports_pcd is False

    asyncio.run(scenario())


def test_base_drives(world):
    base = IsaacBase.new(
        _config("my-base", {"world": "sim-world", "asset": "jetbot"}), {}
    )

    async def scenario():
        assert not await base.is_moving()
        await base.set_velocity(Vector3(x=0, y=200, z=0), Vector3(x=0, y=0, z=45))
        assert await base.is_moving()
        await base.stop()
        assert not await base.is_moving()

        # short timed move
        await base.move_straight(distance=10, velocity=100)
        assert not await base.is_moving()

        props = await base.get_properties()
        assert props.wheel_circumference_meters == pytest.approx(2 * math.pi * 0.05)

    asyncio.run(scenario())


def test_stale_arm_scene_entry_is_removed_before_reconfigure():
    class Scene:
        def __init__(self):
            self.objects = {"pick-arm"}
            self.removals = []

        def object_exists(self, name):
            return name in self.objects

        def remove_object(self, name, registry_only=False):
            self.removals.append((name, registry_only))
            self.objects.remove(name)

        def add(self, name):
            if name in self.objects:
                raise ValueError("name is not unique")
            self.objects.add(name)

    class World:
        def __init__(self, scene):
            self.scene = scene

    scene = Scene()
    manager = SimManager()
    manager.world = World(scene)

    manager._remove_stale_scene_object("arm", "pick-arm")
    scene.add("pick-arm")

    assert scene.removals == [("pick-arm", True)]
