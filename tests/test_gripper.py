"""The suction gripper, exercised on the mock backend.

What is worth pinning here is the behaviour that differs from a kinematic attach, because
that difference is the whole reason to use isaac's surface gripper: a grab reports whether
it actually caught something, and it can say no.
"""

import asyncio

import pytest
from viam.proto.app.robot import ComponentConfig
from viam.utils import dict_to_struct

from isaac_module.models.gripper import IsaacGripper
from isaac_module.sim_manager import MockGripperHandle, SimConfig, SimManager


def config(**attrs):
    base = {"world": "sim-world", "parent_prim": "/World/arm_a/wrist_3_link/flange"}
    base.update(attrs)
    return ComponentConfig(name="suction", attributes=dict_to_struct(base))


@pytest.fixture
def gripper():
    manager = SimManager.get()
    previous_cfg, previous_mock = manager.cfg, manager.mock
    manager.cfg = SimConfig(mock=True)
    manager.mock = True
    manager._booted.set()
    manager._handles.pop("suction", None)
    try:
        yield IsaacGripper.new(config(grab_settle_sec=0.05), {})
    finally:
        manager._handles.pop("suction", None)
        manager.cfg, manager.mock = previous_cfg, previous_mock
        manager._booted.clear()


def test_parent_prim_is_required():
    """A cup has to hang off something; without it the joint has no body to anchor to."""
    bad = ComponentConfig(name="suction", attributes=dict_to_struct({"world": "w"}))
    with pytest.raises(ValueError, match="parent_prim"):
        IsaacGripper.validate_config(bad)


def test_starts_open(gripper):
    assert asyncio.run(gripper.is_moving()) is False
    assert asyncio.run(gripper.is_holding_something()).is_holding_something is False


def test_grab_reports_failure_when_nothing_is_there(gripper):
    """The mock grips nothing, on purpose.

    A mock that always reported success would let a whole round pass without a
    simulator and fail the first time it met one.
    """
    assert asyncio.run(gripper.grab()) is False


def test_open_after_close_returns_to_open(gripper):
    asyncio.run(gripper.grab())
    asyncio.run(gripper.open())
    assert asyncio.run(gripper.is_moving()) is False


def test_stop_does_not_drop_what_is_held():
    """stop() means "stop trying", not "let go" - a held part must stay held."""
    handle = MockGripperHandle("suction", {})
    handle.close()
    assert handle.status() == "Closed"


def test_geometry_is_reported_for_the_planner(gripper):
    """The cup is in neither the kinematics file nor the articulation.

    Unreported, a path clears the flange by a millimetre and puts the cup through the
    bench, with nothing to collide against.
    """
    geometries = asyncio.run(gripper.get_geometries())
    assert len(geometries) == 1
    assert geometries[0].capsule.length_mm > 0
    assert geometries[0].label.endswith("-cup")


def test_geometry_follows_the_configured_offset():
    manager = SimManager.get()
    previous_cfg, previous_mock = manager.cfg, manager.mock
    manager.cfg, manager.mock = SimConfig(mock=True), True
    manager._booted.set()
    manager._handles.pop("suction", None)
    try:
        g = IsaacGripper.new(
            config(offset=[0.0, 0.0, 0.02], geometry={"radius_mm": 10, "length_mm": 80}), {}
        )
        geom = asyncio.run(g.get_geometries())[0]
        assert geom.capsule.radius_mm == 10
        # centre sits half a cup beyond the offset, along the tool axis
        assert geom.center.z == pytest.approx(20.0 + 40.0)
    finally:
        manager._handles.pop("suction", None)
        manager.cfg, manager.mock = previous_cfg, previous_mock
        manager._booted.clear()


def test_joints_are_refused():
    """A suction cup has no controllable joints; pretending otherwise invents state."""
    g = IsaacGripper("suction")
    with pytest.raises(NotImplementedError):
        asyncio.run(g.get_current_inputs())
