"""The obstacle list the planner depends on, checked against the cell's real props.

`SimManager.prop_obstacles()` is the only thing standing between the motion service and
planning straight through the scenery, and it is pure arithmetic over the config, so it
does not need isaac to be checked. What it has to get right is the box: a cube prop's
edge is `size`, scaled per axis, in metres, and viam's geometry messages are full
dimensions in millimetres. Getting `size` and `scale` the wrong way round, or reporting a
half-extent, would silently shrink or inflate every obstacle in the cell.
"""

import json
from pathlib import Path

import pytest

from isaac_module.sim_manager import SimConfig, SimManager

FRAGMENT = Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json"


@pytest.fixture
def obstacles():
    """Run the real config's props through the real method."""
    components = json.loads(FRAGMENT.read_text())["components"]
    world = next(c for c in components if c["model"].endswith(":world"))
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(props=[dict(p) for p in world["attributes"]["props"]])
    try:
        yield {o["label"]: o for o in manager.prop_obstacles()}
    finally:
        manager.cfg = previous


def test_every_cube_prop_is_reported(obstacles):
    assert {"belt", "good_tray", "bad_tray"} <= set(obstacles)


def test_the_floor_is_reported(obstacles):
    """boot() adds a ground plane for any scene without its own stage.

    It is a collider like any other, and an arm on a pedestal has plenty of
    configurations that reach below its own base. Unreported, the planner routes an
    elbow into it and the arm stalls pressing on the floor - which looks like a
    mysterious failure to settle rather than a collision.
    """
    ground = obstacles["ground"]
    assert ground["fixed"]
    top = ground["center_mm"][2] + ground["dims_mm"][2] / 2
    assert top == pytest.approx(0.0), "the floor's top face should be z=0"
    assert min(ground["dims_mm"][:2]) > 5000, "the floor should span the whole cell"


def test_dimensions_are_full_extents_in_mm(obstacles):
    # belt: size 1.0 scaled by [0.2584, 0.836, 0.076] -> 258.4 x 836 x 76 mm.
    belt = obstacles["belt"]
    assert belt["dims_mm"] == pytest.approx([258.4, 836.0, 76.0])
    assert belt["center_mm"] == pytest.approx([547.2, -798.0, 570.0])


def test_scale_applies_per_axis(obstacles):
    # good_tray: size 0.22, scale [1, 1, 0.23] -> square in plan, thin in z.
    tray = obstacles["good_tray"]
    assert tray["dims_mm"][0] == pytest.approx(tray["dims_mm"][1])
    assert tray["dims_mm"][2] < tray["dims_mm"][0] / 2


def test_fixed_is_carried_through(obstacles):
    """`fixed` is what tells a caller whether a prop is scenery or cargo.

    A fixed prop is an obstacle for the whole round. A dynamic one is an obstacle until
    an arm picks it up, and then becomes part of that arm.
    """
    assert all(o["fixed"] for o in obstacles.values()), "the cell's scenery is all fixed"


def test_a_scene_with_its_own_stage_reports_no_floor():
    """A usd_stage brings its own floor, if it has one - do not invent a second."""
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(usd_stage="/scene.usd", props=[])
    try:
        assert manager.prop_obstacles() == []
    finally:
        manager.cfg = previous


def test_no_props_means_no_obstacles():
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(props=[])
    try:
        # The ground plane is still there - it is part of the scene, not a prop.
        assert [o["label"] for o in manager.prop_obstacles()] == ["ground"]
    finally:
        manager.cfg = previous


def test_usd_props_are_skipped():
    """A usd prop's extent lives in the asset, so no box is guessed for it."""
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(props=[
        {"name": "crate", "type": "usd", "usd_path": "/x.usd", "position": [0, 0, 0]},
        {"name": "block", "type": "cube", "size": 0.1, "position": [1, 0, 0]},
    ])
    try:
        labels = [o["label"] for o in manager.prop_obstacles()]
        assert labels == ["ground", "block"]
    finally:
        manager.cfg = previous


def test_prop_poses_reports_configured_positions_in_mock():
    """Without a stage there is nothing to read, so mock reports where props were put.

    It is flagged `live: False` so a caller can tell a spawn position from a measured
    one - the difference matters the moment anything moves.
    """
    manager = SimManager.get()
    previous_cfg, previous_mock = manager.cfg, manager.mock
    manager.cfg = SimConfig(props=[
        {"name": "part", "type": "cube", "size": 0.08, "position": [0.5, -0.4, 0.66]},
    ])
    manager.mock = True
    manager._booted.set()
    try:
        poses = manager.prop_poses()
        assert poses["part"]["position_mm"] == pytest.approx([500.0, -400.0, 660.0])
        assert poses["part"]["live"] is False
        assert manager.prop_poses(["nothing"]) == {}
    finally:
        manager.cfg, manager.mock = previous_cfg, previous_mock
        manager._booted.clear()
