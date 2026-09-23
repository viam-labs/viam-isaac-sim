"""The QC cell fragment has to say the same thing twice, consistently.

An arm is placed in Isaac by its `position` attribute and in Viam's frame system by its
`frame.translation` - the same point written twice, in metres and in millimetres.

Be honest about what this buys. The two cannot actually diverge at runtime:
`apply_frame_to_attrs()` overwrites `position` from the frame before the arm spawns, so
the frame wins and `position` is inert whenever a frame is present. This test therefore
keeps the file readable - a reader who edits one number and not the other is told - but it
does not protect against a mis-placed arm.

The placement risk that is real is rotational, and no config test can see it: the Viam SVA
is rooted in the UR controller's `base` frame while the Isaac USD is a URDF import rooted
in ROS `base_link`, and those differ by a rotation about Z. If they disagree, every planned
x/y is mirrored relative to the simulation. Phase 1 settles that by commanding all-zero
joints and comparing Isaac's end-effector world pose against `motion.GetPose` - mock mode
cannot, because the mock arm ignores `position` and returns a constant end pose.
"""

import json
from pathlib import Path

import pytest

from isaac_module.sim_manager import KNOWN_ASSETS

FRAGMENT = Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json"
MM_PER_M = 1000.0
TOLERANCE_MM = 1e-6


def load(kind):
    config = json.loads(FRAGMENT.read_text())
    return [c for c in config["components"] if c["type"] == kind]


def test_fragment_parses():
    assert load("arm"), "fragment defines no arms"


@pytest.mark.parametrize("arm", load("arm"), ids=lambda a: a["name"])
def test_isaac_position_matches_viam_frame(arm):
    position = arm["attributes"]["position"]
    translation = arm["frame"]["translation"]
    expected = [position[i] * MM_PER_M for i in range(3)]
    actual = [float(translation[axis]) for axis in ("x", "y", "z")]
    for axis, want, got in zip("xyz", expected, actual):
        assert abs(want - got) < TOLERANCE_MM, (
            f"{arm['name']}: isaac position {axis}={want / MM_PER_M} m is "
            f"{want} mm, but frame.translation {axis}={got} mm"
        )


@pytest.mark.parametrize("arm", load("arm"), ids=lambda a: a["name"])
def test_arm_asset_can_serve_kinematics(arm):
    """Without kinematics the motion service cannot plan, which is the whole point here.

    ur10/ur10e/ur16e are spawnable but have no kinematics file upstream, so they would
    boot happily and then refuse to plan.
    """
    asset = arm["attributes"]["asset"]
    assert asset in KNOWN_ASSETS, f"unknown asset {asset!r}"
    assert KNOWN_ASSETS[asset].get("kinematics"), (
        f"{asset!r} has no kinematics file; the motion service cannot plan for it. "
        f"Assets that can: "
        f"{sorted(a for a, m in KNOWN_ASSETS.items() if m.get('kinematics'))}"
    )


def test_every_component_names_the_world():
    components = json.loads(FRAGMENT.read_text())["components"]
    worlds = [c for c in components if c["model"].endswith(":world")]
    assert len(worlds) == 1, "expected exactly one world component"
    name = worlds[0]["name"]
    for component in components:
        if component["name"] == name:
            continue
        assert component["attributes"].get("world") == name, (
            f"{component['name']} does not point at the world {name!r}"
        )
