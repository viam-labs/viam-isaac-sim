"""The QC cell fragment has to say the same thing twice, consistently.

An arm is placed in Isaac by its `position` attribute and in Viam's frame system by its
`frame.translation`. Nothing links them: they are two independent numbers describing the
same point, in metres and in millimetres. If they drift apart the module still boots and
the arm still moves, but the motion service plans against the frame system while the
collisions happen in Isaac - so it confidently returns paths that are clear in a world
that is not the one being simulated, and the failure looks like a planner bug rather than
a config typo.

That is the single largest correctness risk in moving the cell onto this module, and it
costs nothing to check, so it is checked here rather than discovered in a render.
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
