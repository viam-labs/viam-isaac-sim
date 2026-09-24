"""The cell's geometry, checked before any of it reaches a robot.

These are the properties the cell's design rests on. They were previously implied by a
stored pose file that nothing re-derived, and by the time anyone looked it described a
different grip, a different part and a cell with none of the current scenery.

Pure arithmetic over `probes/qc/layout.py`, so it runs without Isaac and fails the moment
someone moves a tray.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

from qc.layout import REACH_MM, Cell, reach_budget  # noqa: E402

PICK_SIDE = "+y"
CARRY_SIDE = "-y"


@pytest.fixture(scope="module")
def cell():
    return Cell.from_fragment()


def test_only_one_grip_side_clears_the_belt(cell):
    """The belt runs away in -y, so only the +y side has room to stand off on.

    This is the constraint everything else hangs from: it is what forces arm-a onto the
    +y side, and it is the layout's doing rather than the gripper's.
    """
    assert cell.grip_clears_belt(PICK_SIDE)
    assert not cell.grip_clears_belt(CARRY_SIDE)


def test_the_pick_is_within_reach(cell):
    position, _ = cell.pick(PICK_SIDE)
    assert cell.reach_mm("arm-a", position) < reach_budget()


def test_arm_a_cannot_place_in_either_tray(cell):
    """The handoff is mandatory, and this is the whole reason the cell has two arms.

    Asserted at the FLANGE for the side arm-a actually holds. Measuring to the tray
    centre passes trivially and would have hidden the finding below.
    """
    for tray in ("good", "bad"):
        position, _ = cell.place(tray, PICK_SIDE)
        assert cell.reach_mm("arm-a", position) > REACH_MM, (
            f"arm-a can reach the {tray} tray holding the {PICK_SIDE} side; "
            "the handoff would be optional"
        )


def test_the_handoff_depends_on_which_side_is_held(cell):
    """Holding the other side, arm-a COULD reach the good tray - by about 75 mm.

    So the layout does not make the handoff mandatory on its own; the belt does, by
    leaving only the +y side approachable. Pinned here because it is the kind of fact
    that quietly stops being true when a tray moves, and because a test that only
    checked the grip actually used would never notice.
    """
    position, _ = cell.place("good", CARRY_SIDE)
    assert cell.reach_mm("arm-a", position) < REACH_MM


def test_arm_b_can_place_in_both_trays(cell):
    for tray in ("good", "bad"):
        position, _ = cell.place(tray, CARRY_SIDE)
        assert cell.reach_mm("arm-b", position) < reach_budget(), (
            f"arm-b cannot place in the {tray} tray, so nothing can"
        )


def test_the_two_arms_grip_opposite_sides(cell):
    """Otherwise the second check re-inspects a face the first one already saw."""
    assert PICK_SIDE != CARRY_SIDE
    pick, _ = cell.pick(PICK_SIDE)
    carry, _ = cell.side_grip(cell.part.centre_mm, CARRY_SIDE)
    assert (pick[1] - cell.part.centre_mm[1]) * (carry[1] - cell.part.centre_mm[1]) < 0


def test_the_part_fits_the_inspection_frame(cell):
    """A presentation the camera cannot frame is not an inspection.

    fov and working distance were chosen for an 80 mm cube; the mailbox is 175 mm on its
    longest edge, and the frame at this distance was 159 mm.
    """
    import math

    distance = math.dist(cell.camera_mm, cell.camera_target_mm)
    config = __import__("json").loads(
        (Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text())
    camera = next(c for c in config["components"] if c["name"] == "inspect-cam")
    fov = float(camera["attributes"]["fov_deg"])
    frame_mm = 2.0 * distance * math.tan(math.radians(fov) / 2.0)
    longest = max(cell.part.dims_mm)
    assert frame_mm > longest * 1.25, (
        f"frame is {frame_mm:.0f} mm and the part is {longest:.0f} mm on its longest "
        f"edge; presentations overflow the image"
    )


def test_the_part_fits_inside_the_box_the_planner_is_told_about():
    """Every piece of the part must sit inside its declared bounding box.

    The planner reasons about one box. A child reaching past it - the flag did, by
    30 mm - means paths get cleared that the real part does not fit through, and the
    failure looks like the arm stalling against nothing.
    """
    import json

    config = json.loads(
        (Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text())
    world = next(c for c in config["components"] if c["model"].endswith(":world"))
    part = next(p for p in world["attributes"]["props"] if p["name"] == "part")
    half = [v / 2 for v in part["scale"]]
    for child in part["children"]:
        position = child.get("position", [0, 0, 0])
        if child.get("shape", "cube") == "cube":
            extent = [v / 2 for v in child["scale"]]
        else:
            radius = child.get("radius", 0.05)
            height = child.get("height", 0.1)
            extent = [height / 2 if child.get("axis") == "X" else radius,
                      radius, radius]
        for axis in range(3):
            reach = abs(position[axis]) + extent[axis]
            assert reach <= half[axis] + 1e-6, (
                f"{child['name']} reaches {reach * 1000:.0f} mm on axis {axis}, "
                f"past the declared {half[axis] * 1000:.0f} mm"
            )


def test_both_presentation_points_are_in_frame(cell):
    """Each arm presents on its own side of the camera axis, and both must be visible.

    The offset is not a style choice: an arm holding a flat side stands off 175 mm
    beyond the part, so pushing the part toward the arm is the only way to put its
    flange somewhere it can actually work. That pushes the part off the camera's axis,
    and the lens has to cover it.
    """
    import json
    import math

    config = json.loads(
        (Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text())
    camera = next(c for c in config["components"] if c["name"] == "inspect-cam")
    distance = math.dist(cell.camera_mm, cell.camera_target_mm)
    half_frame = distance * math.tan(
        math.radians(float(camera["attributes"]["fov_deg"])) / 2.0)
    for side in ("+y", "-y"):
        point = cell.inspect_point_mm(side)
        reach = abs(point[1]) + max(cell.part.dims_mm) / 2.0
        assert reach < half_frame, (
            f"presenting at y={point[1]:.0f} puts the part's edge {reach:.0f} mm off "
            f"axis, outside the {half_frame:.0f} mm half-frame"
        )
