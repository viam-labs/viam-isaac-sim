"""Where the cell's stations are, computed from the cell.

Station poses used to live in `stations.json`, and by the time anyone looked they encoded
a tool-down grip the cell no longer uses, an 80 mm cube the part no longer is, and a scene
with none of the eighteen props now in it. One station had started failing IK and the rest
were wrong in ways nothing checked. A stored pose cannot notice that the cell moved.

So nothing here is stored. Everything is derived from the fragment - the one place the
cell is actually described - and `tests/test_cell_layout.py` asserts the two properties
the cell's design depends on before any of it reaches a robot.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

FRAGMENT = Path(__file__).resolve().parents[2] / "fragments" / "qc-cell.json"

# Measured from the motion service, not a datasheet: asking the planner for a pose it
# refuses reports "max: 1016.72". The UR5e's 850 mm figure is to the wrist centre.
REACH_MM = 1016.7
# The flange-to-cup standoff, which is also the gripper's configured offset.
TOOL_MM = 120.0
# How far below the part's centre the cup sits on a flat side: low enough to be on the
# flat panel rather than the curved crown.
GRIP_DROP_MM = 25.0
# Keep stations this far inside the reach bound. Solutions near full extension are where
# a spherical bound stops being honest, because the orientation may not be achievable.
REACH_MARGIN = 0.10

Vec3 = Tuple[float, float, float]
Pose = Tuple[Vec3, Dict[str, float]]

# Viam orientation vectors for a cup pointing along each world axis.
TOOL_MINUS_Y = {"o_x": 0.0, "o_y": -1.0, "o_z": 0.0, "theta": 0.0}
TOOL_PLUS_Y = {"o_x": 0.0, "o_y": 1.0, "o_z": 0.0, "theta": 0.0}
TOOL_DOWN = {"o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 0.0}


@dataclass(frozen=True)
class Box:
    centre_mm: Vec3
    dims_mm: Vec3

    @property
    def top_mm(self) -> float:
        return self.centre_mm[2] + self.dims_mm[2] / 2.0


@dataclass(frozen=True)
class Cell:
    """The cell as the fragment describes it, in millimetres."""

    arm_bases_mm: Dict[str, Vec3]
    part: Box
    belt: Box
    trays: Dict[str, Box]
    handoff: Box
    camera_mm: Vec3
    camera_target_mm: Vec3

    @classmethod
    def from_fragment(cls, path: Path = FRAGMENT) -> "Cell":
        config = json.loads(Path(path).read_text())
        components = config["components"]
        world = next(c for c in components if c["model"].endswith(":world"))
        props = {p["name"]: p for p in world["attributes"]["props"]}

        def box(name: str) -> Box:
            prop = props[name]
            size = float(prop.get("size", 1.0))
            scale = [float(v) for v in (prop.get("scale") or (1.0, 1.0, 1.0))]
            return Box(
                centre_mm=tuple(float(v) * 1000.0 for v in prop["position"]),
                dims_mm=tuple(size * scale[i] * 1000.0 for i in range(3)),
            )

        bases = {}
        for component in components:
            if component.get("type") == "arm":
                translation = component["frame"]["translation"]
                bases[component["name"]] = (
                    float(translation["x"]), float(translation["y"]),
                    float(translation["z"]),
                )
        camera = next(c for c in components if c["name"] == "inspect-cam")
        translation = camera["frame"]["translation"]
        return cls(
            arm_bases_mm=bases,
            part=box("part"),
            belt=box("belt"),
            trays={"good": box("good_tray"), "bad": box("bad_tray")},
            handoff=box("handoff_stand"),
            camera_mm=(float(translation["x"]), float(translation["y"]),
                       float(translation["z"])),
            camera_target_mm=tuple(
                float(v) * 1000.0 for v in camera["attributes"]["target"]),
        )

    # ---- grips ---------------------------------------------------------------

    def side_grip(self, centre_mm: Vec3, side: str) -> Pose:
        """Flange pose that puts the cup on one flat side of the part.

        `side` is the face the cup covers, named by the world axis its outward normal
        points along. The arm stands off along that normal by half the part plus the
        tool, and the cup points back at the part.
        """
        half = self.part.dims_mm[1] / 2.0
        if side == "+y":
            position = (centre_mm[0], centre_mm[1] + half + TOOL_MM,
                        centre_mm[2] - GRIP_DROP_MM)
            return position, TOOL_MINUS_Y
        if side == "-y":
            position = (centre_mm[0], centre_mm[1] - half - TOOL_MM,
                        centre_mm[2] - GRIP_DROP_MM)
            return position, TOOL_PLUS_Y
        raise ValueError(f"unknown grip side {side!r}; the flat sides are +y and -y")

    def pick(self, side: str = "+y") -> Pose:
        return self.side_grip(self.part.centre_mm, side)

    # Let go a few millimetres up rather than pressing the part onto the surface.
    # Driving a rigid part onto a rigid surface is contact the arm cannot push through,
    # so it stalls short of its commanded pose and the move fails - with the part
    # already where it needs to be.
    RELEASE_GAP_MM = 8.0

    def resting_on(self, surface: Box, gap_mm: float = 0.0) -> Vec3:
        """Where the part's centre sits when it rests on a surface."""
        return (surface.centre_mm[0], surface.centre_mm[1],
                surface.top_mm + self.part.dims_mm[2] / 2.0 + gap_mm)

    def handoff_grip(self, side: str, gap_mm: float = 0.0) -> Pose:
        """Flange pose for the part sitting on the handoff stand, held by `side`."""
        return self.side_grip(self.resting_on(self.handoff, gap_mm), side)

    def place(self, tray: str, side: str, gap_mm: float = 0.0) -> Pose:
        """Flange pose to set the part down on a tray, held by `side`."""
        return self.side_grip(self.resting_on(self.trays[tray], gap_mm), side)

    # ---- constraints ---------------------------------------------------------

    def reach_mm(self, arm: str, flange_mm: Sequence[float]) -> float:
        return math.dist(tuple(flange_mm), self.arm_bases_mm[arm])

    def grip_clears_belt(self, side: str) -> bool:
        """Is there room to stand off on this side without being over the belt?

        The belt runs away in -y from the part, so approaching that side puts the arm
        above it with almost no clearance and the planner refuses. This is a property of
        the layout, not of the gripper, and it is what makes the +y grip the only one
        the pick can use.
        """
        (_, y, _), _ = self.pick(side)
        belt_near_edge = self.belt.centre_mm[1] + self.belt.dims_mm[1] / 2.0
        return y > belt_near_edge


    # How far the part sits toward the presenting arm's own side. An arm holding a flat
    # side stands off 175 mm beyond the part, so where the part sits decides where the
    # flange has to be. Measured: arm-a gets zero IK solutions with its flange at y=+175
    # and reaches y=+60 only in configurations it then cannot roll out of - the wrist
    # ends up 17 degrees short with the part pressed against the arm. At 190 the flange
    # lands near y=0, which it handles comfortably. The camera was pulled back far
    # enough to frame both arms' presentation points.
    PRESENT_OFFSET_MM = 190.0

    def inspect_point_mm(self, side: str = "+y") -> Vec3:
        """Where the part sits to be inspected, for an arm holding `side`."""
        target = self.camera_target_mm
        toward = -self.PRESENT_OFFSET_MM if side == "+y" else self.PRESENT_OFFSET_MM
        return (target[0], target[1] + toward, target[2])

    def present_poses(self, side: str) -> List[Tuple[str, Pose]]:
        """Flange poses that show the camera each face the cup is not covering.

        Rolling about the tool axis shows four faces - the two ends and the crown and
        base - but never the free flat side, which stays edge-on to the camera however
        far it rolls. That one needs the part yawed a quarter turn so it faces the
        camera, and yawed the way that leaves the ARM behind the part rather than
        between it and the lens.
        """
        centre = self.inspect_point_mm(side)
        half = self.part.dims_mm[1] / 2.0
        standoff = half + TOOL_MM
        sign = 1.0 if side == "+y" else -1.0
        covered = side
        free = "-y" if side == "+y" else "+y"

        poses: List[Tuple[str, Pose]] = []
        # Rolling about the tool axis: the tool stays along -+y, the part spins.
        rolled = {0.0: "+x", 90.0: "+z", 180.0: "-x", 270.0: "-z"}
        for theta, face in rolled.items():
            position = (centre[0], centre[1] + sign * standoff,
                        centre[2] - GRIP_DROP_MM)
            orientation = {"o_x": 0.0, "o_y": -sign, "o_z": 0.0, "theta": theta}
            poses.append((face, (position, orientation)))

        # A quarter turn about z puts the free side towards the camera. The camera looks
        # back along -x, so the arm must stand off on -x for the part to be in front of
        # it rather than the other way round.
        poses.append((free, ((centre[0] - standoff, centre[1],
                              centre[2] - GRIP_DROP_MM),
                             {"o_x": 1.0, "o_y": 0.0, "o_z": 0.0, "theta": 0.0})))
        assert covered not in [face for face, _ in poses]
        return poses


def reach_budget() -> float:
    """The distance a station may sit from its arm's base."""
    return REACH_MM * (1.0 - REACH_MARGIN)
