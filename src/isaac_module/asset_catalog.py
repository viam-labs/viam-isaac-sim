"""Device facts the gripper code reads.

Vendored from DTCurrie/viam-isaac-sim, trimmed to what surface_gripper.py imports,
so the two trees share one copy. The epick entry is a real device spec and its
numbers are its manual's; surface_gripper only reads the cup radius, the vacuum
figures and the cup names, and only as default arguments.
"""

from __future__ import annotations

from typing import Any


EPICK: dict[str, Any] = {
    "kind": "gripper",
    "tcp_offset_m": 0.196,
    # gripper mass including the coupling, manual section 6.2
    "mass_kg": 0.706,
    "body": {
        "radius_mm": 35.5,
        # drawn 129 mm long: its rear boss reaches 3 mm past the flange, into
        # the arm's own end-effector space. Collision stops at the flange plane
        "visual_length_mm": 129.0,
        "visual_center_z_mm": -134.5,
        "collision_mm": (71.0, 71.0, 126.0),
        "collision_center_z_mm": -133.0,
    },
    "plate": {
        "size_mm": (204.5, 126.3, 3.2),
        "center_z_mm": -68.4,
    },
    "cups": {
        "radius_mm": 24.5,
        # a 159.5 x 81.3 mm rectangular pattern, one cup per quadrant, named
        # as epick_model.json names its links
        "names": ("cup-xp-yp", "cup-xp-yn", "cup-xn-yp", "cup-xn-yn"),
        "offsets_mm": ((79.75, 40.65), (79.75, -40.65), (-79.75, 40.65), (-79.75, -40.65)),
        "visual_length_mm": 60.0,
        "visual_center_z_mm": -40.0,
        "tip_z_mm": -10.0,
        "collision_mm": (49.0, 49.0, 44.0),
        "collision_center_z_mm": -48.0,
        "tcp_clearance_z_mm": -26.0,
    },
    # "exceeding 4.5 kg per air node could induce damage", manual section 6.2.
    # The number Robotiq stands behind, used instead of the area formula since
    # the CAD's 49 mm cup matches neither the 40 nor the 55 mm cup it rates
    "payload_per_cup_kg": 4.5,
    # the holding force is the cup's inside area times the vacuum, manual
    # section 6.2.1, with 1 % of vacuum worth 1.013 kPa and 80 % the maximum
    # the gripper regulates to (section 6.2). At 80 % a 49 mm cup develops
    # 153 N, and that is the load that breaks an attachment in the sim
    "max_vacuum_pct": 80.0,
    "kpa_per_vacuum_pct": 1.013,
    # gripping and release times for one 40 mm cup, manual section 6.2
    "grip_time_ms": 150,
    "release_time_ms": 180,
}
# the prim name the EPick's body is authored at, under the arm link it rides

CUP_APPROACH_GAP_MM = 5.0
