"""Narrowing the joint ranges served to the motion service.

The planner picks any IK solution inside the range the kinematics declares, with no
preference for the one nearest where the arm already is. The UR SVA declares +-360
degrees, so a cell winds itself up over a few dozen moves until a joint approaches the
limit and the arm jams. `joint_limit_deg` narrows what the planner is told.

What must hold: revolute joints get clamped, ranges already tighter are left alone,
nothing is touched without the attribute, and a urdf is passed through rather than
mangled - the rewrite only understands SVA json.
"""

import json

import pytest
from viam.components.arm import KinematicsFileFormat

from isaac_module.models.arm import IsaacArm

SVA = json.dumps({
    "name": "ur5e",
    "kinematic_param_type": "SVA",
    "links": [{"id": "base_link", "parent": "world"}],
    "joints": [
        {"id": "shoulder_pan_joint", "type": "revolute", "parent": "base_link",
         "axis": {"x": 0, "y": 0, "z": 1}, "max": 360, "min": -360},
        {"id": "shoulder_lift_joint", "type": "revolute", "parent": "shoulder_link",
         "axis": {"x": 0, "y": -1, "z": 0}, "max": 360, "min": -360},
        {"id": "elbow_joint", "type": "revolute", "parent": "upper_arm_link",
         "axis": {"x": 0, "y": -1, "z": 0}, "max": 180, "min": -180},
        {"id": "rail", "type": "prismatic", "parent": "base_link",
         "axis": {"x": 1, "y": 0, "z": 0}, "max": 2000, "min": 0},
    ],
}).encode()

SVA_FMT = KinematicsFileFormat.KINEMATICS_FILE_FORMAT_SVA
URDF_FMT = KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF


def arm_with(**attrs):
    arm = IsaacArm("test-arm")
    arm._attrs = dict(attrs)
    return arm


def joints_of(data):
    return {j["id"]: j for j in json.loads(data)["joints"]}


def test_no_attribute_leaves_the_file_alone():
    assert arm_with()._clamp_joint_limits(SVA_FMT, SVA) == SVA


def test_wide_joint_is_narrowed():
    out = joints_of(arm_with(joint_limit_deg=180)._clamp_joint_limits(SVA_FMT, SVA))
    assert out["shoulder_pan_joint"]["min"] == -180
    assert out["shoulder_pan_joint"]["max"] == 180


def test_already_tight_joint_is_untouched():
    """Clamping must narrow, never widen - the file's own limits are the robot's."""
    out = joints_of(arm_with(joint_limit_deg=360)._clamp_joint_limits(SVA_FMT, SVA))
    assert out["elbow_joint"]["min"] == -180
    assert out["elbow_joint"]["max"] == 180


def test_prismatic_joints_are_not_touched():
    """A rail's limits are millimetres; clamping them to a degree count is nonsense."""
    out = joints_of(arm_with(joint_limit_deg=180)._clamp_joint_limits(SVA_FMT, SVA))
    assert out["rail"]["min"] == 0
    assert out["rail"]["max"] == 2000


def test_a_negative_limit_is_read_as_a_magnitude():
    out = joints_of(arm_with(joint_limit_deg=-180)._clamp_joint_limits(SVA_FMT, SVA))
    assert out["shoulder_pan_joint"]["min"] == -180


def test_urdf_is_passed_through_untouched():
    """The rewrite only understands SVA json; a urdf must survive unchanged."""
    urdf = b"<robot name='x'><joint name='j' type='revolute'/></robot>"
    assert arm_with(joint_limit_deg=180)._clamp_joint_limits(URDF_FMT, urdf) == urdf


def test_the_cell_asks_for_a_limit():
    """The whole point is that the cell config actually turns this on."""
    from pathlib import Path
    fragment = json.loads(
        (Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text()
    )
    arms = [c for c in fragment["components"] if c["type"] == "arm"]
    assert arms, "no arms in the fragment"
    for arm in arms:
        assert arm["attributes"].get("joint_limit_deg") == 180, (
            f"{arm['name']} does not narrow its joint limits; the cell will wind up"
        )


def test_per_joint_override_can_be_asymmetric():
    """A symmetric limit puts the wrap point on every joint at the same place.

    Bounding this cell's arms at +-180 moved the ratchet from pan to shoulder-lift,
    which then sat pinned at 180. An explicit [min, max] still spans one turn - so
    there is exactly one solution per pose - but puts both ends where the arm never
    travels.
    """
    out = joints_of(arm_with(
        joint_limit_deg=180,
        joint_limits_deg={"shoulder_lift_joint": [-270, 90]},
    )._clamp_joint_limits(SVA_FMT, SVA))
    assert out["shoulder_lift_joint"]["min"] == -270
    assert out["shoulder_lift_joint"]["max"] == 90
    # the scalar still governs the joints the map does not name
    assert out["shoulder_pan_joint"]["min"] == -180


def test_per_joint_override_alone_works_without_the_scalar():
    out = joints_of(arm_with(
        joint_limits_deg={"shoulder_lift_joint": [-270, 90]},
    )._clamp_joint_limits(SVA_FMT, SVA))
    assert out["shoulder_lift_joint"]["max"] == 90
    assert out["shoulder_pan_joint"]["min"] == -360, "untouched without a scalar limit"


def test_the_cell_pins_the_lift_joint():
    from pathlib import Path
    fragment = json.loads(
        (Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text()
    )
    for arm in [c for c in fragment["components"] if c["type"] == "arm"]:
        limits = arm["attributes"].get("joint_limits_deg", {})
        assert limits.get("shoulder_lift_joint") == [-270, 90], (
            f"{arm['name']} leaves shoulder_lift on the symmetric limit, where it pins"
        )
