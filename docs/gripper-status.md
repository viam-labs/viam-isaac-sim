# Suction gripper: working

`erh:isaac-sim:gripper` picks up a part and carries it. Verified against live Isaac:

```
  at contact: flange z=  807.9  part z=  648.0  holding=False
  grab() -> True
  after lift: flange z= 1058.0  part z=  898.1  holding=True
  flange rose 250.1 mm; part rose 250.1 mm
```

## What it took, in the order the bugs were found

**The joint must anchor to a rigid body.** It was on the arm's flange, which is a plain
Xform; the body is `wrist_3_link`. A joint parented to a non-body is silently inert. The
module walks up to the nearest `RigidBodyAPI` ancestor and re-expresses the cup offset in
that body's frame, so callers can still name the frame they think in.

**The tool axis differs between the two frames that describe it.** Isaac's flange prim
puts the tool along its local **+x** (the URDF `ee_link` convention); Viam's SVA puts the
same tool along the end-effector frame's **+z**. Measured by commanding the end-effector
tool-down and reading which local axis pointed at the floor. An offset in the wrong frame
points the suction sideways and every grasp silently misses.

**Status codes are numbers, not words.** `get_surface_gripper_status()` returns `"0"` and
`"2"`, not the `"Open"/"Closing"/"Closed"` its docstring advertises, so `is_moving` and
`stop` were comparing against strings that never occur.

**A D6 with no limits constrains nothing** — and this is what made a grasp look like it
worked while holding nothing. All six degrees of freedom were free, so the fabricated
joint held the part in name only: `Closed`, `/World/part` listed as gripped, and the part
sitting still while the arm lifted 250 mm away from it. USD spells a locked axis as a
limit whose low sits above its high.

**The joint must carry the attachment-point API.** Pointing the gripper's relationship at
a joint is not enough; the joint itself needs `ApplyAttachmentPointAPI` or the plugin does
not treat it as somewhere suction can act.

The last two came from reading DTCurrie/viam-isaac-sim, whose vacuum implementation had
both. Credit there; the diagnosis of the first three was ours.

## Still worth taking from that fork

Cup compliance. They model the bellows as a soft limit on the forward axis rather than a
weld, and note that with several cups only one should carry shear - four cups locking the
same three freedoms fight each other, and PhysX reports that fight as load, which the
plugin reads as a grip about to fail. Single-cup suction here does not need it yet.
