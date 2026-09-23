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

## Now built on the shared implementation

The authoring is no longer ours. `surface_gripper.py` is vendored verbatim from
DTCurrie/viam-isaac-sim, with `compat.import_surface_gripper` and the `asset_catalog`
constants it reads, so the two trees carry one implementation rather than two to reconcile
when they merge. What stays ours is `models/gripper.py` - the Viam API surface - and the
translation from this module's config into the frame that module expects.

That translation is the subtle part and is worth stating: it wants a tool frame whose **+Z
is the cup axis**, while our `offset` is in the parent prim's frame, where Isaac's UR
flange puts the tool along **+x**. The boot log prints the resolved axis so a
misconfiguration is visible without a grasp test.

Taking it also brought three things ours lacked, each of which reads as intermittency:

* **`isaac:forwardAxis`.** The plugin raycasts along the joint's forward axis to find
  something to grip. We never set it, so the ray fired wherever the joint frame happened
  to point.
* **The coaxial check turned off.** The plugin compares a *single physics step's* force to
  its limit with no averaging window, and a linear move is waypoints a couple of
  millimetres apart, each a step into stiff drives. A real limit fires on those spikes and
  drops a part that is not slipping. Authored as 0; shear still guards.
* **A world-level scope.** The rig must not sit inside the articulation - a rigid body
  nested under a link is an error - and ours was authored under `wrist_3_link`.

## Open: reliable when measured, unreliable when filmed

`probes/grasp_reliability.py` picks the carton five times out of five, and the gripper's
claim agrees with the simulator every time - including through a lateral carry, which is
where a suction cup actually lets go. So the mechanism works and `is_holding_something` is
not lying on its own account.

`probes/record_cell.py` runs the same sequence and drops the carton every time.

Ruled out so far: **the gripper implementation** - replacing ours with the shared rig kept
the probe at 5/5 and left the filmed run still dropping. And **capture rate**. The frame grabber originally pulled flat out, which
competes with stepping physics on the sim thread; pacing it to 12 fps changed nothing.

Still to try, cheapest first: film the reliability probe itself rather than the recorder's
sequence, to find which of the two differs; check whether the retreat of the idle arm
before the pick perturbs the part; and log the gripper's status and the part pose *during*
the recorded run rather than inferring both from frames afterwards.

The important part is that this is a difference between two of our own scripts, not
evidence against the gripper. A round that picks and carries is measurably fine; something
about the filmed sequence is not.

## Still worth taking from that fork

Cup compliance. They model the bellows as a soft limit on the forward axis rather than a
weld, and note that with several cups only one should carry shear - four cups locking the
same three freedoms fight each other, and PhysX reports that fight as load, which the
plugin reads as a grip about to fail. Single-cup suction here does not need it yet.
