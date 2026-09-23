# Suction gripper: what works, and the one thing that does not

`erh:isaac-sim:gripper` is implemented end to end and wired into the cell, but **it does
not yet physically hold a part**. Recording exactly where it stands so the next attempt
starts from evidence rather than from scratch.

## Works, verified against live Isaac

* The component builds on both arms and authors its prims: a D6 attachment joint and a
  surface-gripper prim.
* `open`, `grab`, `stop`, `is_moving`, `is_holding_something` and a `status` DoCommand all
  behave consistently. `status` reads `Open` before a close and `Closed` after.
* The cup is reported through `GetGeometries` so the planner can see it.
* 8 unit tests cover the mock backend, including that a grab on nothing reports `False`.

## Two real bugs found and fixed on the way

**The joint must anchor to a rigid body.** It was anchored to the arm's flange, which is a
plain Xform; the rigid body is `wrist_3_link`. A joint parented to a non-body is silently
inert. The module now walks up to the nearest `RigidBodyAPI` ancestor and re-expresses the
cup offset in that body's frame, so callers can still name the frame they think in. The log
says when it does this.

**The tool axis differs between the two frames.** Isaac's flange prim puts the tool along
its local **+x** (the URDF `ee_link` convention) — measured by commanding the end-effector
frame tool-down and reading which local axis points at the floor. Viam's SVA puts the same
tool along the end-effector frame's **+z**. Same cup, two frames, two axes; an offset in
the wrong one points the suction sideways and every grasp silently misses.

**Status codes are numbers, not words.** `get_surface_gripper_status()` returns `"0"` and
`"2"`, not the `"Open"/"Closing"/"Closed"` its docstring advertises. Both spellings are now
mapped, so a build that starts returning words will not read as permanently open.

## The remaining failure, precisely

With the arm's flange a tool-length above the part's top face:

```
  at contact: flange z=  807.9  part z=  648.0  holding=False
  grab() -> True
  after lift: flange z= 1058.0  part z=  648.0  holding=True
  flange rose 250.1 mm; part rose 0.0 mm
```

The gripper closes, reports `Closed`, and lists `/World/part` as gripped — and the part
does not move. So the surface gripper believes it has a grip that does not constrain
anything.

Ruled out along the way:

* **Not a stale reading.** `prop_poses` now reads the rigid-body (physics) view rather
  than the USD xform; the answer is unchanged, so the part really is stationary.
* **Not the anchoring.** Confirmed in the log that the joint is now on `wrist_3_link`.
* **Not a late-parse problem**, or at least not one a reset fixes: forcing
  `world.reset()` after authoring the joint changed nothing, and it snaps every prop back
  to spawn, so it was removed.

## Where to look next

1. **The joint may not qualify as D6.** Isaac's example says "Joint Type should be D6";
   this authors a generic `UsdPhysics.Joint` with no DOF limits or drives. A real D6 with
   its axes configured is the most likely fix.
2. **`set_write_to_usd(True)`** — isaac's own example calls this on the surface-gripper
   interface before use, and the module does not.
3. **Authoring time.** These joints are created at component construction, well after the
   articulation exists. Authoring them when the arm is spawned may matter.
