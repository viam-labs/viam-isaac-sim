# Finishing the QC cell

What "finished" means here: a round runs end to end without a human in it — a mailbox
arrives on the belt, both arms inspect it, and it lands in the tray its defects say it
belongs in — and a test says so rather than a person watching a video.

## What a round is, precisely

1. **Pick.** arm-a takes the mailbox off the belt by a **flat side**, approaching from +y.
   The −y side is over the belt and the planner refuses it; that constraint is the
   layout's, not the gripper's.
2. **First check.** arm-a presents the faces it is not covering to `inspect-cam`: crown,
   base, door end, back end, and the free side. Five of six.
3. **Handoff, by set-down.** arm-a places the mailbox on a small fixture near y=0 and
   retreats; arm-b then picks it by the **opposite** flat side. The part has to change
   hands at all because arm-a **cannot reach the trays** by construction.

   An earlier draft had arm-b take it mid-air, and said in one line that arm-b takes "the
   side arm-a was holding" and in the next that arm-b presents "the face arm-a's cup was
   covering". Those cannot both be true, and the first makes the second check pointless.
   It is the *opposite* side.

   Mid-air is also the wrong mechanism. Each cup's attachment joint locks all six degrees
   of freedom, so while both cups are closed on one rigid body any drive mismatch between
   the arms is reported by PhysX as joint load — and load is what breaks the shear limit
   and drops parts. A set-down handoff has no moment where two welds fight, and no moment
   where both arms need to plan into the same space.

4. **Second check.** arm-b presents the face arm-a's cup was covering — the +y panel. This
   is the whole reason there are two arms and two checks.
5. **Place.** arm-b puts it in the good or bad tray according to the verdict.

The learner writes step 2 and 4's *judgement*. Everything else is the cell's job.

## Foundations to fix first

**F1. Station poses must be derived, not stored.** `probes/stations.json` is stale in three
independent ways: it encodes a tool-down grip (we grip the side now), an 80 mm cube (the
part is a 175×110×112 mailbox), and a cell with no scenery (there are 18 props). `arm-a
infeed` already fails IK because of it. It is the hand-copied-constants problem the earlier
review named, and the fix is the one it named: **one module that computes station poses
from the cell's geometry, which the fragment, the probes and the round all import**, with a
test that asserts the reach window and the handoff constraint still hold. Nothing else on
this list is trustworthy until this exists.

**F2. A visual gripper — done.** Landed before this review; visual-only, oriented along
the cup axis.

Originally specified as: The cup exists to the planner (a capsule in `world_state`) and to
physics (the attachment rig) but has no geometry, so the arm appears to pick things up with
nothing on the end of it. Visual-only prims under the flange: a body cylinder and a cup.
**Visual only** — collision is already handled, and a second collider on the tool would
fight the one the planner knows about.

## The round

**R1. Defects the cell can vary.** The part carries its possible defects as composite
children named `defect_*`; a world verb toggles each one's **visibility** at runtime, so a
round is repeatable with no respawn and no restart.

**At least one defect must sit on the +y panel** — the face arm-a's cup covers. Without it
the second check can never change a verdict, the two-arm story has no teeth, and a test
for "defective goes to the bad tray" passes on arm-a's check alone.

**Defect children must not collide.** Visibility in USD is a render attribute; PhysX keeps
the collider either way. An invisible "proud door" still protrudes, an invisible bent flag
still sits in arm-b's approach, and the suction raycast still hits both. `children` needs a
`collision: false` key, honoured where the collider is applied. Structural children still
collide — that distinction needs stating, because `qc-cell-plan.md` currently says
children carry no collision at all, and the code applies it to every one.

A dent also cannot be modelled by adding a child, which is a bump. Use a dark patch or a
small ding.

**R2. Ground truth, occlusion-aware.** Reading back the visibility flag the verb just wrote
is tautological: it says a defect exists, not that the camera could see it at any
presentation. Replicator's `bounding_box_2d_tight` annotator on `inspect-cam`, with
semantics on the defect children, answers the question actually being asked — and the same
mechanism answers "was every face presented". It needs no camera intrinsics, which also
retires that open finding. Set `extent` on every child or the tight boxes inflate.

A toggle followed immediately by a frame grab can return the pre-toggle render, so the verb
must not return until a render step has run.

**R3. The round driver.** `probes/round.py` — pick, present, handoff, present, place —
built on the motion service with the scenery and the held part in `world_state` throughout.
It stays a probe for now rather than becoming `qc:cell` verbs: the verbs are the course's
API and belong in the course module, and extracting them once the sequence is proven is
mechanical. Building them first would mean debugging the cell through an API that is also
being designed.

## Tests

The tests have to run the real thing. A round that passes in mock proves the wiring, not
the cell.

**T1. `test_cell_layout.py`** (no machine), written *with* F1 rather than after the round.
Every station inside the arm's reach with margin; the grip side clear of the belt; and the
handoff constraint asserted **at the flange, for every grip side the cell admits** — not at
the tray centre, which passes trivially. That distinction is not academic: with the cup on
the −y panel the good tray is about 940 mm from arm-a and therefore *reachable*. Only the
belt blocking the −y approach at the pick keeps the handoff mandatory, and the test has to
say which grip the layout relies on.

**T2. `test_round_e2e.py`** (needs the machine). One test per claim, so a failure names
itself:

* the part leaves the belt and stays held through the first presentation;
* the handoff transfers it — asserted on the **simulator**, not the grippers: after the
  set-down the part's pose tracks arm-b's flange and is independent of arm-a's;
* every face is presented to the camera at least once across the two checks, measured by
  the annotator rather than by face-normal arithmetic, which cannot see occlusion by the
  tool or the arm;
* the part ends **upright and at rest** inside the tray the verdict names — the trays are
  solid blocks with no walls, so a part knocked on its side is still "inside" in xy;
  orientation and two reads half a second apart are what make the claim mean something;
* a defective part ends in the bad tray and a clean one in the good tray — the same
  round run twice with different defects, because a cell that always says "bad" passes
  half of any weaker test.

**T3. Assert against the simulator, not against the cell's own claims.** Every check reads
`prop_poses`, not `is_holding_something`. The gripper reported a grip it did not have for
most of a day; a test that trusts it would have passed throughout.

## How this gets verified

* `probes/record_cell.py` after each milestone — the video has found what probes missed
  every time: floating arms, the cup on a curved crown, a part released in mid-air.
* A fable review of this plan, of the tests once written, and of the results once green.
* `probes/grasp_reliability.py` unchanged as the regression guard on the grip.

## Order

**F1 → R3 → T2 is the critical path.** T1 is F1's own test and is written with it. R1 and
R2 touch none of the motion work and can proceed alongside. F2 is done.

F1 also owns the camera: `fov_deg` and `target` were derived for an 80 mm cube, and the
mailbox is 175 mm long against a 159 mm frame, so presentations overflow and the door and
flag ends can fall out of shot.

R3 is a package (`probes/qc/`), not another monolithic script: `stations.py` from F1 and a
`round.py` exposing `pick`, `present`, `handoff`, `place` that take their clients as
arguments, so the tests import it and the later extraction into `qc:cell` verbs really is
mechanical. `record_cell.py` is closures over module-level clients and nothing in it can be
imported.

Delete `.scratch/stations.json` when F1 lands — it is a byte-identical copy of
`probes/stations.json`, the hand-copied-constants problem committed twice.

## Known risks

| risk | what it looks like | |
|---|---|---|
| side grip leaves too few presentable faces | the two checks cannot cover six faces between them | count the faces in T2 rather than assume |
| handoff needs both arms in the same space | planner refuses, or they collide | it is the one moment both arms are close; expect to tune the handoff pose |
| defect marks too small for the camera | the oracle sees them, a detector never could | measure mark pixels at the model input as the old cell did |
| trays are 23 mm apart | a place into one clips the other | still unresolved from the earlier review |
| the part's planner box omits the flag | the flag reaches 30 mm past the declared half-width on the −y side — the side arm-b grips — and the carried-part transform inherits the same box | report the true bounds, or a box per child |
