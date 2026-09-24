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
3. **Handoff.** arm-b takes the mailbox by the side arm-a was holding, and arm-a releases.
   The handoff is not choreography — arm-a **cannot reach the trays**, by construction, so
   the part has to change hands to be placed at all.
4. **Second check.** arm-b presents the sixth face, the one arm-a's cup was covering. This
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

**F2. A visual gripper.** The cup exists to the planner (a capsule in `world_state`) and to
physics (the attachment rig) but has no geometry, so the arm appears to pick things up with
nothing on the end of it. Visual-only prims under the flange: a body cylinder and a cup.
**Visual only** — collision is already handled, and a second collider on the tool would
fight the one the planner knows about.

## The round

**R1. Defects the cell can vary.** The part carries its possible defects as composite
children named `defect_*` — a dent in the crown, a proud door, a bent flag. All are
authored at spawn; a world verb toggles each one's **visibility** at runtime. Ground truth
is then "which defect children are visible", with no respawn and no restart, which is what
makes a round repeatable.

This is deliberately not `spawn_prop`. Runtime spawning is still worth having, but
visibility is enough for defects and costs a fraction as much.

**R2. Ground truth.** A world verb reports the visible defects. The oracle reads it and
returns a verdict. That is the reference implementation the cell is tested against, and
the thing a learner's own perception is scored against later.

**R3. The round driver.** `probes/round.py` — pick, present, handoff, present, place —
built on the motion service with the scenery and the held part in `world_state` throughout.
It stays a probe for now rather than becoming `qc:cell` verbs: the verbs are the course's
API and belong in the course module, and extracting them once the sequence is proven is
mechanical. Building them first would mean debugging the cell through an API that is also
being designed.

## Tests

The tests have to run the real thing. A round that passes in mock proves the wiring, not
the cell.

**T1. `test_cell_layout.py`** (no machine). Every station inside the arm's reach with
margin; both trays outside arm-a's reach; the grip side clear of the belt. Pure arithmetic
over F1's module, so it runs in CI and fails the moment someone moves a tray.

**T2. `test_round_e2e.py`** (needs the machine). One test per claim, so a failure names
itself:

* the part leaves the belt and stays held through the first presentation;
* the handoff transfers it — arm-b holds it *and* arm-a does not, checked on both;
* every face is presented to the camera at least once across the two checks;
* the part ends inside the tray the oracle's verdict names, and at rest;
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

F1 → F2 → R1 → R2 → R3 → T1 → T2, then iterate to green. F1 first because every pose in
the round depends on it, and F2 early because it is cheap and every video from then on
shows the real tool.

## Known risks

| risk | what it looks like | |
|---|---|---|
| side grip leaves too few presentable faces | the two checks cannot cover six faces between them | count the faces in T2 rather than assume |
| handoff needs both arms in the same space | planner refuses, or they collide | it is the one moment both arms are close; expect to tune the handoff pose |
| defect marks too small for the camera | the oracle sees them, a detector never could | measure mark pixels at the model input as the old cell did |
| trays are 23 mm apart | a place into one clips the other | still unresolved from the earlier review |
