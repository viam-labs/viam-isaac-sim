# Joint wind-up: diagnosis and fix

Both open problems from the obstacle work — `place_bad` failing to plan in sequence but
not in isolation, and arms stalling on moves to empty poses — are the same bug.

## Diagnosis

The UR5e SVA gives every revolute joint a range of **±360°**, and the cell winds itself up
inside it:

| | arm-a max abs joint | arm-b max abs joint |
|---|---|---|
| after `world reset` | 0° | 0° |
| after one pass over the stations | 97° | 230° |
| after two passes | **270°** | 230° |

Arm-a's joint 0 went −26.4° → 206.4° in a single pass.

**Why, corrected.** The first version of this document blamed a planner with "no reason to
prefer the solution nearest the current pose". That is wrong, and worth recording because
it is the opposite of the truth: RDK seeds each IK attempt from the start configuration
and scores candidates by joint-space distance from it. The planner is *greedy for the
nearest solution*.

The real driver is where the workspace sits. The UR5e's zero configuration points the
flange at roughly −164° in its base frame while every station in this cell is at +x, so
the stations all need a pan within about 30° of ±180 — the wrap point. Greedy
nearest-neighbour across a discontinuity ratchets: each move picks the closest
representative, which may be the one a turn away, and the choice sticks. That also
explains a detail pure accumulation does not — arm-b reached 230° and then *stayed* there
across a second pass rather than climbing to 360. It had settled into a cycle.

Two consequences worth keeping in mind. The retreat pose the station check used sits
directly behind the base, right on the wrap point, so the probe contributed to the
ratchet it measured. And the structurally better fix is to rotate both arm frames 180° so
the stations sit near pan 0 and the discontinuity falls behind the arm, where nothing
works — see "Not done here".

Once a joint approaches 360° the cell degrades in three separate-looking ways:

* **planning fails** — the planner must unwind to reach the goal, and often cannot;
* **execution stalls** — the arm fails to settle within the 0.5° final tolerance and times
  out after `move_timeout` (30 s). See the caveat below: most of these turned out not to
  be wind-up at all;
* **the move is rejected outright** — `joint 0 needs to be within range [-360, 360] and
  cannot be moved to 363.6`.

That is why the failures looked intermittent and state-dependent, and why every isolated
retest passed: a fresh boot starts at zero, and the symptom needs a few dozen moves to
appear. It also means the cell would jam within a round or two of a real course — a round
issues around fifty moves.

Nothing here is specific to the QC cell. Any two-arm scene driven through the motion
service on this module will wind up the same way.

### The stalls were mostly the floor, not wind-up

A stall showing `j1: at 232.0 want 230.4` is a 1.6° residual on one joint that never
closes. That is a force balance, not a contorted pose: the arm is pressing on something.
And it was — `boot()` calls `add_default_ground_plane()` for any scene without its own
stage, so there is a collider at z=0 that `prop_obstacles()` never reported. j1 = 232°
(≡ −128°) points the upper arm steeply down from a base only 391 mm up. The planner,
shown a world with no floor, routed the elbow into the ground and the arm stalled leaning
on it.

Settling itself is fine: on a freshly reset arm, gravity-loaded joint moves settle inside
the 0.5° tolerance in 0.4 s. So the tolerance is not the problem and did not need changing.

## Fix

### 1. Stop the planner choosing multi-turn solutions (the cause)

Add a `joint_limit_deg` attribute to the arm. When set, the module clamps every revolute
joint's `min`/`max` in the SVA it serves from `GetKinematics` before handing it to the
motion service.

`joint_limit_deg: 180` costs nothing in reachable poses — ±180° of any revolute joint
already covers every orientation — and removes exactly the freedom being abused. The cell
sets it; the default stays unclamped so nobody else's behaviour changes.

This is the right layer: the limits travel with the kinematics, so every consumer of the
motion service gets them without having to remember anything.

Only applies to SVA JSON. A URDF `kinematics_url` is passed through untouched, and the
attribute is documented as such rather than silently doing nothing.

### 1b. Tell the planner about the floor

`prop_obstacles()` now reports the ground plane as a fixed slab whose top face is z=0,
alongside the props. Without it the planner will keep routing elbows through the floor
however the joint limits are set.

### 2. Give the cell a way back (the safety net)

Limits stop drift accumulating past 180°, but an arm can still be left somewhere awkward
by a failed move. `qc:cell` sends both arms to a known joint configuration between rounds
with `move_to_joint_positions`. No module change: the verb already exists, and a round
boundary is the natural place for it.

`world reset` also zeroes the arms, but it snaps every prop back to its spawn state too,
so it is a between-runs tool, not a between-rounds one.

### 3. Stop a skipped waypoint corrupting the path (correctness)

`move_through_joint_positions` currently **skips** an intermediate waypoint that times
out, logs a warning, and drives on to the next one — from wherever the arm actually got
to, which is off the path the planner checked for collisions. That converts a recoverable
timeout into an unchecked motion through a cell the planner has cleared for a different
trajectory.

An intermediate waypoint that cannot be reached should fail the move, like the final one.
The loose 2° tolerance for intermediate waypoints stays — that is what keeps motion
flowing rather than stopping dead at every waypoint — so this only bites when the arm is
genuinely stuck, which is the case that must not continue silently.

### 4. Report a stall as a stall (diagnosis)

`probes/layout_check.py` had no branch for stall errors, so a 30 s execution timeout was
reported as `no plan` — a planner that could not find a route. That mislabelling is what
sent the first investigation after the geometry instead of the joints. Already fixed;
recorded here because it is why the bug took three sessions of probing to see.

### 5. Stop the drive when a move fails

Both move paths raised with the last `set_joint_targets` still applied, so a blocked arm
went on pressing into whatever stopped it at full drive force for as long as the sim ran.
They now call `stop()` (which holds the current position) before raising.

## Not done here

**Rotating the arm frames 180°** is the better structural fix: it puts every station near
pan 0 and moves the wrap point behind the arm, removing the ratchet at its source rather
than walling it off. It is not done in this change because it moves every arm base
orientation and would invalidate the verified base-frame yaw work and the station
coordinates in one step, making the verification ambiguous. The clamp is measured to hold
(below), so this is an improvement to sequence deliberately, not a fire.

**Re-sizing the trays** (23 mm gap) is real and unrelated; mixing a geometry change into a
joint-limits fix would make the numbers impossible to attribute.

**Margin.** With the clamp at ±180 the cell now peaks at 179° on one joint. The served
limit needs headroom greater than the 0.5° settle tolerance, or an arm that settles at
180.3° fails its next start check. `probes/windup_check.py` reports the per-joint peak so
this is visible as drift rather than as a jam; the frame rotation above is what gives it
real margin.

## Measured result

`probes/windup_check.py`, resetting then making repeated passes over every station:

| | arm-a peak | arm-b peak | stalls | planning failures |
|---|---|---|---|---|
| before (2 passes) | 270° | 230° | yes | yes |
| clamp only (4 passes) | 179° | 176° | **7** | 0 |
| clamp + floor + base yaw (4 passes) | 179° | **117°** | **0** | 0 |
| same, 6 passes (~180 moves) | **180°** | 117° | 0 | 1 |

The clamp alone bounds the drift but is not sufficient: with the workspace still straddling
the wrap point, both arms crept to the limit and then stalled seven times trying to track
paths that hug it. Adding the base yaw moved arm-b's peak down to 117° and removed the
stalls entirely.

### The residual, stated plainly

**arm-a's j1 reaches exactly 180° and stays there**, and a 6-pass run produced one
planning failure. Clamping puts a wrap discontinuity at ±180 on *every* joint, and
arm-a's shoulder-lift working range straddles it — so the ratchet has moved from j0 to
j1 rather than being eliminated. The base yaw fixes pan, which is why arm-b (whose lift
range does not straddle) is clean at 117°.

The fix for this is **per-joint, asymmetric limits**: arm-a's lift works in roughly
[−180°, 0°], so a range like [−270°, 90°] puts the discontinuity where the arm never
goes, exactly as the base yaw does for pan. `joint_limit_deg` currently takes one
symmetric number and would need to accept a per-joint map. Not done here — the change is
small but wants its own verification run, and mixing it in would make this table
unattributable.

So: the cell no longer jams, it survives ~180 moves where it used to fail within two
passes, and the remaining failure mode is understood and bounded rather than mysterious.

## How the fix is checked

The accumulation table above is the test. Run several passes over the stations and assert
that no joint exceeds the configured limit and that no move stalls — a fresh boot is not
enough to show either the bug or the fix, so the check has to run long enough to
accumulate.
