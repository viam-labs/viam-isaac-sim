# Joint wind-up: diagnosis and fix

Both open problems from the obstacle work — `place_bad` failing to plan in sequence but
not in isolation, and arms stalling on moves to empty poses — are the same bug.

## Diagnosis

The UR5e SVA gives every revolute joint a range of **±360°**. Viam's motion service is
free to pick any IK solution inside that range, and it has no reason to prefer the one
nearest the current pose. So each move can add most of a turn to a joint, and successive
moves accumulate:

| | arm-a max abs joint | arm-b max abs joint |
|---|---|---|
| after `world reset` | 0° | 0° |
| after one pass over the stations | 97° | 230° |
| after two passes | **270°** | 230° |

Arm-a's joint 0 went −26.4° → 206.4° in a single pass. Once a joint approaches 360° the
cell degrades in three separate-looking ways:

* **planning fails** — the planner must unwind to reach the goal, and often cannot;
* **execution stalls** — the arm is commanded into a contorted configuration and fails to
  settle within the 0.5° final tolerance, timing out after `move_timeout` (30 s);
* **the move is rejected outright** — `joint 0 needs to be within range [-360, 360] and
  cannot be moved to 363.6`.

That is why the failures looked intermittent and state-dependent, and why every isolated
retest passed: a fresh boot starts at zero, and the symptom needs a few dozen moves to
appear. It also means the cell would jam within a round or two of a real course — a round
issues around fifty moves.

Nothing here is specific to the QC cell. Any two-arm scene driven through the motion
service on this module will wind up the same way.

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

## How the fix is checked

The accumulation table above is the test. Run several passes over the stations and assert
that no joint exceeds the configured limit and that no move stalls — a fresh boot is not
enough to show either the bug or the fix, so the check has to run long enough to
accumulate.
