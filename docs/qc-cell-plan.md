# Viam 103 QC cell on viam-isaac-sim — implementation plan

Branch: `viam-103-qc-cell`. Rebuilds the bimanual QC cell (currently in
`~/viam/isaac-arcade`) on this module, so the learner drives it through standard Viam
APIs and the **motion service** does the path planning.

The existing isaac-arcade cell and its machine stay untouched and keep working. This is
a parallel build on a new machine.

## Why move

isaac-arcade drives Isaac over a private JSON-lines bridge and commands joints directly:
`move_joints` sets targets and the articulation drives straight there. There is no path
planning and no collision avoidance, so every transit collision has to be found by
inspection and fixed by hand-tuning poses. Today's session found them by watching video,
and five successive instrument fixes were needed before the probe agreed with the render.

On this module the arms are `rdk:component:arm` with `GetKinematics` served, so
`motion.Move()` plans around obstacles. That removes the entire transit-collision class
structurally rather than by tuning. It is the main reason to move, which makes planning
latency the assumption the whole plan rests on — measured first, in phase 0.

### The planner can see the arms — verified

That claim only holds if the planner knows where the obstacles are, so it was checked
rather than assumed. `arm.py:221` has `get_geometries()` returning `[]`, which looks
fatal and is not: for an arm the motion service builds its collision model from
`GetKinematics`, and `get_geometries` covers only geometry beyond the kinematic chain.
The UR5e SVA carries a capsule or sphere on **all 7 links** (`base_link` r60×l260,
`upper_arm_link` r65×l550, `forearm_link` r50×l490, …), so the planner does model both
arms and will plan arm A around arm B. This is the arm-vs-arm collision the user reported
three separate times, and it is fixed structurally by the move.

The **held part is not** covered by that. A planner that only knows the kinematic chain
will route the flange clear and swing an 80 mm box straight through arm B — precisely the
reported bug. `qc:cell` must therefore pass the carried part to `motion.Move()` in
`world_state` as a geometry attached to the moving arm, for every motion between grasp and
release. This is a requirement on the verbs, not something the module provides, so it is
in the risk table and in the phase-1 test.

## Decisions

### Arm: `ur5e`, and the cell scales down with it

`ur10` cannot be used. `KNOWN_ASSETS` gives it a USD but no `kinematics` entry, and
upstream has no file — verified directly:

| asset | `…/universal-robots/main/src/kinematics/<a>.json` |
|---|---|
| ur3e, ur5e, ur20 | HTTP 200 |
| **ur10, ur10e, ur16e** | **HTTP 404** |

Without kinematics the motion service cannot plan, which removes the reason for the move.
So the arm must be one of ur3e / ur5e / ur20.

`ur20` looks like the easy swap and is a trap. The layout has a load-bearing constraint —
the picking arm must **not** be able to reach the bins, which is what forces the handoff
and is the whole reason the cell has two arms. Current margins from arm A:

| | distance | vs UR10 (1.28 m) | vs UR20 (1.75 m) |
|---|---|---|---|
| good bin | 1.457 m | out of reach | **reachable** |
| bad bin | 1.715 m | out of reach | **reachable** |

Under UR20 both bins come into reach and the handoff silently becomes optional. A 20 kg
arm inspecting an 80 mm carton is also wrong for the course visually.

`ur5e` (0.85 m) is the arm this application really uses, and the cell scales down with it.

The scale had to be **measured**, not derived. The tempting argument — every distance
shrinks by `s`, so the ratio between the inspect station and the bins is preserved — is
wrong, because the tool hangs a fixed 0.168 m off the flange and does not scale
(`cell.py:471`: "a bin drop that looked like 1.25 m was 1.29 m at the flange"). That
offset can point toward or away from the base, and the sign differs between the presenting
and bin stations, so it swings the result by more than the margin the ratio argument
claims. `probes/scale_study.py` sweeps the scale against both constraints at once:

The reach bound also had to be measured. UR5e is sold as an 850 mm arm, but that is to
the wrist centre; what the layout must satisfy is the bound the **motion service** puts on
a requested flange pose. Asking the planner for a pose it refuses reports it exactly:

```
asked for a pose too far max: 1016.72, asked for: 1076.72
```

| scale | worst station | nearest tray | |
|---|---|---|---|
| 0.68 | 0.816 | 1.009 | good tray reachable from arm-a |
| **0.70 – 0.76** | 0.836 – 0.898 | 1.037 – 1.121 | **ok** |
| 0.78 | 0.918 | 1.149 | station past the 10% margin |

**Scale 0.76.** Worst station 0.898 m (88% of reach); nearest tray 1.121 m, 0.104 m
beyond reach.

This number was wrong twice before it was right, both times for the same reason — a bound
asserted rather than measured. First 0.68, from a ratio argument that ignored the
non-scaling tool offset. Then 0.62, from the sweep run against the 850 mm datasheet
figure, which put the trays "safely" outside a reach that was not the real one: at 0.62 the
planner reached the good tray from arm-a **5/5**. The handoff would have been optional and
nothing in the cell would have said so.

The part stays 80 mm and the tool 0.12 m — neither scales. The camera comes from 0.57 m to
0.433 m, which *improves* the detector: frame 210 mm → 159 mm, smallest mark
18 px → **24 px** at model input.

That improvement depends entirely on setting `fov_deg`. It defaults to **70°**, which at
0.433 m frames 606 mm and puts the smallest mark at **6 px** — unusable. The arcade's lens
is 57 mm, i.e. 20.83°, and the fragment now says so explicitly.

The viable window is 0.06 wide, so a small layout change can close it from either side.
Re-run `probes/scale_study.py` after any change, and re-run the planner check below —
the arithmetic sweep is a fast filter, not the authority.

### Split: generic capability upstream, QC logic in its own module

Anything a non-QC user would want goes on this branch (the README already lists gripper
as a planned TODO, so upstream wants it). Course-specific scoring does **not** — putting
it in a viam-labs repo that has open PRs from other people makes it unmergeable and
couples the course to a repo we do not own.

| goes here (`viam-isaac-sim`) | goes in the course module |
|---|---|
| gripper model (surface gripper) | round/scoring logic |
| composite props (marks) | oracle ground truth |
| runtime prop spawn/remove/pose | defect generation |
| conveyor prop type | the cell verbs |

### Module entry: local, not registry

`meta.json` is `erh:isaac-sim` under erh's namespace — a modified build cannot be
published there and should not be. The new machine gets a **local** module entry pointing
at this working tree.

## What is missing upstream

Four gaps, all generic, all upstreamable:

1. **Gripper** — `README.md:246` is `- [ ] gripper support`. Isaac has
   `isaacsim.robot.surface_gripper`, which is the right primitive for a vacuum tool.
   **Do not build this yet** — Abe says Devin has looked at it and may have code. Ask at
   the 10-minute sync. Phase 1 does not need it: arms moving to poses validates layout,
   frames and planning with no grasp at all.
2. **Runtime prop management** — world `DoCommand` supports only
   `status, play, pause, reset, add_usd`. The round loop needs `spawn_prop`,
   `remove_prop`, and `prop_pose` (the last also feeds the oracle).
3. **Composite props** — see below.
4. **Conveyor** — Isaac ships a conveyor utility
   (`ext_isaacsim_asset_gen_conveyor`); Abe suggests exposing it via `props`. Phase 1 can
   use a fixed cube as the belt and defer the real one.

### Composite props: the hidden hard part

Defect marks do not fit `props` as it stands. A prop is a monochrome cube or a USD
reference, but a mark is a small patch on a face of the part, randomly placed, up to
three per part across six faces. As separate props they would be independent
`DynamicCuboid` rigid bodies and would simply fall off.

Pre-authoring a USD per permutation is not possible. Three options:

| | verdict |
|---|---|
| generate a USD per part at runtime | works, but adds a file-generation step to the round loop |
| texture the part, marks painted in | needs a mesh box with real UVs; `UsdGeom.Cube` has no useful UVs |
| **`children` on a prop** | **chosen** |

Extend a cube prop with `children`: child boxes parented to the same rigid-body Xform
(no `RigidBodyAPI` of their own), so they are rigidly part of the part and move with it.
This is what isaac-arcade already does with child prims, so the mark-placement code ports
directly. It is small, generic ("a prop can be a composite of boxes"), and upstreamable.

Note `feedback_usd_implicit_prim_extents`: set `extent` explicitly on every child or
Replicator's tight 2D boxes inflate.

## The world config

Phase 1 target (`fragments/qc-cell.json`), abbreviated to the parts that carry decisions.
Distances are the UR10 layout × 0.62. **`position` (Isaac spawn) and `frame.translation`
(Viam frame system, in mm) must agree for every arm** — if they disagree the motion
service plans in a world that is not the one being simulated. This is the single largest
correctness risk in the move and phase 0 tests it explicitly.

```json
{
  "components": [
    {
      "name": "sim-world",
      "type": "generic", "model": "erh:isaac-sim:world",
      "attributes": {
        "headless": true, "livestream": true,
        "props": [
          { "name": "belt", "type": "cube", "fixed": true,
            "position": [0.4464, -0.3472, 0.50], "size": 1.0,
            "scale": [0.62, 0.20, 0.06], "color": [0.18, 0.18, 0.20] },
          { "name": "good_tray", "type": "cube", "fixed": true,
            "position": [0.5022, 0.3844, 0.44], "size": 0.25,
            "scale": [1.0, 1.0, 0.10], "color": [0.15, 0.45, 0.20] },
          { "name": "bad_tray", "type": "cube", "fixed": true,
            "position": [0.5022, 0.5828, 0.44], "size": 0.25,
            "scale": [1.0, 1.0, 0.10], "color": [0.55, 0.15, 0.15] }
        ]
      }
    },
    {
      "name": "arm-a", "type": "arm", "model": "erh:isaac-sim:arm",
      "frame": { "parent": "world", "translation": { "x": 0, "y": -310, "z": 319.3 } },
      "attributes": { "world": "sim-world", "asset": "ur5e",
                      "position": [0.0, -0.31, 0.3193] }
    },
    {
      "name": "arm-b", "type": "arm", "model": "erh:isaac-sim:arm",
      "frame": { "parent": "world", "translation": { "x": 0, "y": 310, "z": 319.3 } },
      "attributes": { "world": "sim-world", "asset": "ur5e",
                      "position": [0.0, 0.31, 0.3193] }
    },
    {
      "name": "inspect-cam", "type": "camera", "model": "erh:isaac-sim:camera",
      "frame": { "parent": "world", "translation": { "x": 775, "y": 0, "z": 682 } },
      "attributes": { "world": "sim-world", "target": [0.4216, 0.0, 0.6944],
                      "width": 1280, "height": 720 }
    }
  ]
}
```

Coordinates come from the 0.62 sweep (`probes/scale_study.py`), but the tray and belt
solids are still eyeballed — phase 1 re-derives them and re-runs the reach and clearance
checks before anything is rendered.

## The course module

Separate repo, consuming the sim over standard Viam APIs. Verbs stay — the standing goal
is that the learner writes perception and judgment against high-level cell verbs; what
changes is that those verbs are now implemented on `motion.Move()` instead of a private
bridge.

| model | API | role |
|---|---|---|
| `qc:cell` | generic | `pick`, `present`, `handoff`, `place`; wraps motion service |
| `qc:scoreboard` | sensor | `get_readings` → score, round, calls, accuracy |
| `qc:oracle` | vision | ground-truth detections via world `prop_pose` |

The learner implements judgment against `qc:cell` + a camera, and is graded by
`qc:scoreboard`. `qc:oracle` is the reference implementation and the phase-1 validation
path.

## Sequencing

**Phase 0 — done. `mock: true`, no GPU.**

Machine `viam-103-qc-cell` exists (part `24314c96…`, separate from the isaac-arcade part
`8be3608f…`, which is untouched), running a **local** module entry against this tree, with
its cloud config in `.scratch/secrets/`. All four components construct in 258 ms and
`rdk:service:motion/builtin` comes up.

The module did not build at all at first: `requirements.txt` said `viam-sdk>=0.80.0`, which
resolves to 0.82, and 0.82 adds five abstract methods to `Arm`, so `IsaacArm` cannot be
instantiated. Pinned to 0.80.0; implementing those methods is what lifts the pin.

**The layout holds, checked against the planner rather than against arithmetic.**
`.scratch/verify_layout.py` drives all 17 stations at scale 0.76: every station an arm must
reach is reached, and both trays are refused from arm-a with `too far`. That is the
handoff constraint proven by the same solver the course will run.

**Planning latency: settled, and it is cheap.** `motion.Move()` plans *and* executes and
only returns once the arm has arrived, and the SDK has no plan-only call, so the two could
not be told apart. Running the move as a concurrent task and polling the joints makes the
split observable: the instant they first budge is the boundary.

Measured against the real obstacle set, with scenery and both tools declared:

| | median | max |
|---|---|---|
| planning | 150 ms | 175 ms |
| execution | 143 ms | 850 ms |

437 ms per move, so a 50-move round is about **22 s of arm time** — not the ~100 s that
was the risk this phase existed to rule out. Planning is the smaller half and barely
varies.

An earlier note here claimed 3-5 ms and concluded planning was free; that was an artifact
of repeating a move the arm had already made, which returns immediately having done
nothing. `probes/latency_check.py` reports moves that produced no motion separately for
exactly that reason. It also commands one fixed tool orientation, so the four
side-presenting stations collapse onto one flange pose and read as no-ops — real
presenting rotates the part, so execution will be higher than this table. Planning will
not be.

**Phase 1 — layout, no gripper.** Scaled ur5e layout; re-run the reach check (all
stations < 0.85 m, both bins > 0.85 m from arm A); arms move to every station under the
motion service. Validate with `--vision oracle`.

**Phase 2 — gripper.** After the sync with Abe/Devin. Surface gripper, then real grasps.

**Phase 3 — QC logic.** Port scoring, defect generation, round management onto the verbs.

**Phase 4 — retrain.** Only once the scene is settled.

### The one test that proves the machine works

A round runs end to end on the new machine with `--vision oracle`: a part spawns at the
belt head, arm A picks it, presents all faces to the inspection camera, hands off to arm
B, arm B places it in the tray the oracle's verdict selects, and the scoreboard reads 6/6,
with **every arm motion issued through `motion.Move()`**.

The first draft added "and zero collisions reported, without a clearance probe, because
the planner is what is being tested." That is circular: remove the probe and nothing
reports collisions, so the test passes whether the planner works or not — the same shape
as the five stale checks that had to be fixed today. The planner has to *earn* that trust:

* keep an independent part-vs-arm check running through phase 1, and
* **record the round.** Watching the video is what actually caught every collision this
  session, while the instruments agreed the cell was clean.

The probe can retire once a run with the check enabled reports clean and the video agrees
— not before, and not on the strength of the architecture.

## Explicitly not doing

The deferred isaac-arcade scene work is throwaway under this architecture: floating
backdrop, six-face carton printing, enlarging the camera, the overhead-camera option, and
arm B's home pose. Carry the *diagnoses* forward — in particular that arm B's home pose
parks a wrist at x=0.739 beside the inspection point, since arms are placed by frame
config here and the mistake is easy to repeat — but spend no renders on the fixes.

`isaac-cell-marks-v3` was trained on the current framing, backdrop and part appearance. A
props-built scene invalidates it; phase 1 validates with the oracle and the retrain waits
for a settled scene.

## Open findings from review

Ordered by what breaks worst. Items 1 and 2 gate phase 1.

1. ~~**Base-frame yaw between the SVA and the Isaac USD.**~~ **Resolved.** It was real:
   Isaac's UR assets are URDF imports rooted at ROS `base_link` while Viam's kinematics
   use the UR controller's `base` frame, a half turn apart about Z, so every planned x and
   y was mirrored against the simulation. Measured on ur5e at all-zero joints — SVA
   (−817.2, −232.9, 62.8) mm vs Isaac (817.2, 232.9, 63.1) — and fixed by composing the
   half turn into the spawn (`_UR_BASE_ROTATION_WXYZ`).

   Fixing it exposed two further bugs. `SingleArticulation`'s `position`/`orientation`
   kwargs do nothing on Isaac 6.1, and `set_world_pose` sets the *current* pose while
   `world.reset()` restores the *default* — and the module resets on every arm creation,
   so with two arms the last one built silently returned to the origin. The cell came up
   with arm-b correct and arm-a mirrored, which reads as a layout problem rather than a
   spawn bug.

   `probes/verify_yaw_e2e.py` now checks it through the running machine: both arms land
   within 0.3 mm of the SVA prediction, agree with the frame system exactly, and report
   different positions from each other.

2. ~~**The planner is blind to the scenery and to both tools.**~~ **Resolved.** Confirmed
   the hard way first: asked to put a flange inside the belt, the planner planned straight
   there and the arm drove in until the isaac collider stalled it. Nothing is
   auto-collected from components — obstacles have to be passed per call.

   `sim-world` now answers `DoCommand {"command": "obstacles"}` with every cube prop as
   `{label, fixed, center_mm, dims_mm}`. Reported rather than published as component
   geometry because `fixed` is not static: a fixed belt is an obstacle forever, while a
   dynamic part is an obstacle until an arm grasps it and then becomes part of *that arm*
   — the same prop, two roles, switching mid-round. Only the caller knows which.

   The tool is a `Transform` parented to the arm's end-effector frame, not an entry in
   `obstacles`: obstacles resolve to world once at plan start, so an obstacle "on" the arm
   would sit wherever the arm happened to be. A transform with a `physical_object` is
   carried by its parent and travels with the arm. The held part will use the same
   mechanism.

   `probes/obstacle_check.py` verifies all of it, with a control: a pose the flange can
   legally reach must still *succeed* without the tool declared and be *refused* once it
   is. Both hold, which is also what proves the tool axis is right.

3. **Do not port `ee_pose()`.** Settled while building the tool geometry. The SVA's
   `ee_link` carries `ov_degrees (0, −1, 0, 90)`, and a Viam orientation vector names the
   frame's **z-axis**, so the end-effector frame's +z is the direction the final
   (0, −99.6, 0) translation continues — out of the flange. **The tool runs along +z of
   the arm frame.** `cell.py`'s `EE_FROM_TOOL` assumes +x, the URDF `ee_link` convention,
   so porting it would command the wrist 90° off and read as a tolerance problem.
4. **Execution does not track the plan.** `move_through_joint_positions` sets joint targets
   and lets the articulation drive there at its own gains, unsynchronized, so the path
   between coarse waypoints bows off the segment the planner checked; `MoveOptions` are
   ignored; and a waypoint that times out is **skipped with a warning**, after which the
   arm cuts to the next one from wherever it is — off the checked path entirely. This is
   the strongest argument for keeping the independent probe and the video, stronger than
   distrust of the planner itself. Failing the move on an intermediate timeout is the fix.
5. **Runtime prop management is what buys fast iteration, not mock.** Props spawn only at
   boot and handles are cached across reconfigure, so every belt or tray tweak in phase 1
   is a full Isaac restart. Build `spawn_prop` / `remove_prop` before iterating on layout.
   Neither may call `reset()` — the module already resets on every arm creation and on
   `DoCommand reset`, which snaps every prop and both arms back to spawn state.
6. **Composite `children` must carry neither `RigidBodyAPI` nor `CollisionAPI`.** A
   colliding mark on the bottom face would tilt a resting part by its own thickness.
7. **`qc:oracle` needs more than `prop_pose`.** The module exposes no camera intrinsics,
   and the fragment's camera frame has translation but no orientation while Isaac aims it
   via `target` — so the oracle cannot currently project ground truth into image space.
8. **Name the dependency rather than claim it was avoided.** The course module needs
   `spawn_prop`, `prop_pose`, the gripper, the tool frame and intrinsics — all of which
   live in this fork, pinned as a local path. That is fine for an instructor machine, but
   it is a coupling, and gaps 2–4 above are not yet assigned to a phase.

## What the obstacle work turned up

Running all 17 stations against the scenery and the tool (`probes/layout_check.py`)
settles some things and opens others.

**The planner now catches arm-vs-arm interference.** This was demonstrated accidentally
and is the clearest evidence the move was worth making: with arm-a left parked at the
inspection point by the previous station, every one of arm-b's presenting stations was
refused. That is the exact collision the user reported three times from video, now refused
before anything moves. The check retreats the idle arm first, because a round never has
both arms at the inspection point either.

**16 of 17 stations plan cleanly** with scenery and both tools declared. Median planning
298 ms.

**Two open problems, and they are different failures.**

1. `place_bad` fails with "no plan" after a 30 s timeout **in the full sequence**, but
   reaches in 223–1268 ms when tried in isolation — from a clean home, from `place_good`,
   and with arm-a sitting at the retreat pose. So it is state-dependent rather than
   random, and not yet root-caused. It is not obviously geometric: the trays are only
   23 mm apart at my provisional sizing, which is worth fixing regardless, but the same
   goal plans fine most of the time.
2. **Execution stalls are separate from planning.** Arm-a was seen to plan a move to an
   empty pose and then physically stall against nothing the planner knew about. That is
   finding 4 (execution does not track the plan) showing up in practice, and it is why the
   independent probe and the video have to stay.

Neither is a reason to stop, but "the planner removes the collision class" should be read
as "for planning" — execution is still its own risk.

## Risks

| risk | mitigation |
|---|---|
| planning latency too slow for a 2 h course | measured in phase 0, before anything depends on it |
| ~~Isaac `position` vs Viam `frame` disagree~~ | resolved — see finding 1; verified end to end on both arms |
| held part not in `world_state` → box swings through arm B | mechanism now proven: a `Transform` parented to the arm's EE frame, same as the tool. `qc:cell` attaches the part between grasp and release; phase-1 test covers it |
| planning is state-dependent: one station times out at 30 s in sequence but plans in ~1 s alone | root-cause before the round loop depends on it; a 30 s stall is fatal to a 2 h course |
| viable scale window is only 0.04 wide | re-run `scale_study.py` after any layout change; reshape (bins outward) if phase 1 finds it tight |
| gripper blocked on Devin | phase 1 needs no gripper |
| two Isaac instances | one at a time — stop the isaac-arcade sim before testing this one |
| composite props rejected upstream | keep the change small and generic; it is useful beyond QC |
