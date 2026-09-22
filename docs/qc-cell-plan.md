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
will route the flange clear and swing a 60 mm box straight through arm B — precisely the
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
arm inspecting a 60 mm carton is also wrong for the course visually.

`ur5e` (0.85 m) is the arm this application really uses, and the cell scales down with it.

The scale had to be **measured**, not derived. The tempting argument — every distance
shrinks by `s`, so the ratio between the inspect station and the bins is preserved — is
wrong, because the tool hangs a fixed 0.168 m off the flange and does not scale
(`cell.py:471`: "a bin drop that looked like 1.25 m was 1.29 m at the flange"). That
offset can point toward or away from the base, and the sign differs between the presenting
and bin stations, so it swings the result by more than the margin the ratio argument
claims. `probes/scale_study.py` sweeps the scale against both constraints at once:

| scale | worst station | nearest bin | |
|---|---|---|---|
| 0.56 | 0.693 | 0.842 | good bin reachable |
| **0.58 – 0.62** | 0.713 – 0.754 | 0.870 – 0.926 | **ok** |
| 0.64 | 0.775 | 0.953 | station past the 10% margin |
| 0.68 | 0.816 | 1.009 | station at 96% of reach |

**Scale 0.62.** Worst station `present right (b)` at 0.754 m (89% of reach); nearest bin
0.926 m, 0.076 m beyond reach, so the handoff stays mandatory.

The first draft of this plan said 0.68 on the ratio argument. The sweep shows 0.68 puts
the worst station at 96% of reach, where a spherical reach bound stops being honest — near
full extension the wrist orientation `present_rotation` asks for may not be achievable at
all. That is exactly the "stale check" shape that cost five instrument fixes today.

The part stays 60 mm and the tool 0.12 m — neither scales. The camera comes from 0.57 m to
0.354 m, which *improves* the detector: frame 210 mm → 130 mm, smallest mark
18 px → **30 px** at model input.

The viable window is only 0.04 wide, so the cell barely fits a UR5e and a small layout
change could close it from either side. Widening it means moving the bins outward relative
to the arms — a reshape, not a rescale. Worth doing if phase 1 finds the margins tight in
practice; the sweep is cheap to re-run.

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

**Phase 0 — prove the assumption. `mock: true`, no GPU.**
The README says mock runs without Isaac, so config shape and module wiring iterate with
no 7-minute cycles and no contention with the peer session.
- Local module entry; new machine (`viam-103-qc-cell`); machine config in `.scratch/secrets/`.
- **Measure `motion.Move()` latency.** This is the load-bearing number. A round issues
  ~50 verbs; at ~2 s/plan that is +100 s on a 140 s round, on top of 250 ms inference. If
  it lands there, the round needs to be coarser — better to know now than after the scene
  is built.
- Assert Isaac `position` == `frame.translation` for both arms.

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

## Risks

| risk | mitigation |
|---|---|
| planning latency too slow for a 2 h course | measured in phase 0, before anything depends on it |
| Isaac `position` vs Viam `frame` disagree | asserted in phase 0; planner would otherwise plan in the wrong world |
| held part not in `world_state` → box swings through arm B | `qc:cell` attaches the part geometry to the moving arm on every motion between grasp and release; phase-1 test covers it |
| viable scale window is only 0.04 wide | re-run `scale_study.py` after any layout change; reshape (bins outward) if phase 1 finds it tight |
| gripper blocked on Devin | phase 1 needs no gripper |
| two Isaac instances | one at a time — stop the isaac-arcade sim before testing this one |
| composite props rejected upstream | keep the change small and generic; it is useful beyond QC |
