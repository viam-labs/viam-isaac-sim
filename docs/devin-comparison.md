# Devin's module vs this branch: what to adopt, and what not to

Reviewed `DTCurrie/viam-isaac-sim` (Devin Currie — the "devin" Abe said had looked at
vacuum grippers). It is a fork of `erh/viam-isaac-sim` carrying 224 files and ~65k lines
beyond it, with 96 test files, published as the **private registry module
`viam:isaac-sim-devin` at 0.4.1**.

## What he has that we do not

| | his | ours |
|---|---|---|
| models | world, arm, camera, base, **gripper, vacuum, scene-finalizer, conductor, sorter-sensor, palletizer** | world, arm, camera, base, gripper |
| scene | matte ground plane, viewport grid off, dome HDRI + sphere lighting | Isaac's default grid, no lighting control |
| config | `simulates.json` table + `config_resolver.py` deriving a sim twin from a **real** machine's config | one hand-authored fragment |
| fragments | a base world fragment plus per-cell fragments, with `$variable` parameters | one monolith |
| ordering | `depends_on` and a `scene-finalizer` component | none |
| tooling | `tools/` — machine creation, config resolution, shader-cache warming, mesh conversion | two scripts in `.scratch/` |
| vacuum | cup compliance modelled: bellows as a soft limit, one cup carries shear | single rigid cup |

## Corrections after review

Four of this document's original arguments were wrong or weak, and the strongest reason to
stay was missing. Recording that, because the conclusion survived and the reasoning did
not.

**The Isaac version gap is the real reason, and it was absent.** Devin targets Isaac Sim
**5.0**; this machine runs **6.1** (5.1 crashes on driver 595). Building on his module
means porting 65k lines from 5.0 to 6.1 or downgrading Isaac. That single fact decides the
question, and it also prices every piece of his physics code we might lift.

**"Private module = supply risk" was inverted.** Our fragment ships
`"type": "local", "executable_path": "/home/shrews/viam/..."` — a path on one laptop is a
worse supply chain than a private module in our own org's namespace, and "we cannot publish
to it" is a permissions question with a short answer. The real supply risk is ours:
`_UR_KINEMATICS` fetches SVA files from raw.githubusercontent.com at first boot, so an
offline classroom cannot plan. Devin packages his kinematics.

**"Re-verify everything" was half true.** He already carries the base-frame yaw correction
(`base_frame_correction (0,0,0,1)` for ur5e, with the same finding we measured). What he
does not have is the joint wind-up clamp, and no two-arm cell has run on his line.

**"Its value is concentrated in the resolver" was wrong.** The resolver is one of ~60
source files. He also has `spawn_prop` / `set_prop_pose` / `ignore_props`, a single
`_reset_world` chokepoint with post-reset hooks, a reattach path that avoids a duplicate
reset, stall detection that fails the move, packaged kinematics, and config validators —
several of which are things this plan lists as missing.

**"No real counterpart" did not survive either.** `simulates.json` has rows for
`viam:universal-robots:ur5e`, `viam:robotiq:epick` and a RealSense; a QC cell authored as a
*real* config resolves into its sim twin. For a course, "the config is the machine, the sim
is derived from it" is arguably a better lesson than a hand-authored sim fragment. What the
resolver cannot produce is the scenery — belt, trays, pedestals — and his four-cup epick is
not our single cup.

He is also three commits behind `erh/main` (forked in August, missing #2, #4 and #5). No
line contains everything.

## The central idea, and why it is not ours

His architecture treats **the real machine's config as the source of truth** and derives
the sim from it: every hardware model with a row in `simulates.json` is rewritten to its
sim counterpart, templates and `carry` maps applied, everything else left byte for byte.
Resolving twice is a no-op. The docstring is explicit that this is "the reference behavior
for an app button or a viam-server flag that would do the same thing".

That is a good idea aimed at a problem we do not have. **The Viam 103 cell has no real
counterpart** — it is invented for teaching, so there is no config to resolve from. Adopting
the resolver would buy us machinery with nothing to feed it.

## Recommendation: adopt pieces, do not move house

**Do not refactor onto his fork**, for one reason that decides it and two that support it:

1. **He targets Isaac 5.0; we run 6.1.** Moving means a 65k-line port or an Isaac
   downgrade that this machine's driver will not take.
2. **It diverges structurally from erh/main** — new package layout, a separate
   `src/pickcell/` — so adopting it means leaving the line Abe pointed us at, and
   re-verifying the joint wind-up work and the two-arm cell, neither of which his line has.
3. Its cell is a palletizer/block-sorter; ours is a two-arm QC cell. The overlap is the
   module, not the application — which is the argument for *sharing* the module, and so for
   sending our fixes upstream rather than keeping them here.

**Do adopt these, in this order** (revised after review):

1. **Root-cause the grasp flake first** — it is the only item that blocks the course. Cup
   compliance is *not* the cause: that failure mode needs two or more cups fighting over
   the same freedoms, and we have one, fully locked. The applicable finding from his code
   is different and does apply to a single cup: the plugin's coaxial check compares one
   physics step's force against the limit with no averaging window, so a stiff-drive
   waypoint onset spikes it. Our `coaxial_force_limit` default is 50 N against an unknown
   part mass. Also check whether `is_holding_something` reads a relationship that stays
   populated after the joint breaks.
2. **Gripper ordering, done in the module rather than the fragment**, plus a reset after
   authoring the gripper prims. Devin derives the dependency from `frame.parent`, so a
   learner editing a fragment cannot lose it. The reset matters because the surface-gripper
   plugin only looks for grippers on the first physics frame after play; today our grippers
   work only because a later arm creation happens to reset after them, and ordering the
   grippers last would leave the final one inert. That reset re-introduces state-snapping,
   so it wants his post-reset hooks with it. Fix the gripper geometry bug in the same
   change (below).
3. **Stall detection that fails the move**, from his arm handle — the plan already names
   this as the fix for finding 4.
4. **`spawn_prop` / `set_prop_pose` / `ignore_props`, packaged kinematics, config
   validators.** `ignore_props` is exactly the "obstacle until grasped, then carried"
   mechanism this plan describes. Packaged kinematics removes the offline-classroom
   failure. Validators are the cheapest place to catch the class of config error that cost
   this session several full Isaac boots — wrong tool axis, wrong flange path, `fov_deg`
   defaulting to 70.
5. **Scene presentation.** Not a config copy: `ground`, `render.viewport_grid` and
   `lighting.dome` are keys backed by ~300 lines of his code plus an HDRI asset, and his
   matte settings are confirmed on 5.0 only. Real feature, priced accordingly.
6. **Namespace, then fragments.** Fragment composition is meaningless while the module
   entry is a local path, so pick a namespace first. Drop `tools/` — his tools do a
   different job, and ours is already covered by existing skills.

Also worth taking, as a pattern rather than a file: a constants module both the fragment
and the probes import, with a test proving the reach window, replacing the 60-line
`_comment` in `qc-cell.json` and the hand-copied 0.76 numbers.

### A bug this review found in our own code

`get_geometries` on the gripper places the cup capsule using `offset[0]` as x and
`offset[2]` as z — but `offset` is documented and configured in the *parent prim's* frame,
where the tool runs along +x, while the geometry is reported in the Viam frame, where the
tool is +z. With the fragment's `[0.12, 0, 0]` the reported cup sits 120 mm sideways. It is
latent only because our grippers carry no `frame`; adding one to fix the ordering makes it
live. The unit test misses it because it uses a z-only offset.

## The finding that matters more than any of this

There are now **four** implementations diverging: `erh/main` (live), `viam-labs/main`
(stale fork, 2 commits behind, the one Abe pointed at), `viam:isaac-sim-devin` (private,
0.4.1, far ahead), and this branch. We independently rediscovered the multi-arm base
position bug that Devin's line had already fixed, and independently built a gripper that
his module already had.

That is a coordination problem, not a code problem, and no refactor fixes it. Concrete
actions rather than an observation:

* send the base-frame yaw fix, the joint wind-up clamp and the two-arm findings to
  `erh/main` as PRs, since Devin merges from there too;
* ask Devin to rebase onto erh `#5`;
* settle with Abe whether `viam-labs/main` tracks `erh/main` or is retired.

Worth being blunt about the pattern: this plan's own phase 2 said "do not build the gripper
yet, ask Devin", the gripper was built anyway, and two of its fixes were then lifted from
his code. The cost of not coordinating is already measurable.
