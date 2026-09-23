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

**Do not refactor onto his fork.** Four reasons, in order of weight:

1. **`viam:isaac-sim-devin` is private, under the `viam` namespace.** We cannot publish to
   it, and a public course that depends on a private module owned by someone else is a
   supply risk that only shows up when it breaks.
2. **It diverges structurally from erh/main** — new package layout, a separate
   `src/pickcell/`. Adopting it means leaving the line Abe pointed us at and re-verifying
   everything measured this session (base-frame yaw, joint wind-up, obstacle visibility,
   the grasp) against a different codebase.
3. **Its value is concentrated in the resolver**, which we cannot use.
4. Its cell is a palletizer/block-sorter. Ours is a two-arm QC cell. The overlap is the
   module, not the application.

**Do adopt these, in this order.** Each is self-contained and independently verifiable:

1. **`depends_on`.** Our `suction-b` failed to build on first attempt with "parent_prim is
   not in the stage" and only succeeded on RDK's retry — a resource-ordering race. He
   orders construction explicitly. One line per gripper, and it removes a real flake.
2. **Scene presentation** — matte ground, viewport grid off, dome lighting. The video made
   the case: the cell reads as objects floating over a blue grid. This is the cheapest
   large improvement available and it is a module capability we would want anyway.
3. **Cup compliance** for the vacuum. Our grasp is verified but flaky — the recording
   shows `holding=True` with the carton left on the belt. He models the bellows as a soft
   limit rather than a weld, and notes that with several cups only one should carry shear,
   because cups locking the same freedoms fight each other and PhysX reports that fight as
   load the plugin reads as a failing grip. That is a plausible cause of our flake.
4. **Fragment composition and `tools/`.** A base world fragment plus a cell fragment, and
   promoting our two `.scratch/` scripts into a real `tools/` with the same job his has.
   Housekeeping, not capability — last.

## The finding that matters more than any of this

There are now **four** implementations diverging: `erh/main` (live), `viam-labs/main`
(stale fork, 2 commits behind, the one Abe pointed at), `viam:isaac-sim-devin` (private,
0.4.1, far ahead), and this branch. We independently rediscovered the multi-arm base
position bug that Devin's line had already fixed, and independently built a gripper that
his module already had.

That is a coordination problem, not a code problem, and no refactor fixes it. Worth raising
before more effort is duplicated.
