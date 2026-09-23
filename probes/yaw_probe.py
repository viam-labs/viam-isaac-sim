"""Does Isaac's UR5e USD agree with Viam's UR5e SVA about which way the arm faces?

The motion service plans in the frame system, which it builds from the SVA served by
GetKinematics. Isaac simulates the USD. Nothing checks that the two share a base
orientation, and they have every reason not to: Viam's SVA is rooted in the UR
controller's `base` frame, while Isaac's UR assets are URDF imports rooted in ROS
`base_link`, and those differ by a rotation about Z. If they disagree, every planned x/y
is mirrored against the simulation - the planner returns a clear path and the arm drives
somewhere else.

The SVA chain at all-zero joints is pure arithmetic, so it is known in advance:

    world -> base_link      (0, 0, +162.5)
          -> upper_arm_link (-425, 0, 0)
          -> forearm_link   (-392.2, 0, 0)
          -> wrist_1_link   (0, -133.3, 0)
          -> wrist_2_link   (0, 0, -99.7)
          -> ee_link        (0, -99.6, 0)
    = (-817.2, -232.9, +62.8) mm

Spawn the USD at the origin, command all-zero joints, read where the links land. One arm
per run, at the origin: spawning two at once silently ignored the `position` kwarg and
reported both at the origin, which is its own trap but not the one being measured here.

    scripts/run_guarded.sh .venv-isaac/bin/python .scratch/yaw_probe.py            # raw
    scripts/run_guarded.sh .venv-isaac/bin/python .scratch/yaw_probe.py --corrected
"""

import sys

SVA_ZERO_MM = (-817.2, -232.9, 62.8)
# 180 deg about Z, the correction the module composes in for UR assets.
CORRECTION_WXYZ = [0.0, 0.0, 0.0, 1.0]
TIP_NAMES = ("flange", "tool0", "wrist_3_link")


def main():
    corrected = "--corrected" in sys.argv

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})

    import numpy as np
    import omni.timeline
    import omni.usd
    from pxr import UsdGeom, UsdPhysics

    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.storage.native import get_assets_root_path

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("could not resolve the Isaac assets root")
    usd = f"{assets_root}/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

    add_reference_to_stage(usd_path=usd, prim_path="/World/arm")
    kwargs = dict(prim_path="/World/arm", name="arm", position=[0.0, 0.0, 0.0])
    if corrected:
        kwargs["orientation"] = CORRECTION_WXYZ
    art = SingleArticulation(**kwargs)
    world.scene.add(art)
    world.reset()

    omni.timeline.get_timeline_interface().play()
    for _ in range(30):
        app.update()
    art.initialize()

    # SingleArticulation(position=/orientation=) did NOT take effect here: a corrected
    # spawn landed on exactly the raw tip. Set the root pose explicitly after
    # initialize() instead, and report the root prim so the claim is checkable.
    if corrected:
        art.set_world_pose(position=np.array([0.0, 0.0, 0.0]),
                           orientation=np.array(CORRECTION_WXYZ))
    root_p, root_q = art.get_world_pose()
    print(f"root pose after spawn: p={np.round(root_p, 4).tolist()} "
          f"q_wxyz={np.round(root_q, 4).tolist()}")

    art.set_joint_positions(np.zeros(len(art.dof_names)))
    for _ in range(60):
        world.step(render=False)

    xform_cache = UsdGeom.XformCache()
    joints = np.round(art.get_joint_positions(), 4).tolist()
    tips = {}
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        name = path.rsplit("/", 1)[-1]
        if not path.startswith("/World/arm/") or name not in TIP_NAMES:
            continue
        if not prim.IsA(UsdGeom.Xformable):
            continue
        t = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
        tips[name] = tuple(round(v * 1000, 1) for v in t)

    label = "corrected (180 deg about Z)" if corrected else "raw (identity)"
    print(f"\nspawn: {label}")
    print(f"joints after settle: {joints}")
    for name, position in tips.items():
        print(f"  {name:>14}  {position}")

    tip = next((tips[n] for n in TIP_NAMES if n in tips), None)
    if tip is None:
        print("no tip prim found")
        app.close()
        return 1
    err = max(abs(tip[i] - SVA_ZERO_MM[i]) for i in range(3))
    print(f"\nSVA zero pose: {SVA_ZERO_MM}")
    print(f"isaac tip:     {tip}")
    print(f"max error:     {err:.1f} mm")
    print("VERDICT: " + ("AGREES with the SVA" if err < 1.0 else "MISMATCH"))

    app.close()
    return 0 if (err < 1.0) == corrected else 1


if __name__ == "__main__":
    sys.exit(main())
