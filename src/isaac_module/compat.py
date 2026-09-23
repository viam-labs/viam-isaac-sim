"""Isaac imports that move between releases, in one place.

Vendored from DTCurrie/viam-isaac-sim so the two trees carry one copy rather than
two implementations to reconcile when they merge. Kept verbatim for that reason -
edit it upstream, not here.
"""

from __future__ import annotations

from typing import Any


def import_surface_gripper() -> dict[str, Any]:
    """Isaac's surface gripper extension and the USD modules that author it,
    imported once per process. Enables ``isaacsim.robot.surface_gripper``,
    then returns a mapping with the keys ``report`` (what was found, for a
    boot log line), ``surface_gripper`` (the ``_surface_gripper`` module,
    whose ``acquire_surface_gripper_interface()`` drives a gripper by path),
    ``robot_schema``, ``Gf``, ``Sdf``, ``UsdGeom``, ``UsdPhysics`` and
    ``PhysxSchema`` (None where pxr has none). Isaac only."""
    report: dict[str, Any] = {}
    from isaacsim.core.utils.extensions import enable_extension

    report["extension_enabled"] = bool(enable_extension("isaacsim.robot.surface_gripper"))
    from isaacsim.robot.surface_gripper import _surface_gripper

    try:
        from usd.schema.isaac import robot_schema
    except ImportError as first:
        try:
            from isaacsim.robot.schema import robot_schema
        except ImportError as second:
            raise ImportError(
                f"no robot_schema module: usd.schema.isaac ({first}); "
                f"isaacsim.robot.schema ({second})"
            ) from second
        report["robot_schema_module"] = "isaacsim.robot.schema"
    else:
        report["robot_schema_module"] = "usd.schema.isaac"
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    try:
        from pxr import PhysxSchema
    except ImportError:
        # a module name, kept as pxr spells it
        PhysxSchema = None
    report["physx_schema"] = PhysxSchema is not None

    return {
        "report": report,
        "surface_gripper": _surface_gripper,
        "robot_schema": robot_schema,
        "Gf": Gf,
        "Sdf": Sdf,
        "UsdGeom": UsdGeom,
        "UsdPhysics": UsdPhysics,
        "PhysxSchema": PhysxSchema,
    }
