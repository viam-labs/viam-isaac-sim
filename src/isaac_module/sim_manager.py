"""The singleton that owns Isaac Sim.

Isaac Sim (Omniverse Kit) wants to be created and stepped from a single
thread, so the module runs it on the process main thread (see main.py) and
everything else - the Viam module server, component handlers - submits work
to that thread through a queue. Handles returned by create_arm/create_camera/
create_base wrap that queue so component models can stay simple.

A "mock" backend (world attribute: {"mock": true}) implements the same
handle interfaces with plain python so the module can run and be tested on
machines without Isaac Sim installed.
"""

import math
import queue
import sys
import threading
import time
from concurrent.futures import Future
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from viam.logging import getLogger

LOGGER = getLogger("viam-isaac-sim")

# Assets shipped on the Isaac Sim nucleus/content server, addressable by a
# short name in component config. Paths are relative to the assets root;
# where isaac 5.0 moved an asset, the 5.0 path is listed first with the 4.x
# path as a fallback - the first candidate that exists is used.
_UR_KINEMATICS = "https://raw.githubusercontent.com/viam-modules/universal-robots/main/src/kinematics"

# Isaac's Universal Robots assets are URDF imports, so their root prim is ROS
# `base_link`. Viam's UR kinematics are expressed in the UR *controller's* `base`
# frame, and the two differ by a half turn about Z. Spawning the USD unrotated
# therefore mirrors every x and y between what the motion service plans and what
# Isaac simulates - the planner returns a clear path and the arm drives elsewhere.
#
# Measured on ur5e, all joints zero, USD spawned at the origin with identity
# orientation: Isaac puts the flange at (817.2, 232.9, 63.1) mm while the SVA chain
# gives (-817.2, -232.9, 62.8). Composing this rotation into the spawn lines them up.
_UR_BASE_ROTATION_WXYZ = (0.0, 0.0, 0.0, 1.0)  # 180 deg about Z

KNOWN_ASSETS: Dict[str, Dict[str, Any]] = {
    "ur3e": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur3e/ur3e.usd"],
        "kinematics": f"{_UR_KINEMATICS}/ur3e.json",
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "ur5e": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"],
        "kinematics": f"{_UR_KINEMATICS}/ur5e.json",
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "ur10": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur10/ur10.usd"],
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "ur10e": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"],
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "ur16e": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd"],
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "ur20": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur20/ur20.usd"],
        "kinematics": f"{_UR_KINEMATICS}/ur20.json",
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "franka": {
        "usd": [
            "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            "/Isaac/Robots/Franka/franka.usd",
        ]
    },
    "jetbot": {
        "usd": [
            "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
            "/Isaac/Robots/Jetbot/jetbot.usd",
        ],
        "wheel_joints": ["left_wheel_joint", "right_wheel_joint"],
        "wheel_radius": 0.03,
        "wheel_base": 0.1125,
    },
}


GROUND_PLANE_NAME = "ground_plane"
GROUND_PLANE_PRIM_PATH = "/World/GroundPlane"
DOME_LIGHT_PRIM_PATH = "/World/DomeLight"
SPHERE_LIGHT_PRIM_PATH = "/World/defaultLight"
MATTE_OBJECT_SETTING = "/rtx/matteObject/enabled"
SHADOW_CATCHER_SETTING = "/rtx/shadowCatcher/enabled"
VIEWPORT_GRID_SETTING = "/app/viewport/grid/enabled"
DEFAULT_DOME_INTENSITY = 1000.0
DEFAULT_DOME_COLOR = (1.0, 1.0, 1.0)

# A floor, not a drawing of one. Isaac's default is a lit grid, which reads as a
# CAD viewport rather than a room - the thing that made the first cell video look
# like parts floating over graph paper.
GROUND_DEFAULTS: Dict[str, Any] = {
    "kind": "grid",
    "size": 40.0,
    "color": (0.28, 0.28, 0.30),
    "friction": 0.6,
    "restitution": 0.0,
    "matte": False,
}


def ground_plan(
    ground: Optional[Dict[str, Any]], usd_stage: Optional[str]
) -> Tuple[str, Dict[str, Any]]:
    """Pure. What `_boot` should author for the floor.

    "skip" when a ground config is set alongside someone's own usd_stage, "grid" for the
    default, "none" to author nothing, or "plane" with the kwargs add_ground_plane takes.
    """
    if usd_stage is not None and ground is not None:
        return "skip", {"reason": f"usd_stage {usd_stage!r} is set"}
    if ground is None:
        return "grid", {}
    kind = ground.get("kind", GROUND_DEFAULTS["kind"])
    if kind == "grid":
        return "grid", {}
    if kind == "none":
        return "none", {}
    friction = float(ground.get("friction", GROUND_DEFAULTS["friction"]))
    return "plane", {
        "size": float(ground.get("size", GROUND_DEFAULTS["size"])),
        "color": [float(v) for v in ground.get("color", GROUND_DEFAULTS["color"])],
        "static_friction": friction,
        "dynamic_friction": friction,
        "restitution": float(ground.get("restitution", GROUND_DEFAULTS["restitution"])),
    }


def ground_is_matte(ground: Optional[Dict[str, Any]]) -> bool:
    """Pure. A matte floor is invisible to primary rays but still catches shadows.

    Useful for compositing a cell over a backdrop; wrong for a room, where the floor is
    part of what you are looking at. Off by default for that reason.
    """
    if ground is None:
        return bool(GROUND_DEFAULTS["matte"])
    return bool(ground.get("matte", GROUND_DEFAULTS["matte"]))


@dataclass
class SimConfig:
    mock: bool = False
    headless: bool = True
    livestream: bool = True
    usd_stage: Optional[str] = None
    physics_dt: float = 1.0 / 60.0
    rendering_dt: float = 1.0 / 60.0
    boot_timeout: float = 300.0
    # IP the livestream advertises to clients; auto-detected if empty
    livestream_public_ip: str = ""
    # resolution kit renders (and therefore streams) at
    livestream_width: int = 1280
    livestream_height: int = 720
    # props to spawn into the scene at boot; each entry:
    #   {"type": "cube"|"usd", "name": ..., "position": [x,y,z] (m),
    #    "size": edge_m, "scale": [sx,sy,sz], "color": [r,g,b] 0-1,
    #    "fixed": bool, "usd_path": ...}
    props: List[Dict[str, Any]] = field(default_factory=list)
    # kit console verbosity (verbose/info/warning/error). Kit prints thousands
    # of lines at info, and viam-server records the module's stderr as
    # error-level logs, so default to warning.
    kit_log_level: str = "warning"
    # scene presentation. `ground` picks what the floor is, `render` carries the
    # viewport overlays, `lighting` the dome and key lights.
    ground: Optional[Dict[str, Any]] = None
    render: Dict[str, Any] = field(default_factory=dict)
    lighting: Dict[str, Any] = field(default_factory=dict)


class SimManager:
    """Owns the sim thread. Get the process-wide instance via SimManager.get()."""

    _instance: Optional["SimManager"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get(cls) -> "SimManager":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = SimManager()
            return cls._instance

    def __init__(self) -> None:
        self._tasks: "queue.Queue[Tuple[Callable[[], Any], Future]]" = queue.Queue()
        self._boot_requested = threading.Event()
        self._booted = threading.Event()
        self._boot_error: Optional[BaseException] = None
        self._stop = threading.Event()
        self._sim_thread_id: Optional[int] = None

        self.cfg: Optional[SimConfig] = None
        self.mock = False
        self._sim_app = None
        self.world = None
        self._isaac = None  # lazily-populated namespace of isaac imports
        self._step_callbacks: Dict[str, Callable[[float], None]] = {}
        # component name -> (spawn attrs, handle). viam-server rebuilds
        # resources on config change, but prims can't be re-spawned without
        # restarting kit, so handles are cached per component name.
        self._handles: Dict[str, Tuple[Dict[str, Any], Any]] = {}

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------

    def ensure_booted(self, cfg: SimConfig) -> None:
        """Called by the world component's reconfigure. Boots the sim on the
        sim thread the first time; subsequent calls with a different config
        log that a module restart is required (Kit can't be re-created)."""
        if self._booted.is_set():
            if self.cfg != cfg:
                LOGGER.warning(
                    "isaac sim is already running; changes to world config "
                    "(stage/headless/etc) require restarting the module"
                )
            return
        if self._boot_error is not None:
            raise RuntimeError(f"isaac sim failed to boot previously: {self._boot_error}")

        self.cfg = cfg
        self._boot_requested.set()
        if not self._booted.wait(timeout=cfg.boot_timeout):
            raise TimeoutError(f"isaac sim did not boot within {cfg.boot_timeout}s")
        if self._boot_error is not None:
            raise RuntimeError(f"isaac sim failed to boot: {self._boot_error}")

    def request_stop(self) -> None:
        self._stop.set()

    def main_loop(self) -> None:
        """Run forever on the owning (main) thread: wait for a boot request,
        boot, then step the sim while draining queued tasks."""
        self._sim_thread_id = threading.get_ident()

        while not self._stop.is_set() and not self._boot_requested.wait(timeout=0.1):
            self._drain_tasks()

        if self._stop.is_set():
            return

        try:
            self._boot()
        except BaseException as e:  # SimulationApp failures can be SystemExit
            LOGGER.exception("failed to boot isaac sim")
            self._boot_error = e
            self._booted.set()
            return
        self._booted.set()

        last = time.monotonic()
        while not self._stop.is_set():
            self._drain_tasks()
            now = time.monotonic()
            dt = now - last
            last = now
            if self.mock:
                for cb in list(self._step_callbacks.values()):
                    cb(dt)
                time.sleep(0.01)
            else:
                self.world.step(render=True)

        if self._sim_app is not None:
            try:
                self._sim_app.close()
            except Exception:
                LOGGER.exception("error closing isaac sim")

    def _drain_tasks(self) -> None:
        while True:
            try:
                fn, fut = self._tasks.get_nowait()
            except queue.Empty:
                return
            if fut.set_running_or_notify_cancel():
                try:
                    fut.set_result(fn())
                except BaseException as e:
                    fut.set_exception(e)

    def run(self, fn: Callable[[], Any], timeout: float = 30.0) -> Any:
        """Run fn on the sim thread and return its result."""
        if threading.get_ident() == self._sim_thread_id:
            return fn()
        fut: Future = Future()
        self._tasks.put((fn, fut))
        return fut.result(timeout=timeout)

    # ------------------------------------------------------------------
    # boot
    # ------------------------------------------------------------------

    def _boot(self) -> None:
        cfg = self.cfg
        assert cfg is not None
        if cfg.mock:
            LOGGER.info("booting in MOCK mode - no isaac sim")
            self.mock = True
            return

        LOGGER.info("booting isaac sim (headless=%s)...", cfg.headless)
        try:
            from isaacsim import SimulationApp  # isaac sim >= 4.5
        except ImportError:
            from omni.isaac.kit import SimulationApp  # older releases

        # quiet kit's console stream; unknown argv entries are forwarded to kit
        level = cfg.kit_log_level.capitalize()
        sys.argv.append(f"--/log/outputStreamLevel={level}")

        streaming = cfg.livestream and cfg.headless
        launch: Dict[str, Any] = {"headless": cfg.headless}
        if streaming:
            # SimulationApp hides kit's UI whenever headless is set, which
            # leaves a connected streaming client looking at an empty frame.
            # These mirror isaacsim.exp.full.streaming and the livestream
            # standalone example: keep the UI, render it at a size worth
            # streaming, and show the default grid.
            launch.update(
                hide_ui=False,
                width=cfg.livestream_width,
                height=cfg.livestream_height,
                window_width=cfg.livestream_width,
                window_height=cfg.livestream_height,
                renderer="RaytracedLighting",
                display_options=3286,
            )

        self._sim_app = SimulationApp(launch)

        try:
            import carb.settings

            carb.settings.get_settings().set("/log/outputStreamLevel", level)
        except Exception:
            pass

        if streaming:
            try:
                try:
                    from isaacsim.core.utils.extensions import enable_extension
                except ImportError:
                    from omni.isaac.core.utils.extensions import enable_extension

                ip = cfg.livestream_public_ip or _local_ip()
                self._sim_app.set_setting("/app/window/drawMouse", True)
                # the client asks to resize the stream as soon as it connects;
                # without this the request is refused and the view stays blank
                self._sim_app.set_setting("/app/livestream/allowResize", True)
                self._sim_app.set_setting("/app/livestream/port", 49100)
                if ip:
                    self._sim_app.set_setting("/app/livestream/publicEndpointAddress", ip)
                # 5.0 ships the streaming service under omni.services; 4.5 and
                # older only have the kit extension
                for ext in ("omni.services.livestream.nvcf", "omni.kit.livestream.webrtc"):
                    if enable_extension(ext):
                        break
                else:
                    raise RuntimeError("no livestream extension could be enabled")
                LOGGER.info(
                    "livestream enabled - connect the 'Isaac Sim WebRTC Streaming "
                    "Client' app to %s (TCP 49100 + UDP 47998 must be reachable)",
                    ip or "<this machine's IP>",
                )
            except Exception:
                LOGGER.exception("could not enable livestream; continuing without it")

        self._isaac = _import_isaac()

        if cfg.usd_stage:
            LOGGER.info("opening stage %s", cfg.usd_stage)
            self._isaac.open_stage(cfg.usd_stage)

        self.world = self._isaac.World(
            physics_dt=cfg.physics_dt,
            rendering_dt=cfg.rendering_dt,
            stage_units_in_meters=1.0,
        )
        self._add_ground(cfg)
        self._apply_viewport_grid(bool(cfg.render.get("viewport_grid", True)))
        self._apply_lighting(cfg.lighting)
        for prop in cfg.props:
            try:
                self._spawn_prop(prop)
            except Exception:
                LOGGER.exception("failed to spawn prop %s", prop.get("name"))
        self.world.reset()
        LOGGER.info("isaac sim world ready")

    def _spawn_prop(self, prop: Dict[str, Any]) -> None:
        """Add a configured prop to the scene (runs on the sim thread,
        before the initial world.reset)."""
        import numpy as np

        from .spatial import to_vec3

        if not prop.get("name"):
            raise ValueError(f"every prop needs a name: {prop}")
        name = _prim_name(str(prop["name"]))
        prim_path = f"/World/{name}"
        position = list(to_vec3(prop.get("position")))
        kind = str(prop.get("type", "cube"))

        if kind == "usd":
            usd_path = prop.get("usd_path")
            if not usd_path:
                raise ValueError(f"prop {name}: type 'usd' needs usd_path")
            if self._usd_exists(usd_path) is False:
                raise ValueError(f"prop {name}: usd not found: {usd_path}")
            self._isaac.add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            self._isaac.SingleXFormPrim(prim_path).set_world_pose(position=position)
            return

        if prop.get("children"):
            self._spawn_composite_prop(prop, name, prim_path, position)
            return

        if kind != "cube":
            raise ValueError(
                f"prop {name}: unknown type {kind!r} (cube, usd, or a cube with children)"
            )

        kwargs: Dict[str, Any] = dict(
            prim_path=prim_path,
            name=name,
            position=np.array(position),
            size=float(prop.get("size", 0.05)),
        )
        if prop.get("scale") is not None:
            kwargs["scale"] = np.array([float(v) for v in prop["scale"]])
        if prop.get("color") is not None:
            kwargs["color"] = np.array([float(v) for v in prop["color"]])
        cls = self._isaac.FixedCuboid if prop.get("fixed") else self._isaac.DynamicCuboid
        self.world.scene.add(cls(**kwargs))

    _CHILD_SHAPES = ("cube", "cylinder", "capsule", "cone", "sphere")

    def _spawn_composite_prop(
        self, prop: Dict[str, Any], name: str, prim_path: str, position: List[float]
    ) -> None:
        """A prop made of several shapes that behave as one part.

        A real part is rarely a box. A stamped shell has a formed crown, a door, a
        flag - and a defect mark is a patch on one of its faces. Both are the same
        problem: several pieces of geometry that have to move as one thing.

        So the parent carries the rigid body and the children carry only geometry and
        collision. A child with its own RigidBodyAPI would be a separate part that
        happens to start nearby, and it would fall off the moment the sim stepped.

        The parent's own `size`/`scale` are not drawn. They stay the part's bounding
        box, which is what `prop_obstacles` reports to the planner, so a composite is
        still one obstacle rather than a cloud of them.
        """
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        from .spatial import to_vec3

        stage = self._isaac.omni_usd.get_context().get_stage()
        root = UsdGeom.Xform.Define(stage, prim_path)
        root_prim = root.GetPrim()
        root.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in position]))

        fixed = bool(prop.get("fixed"))
        if not fixed:
            UsdPhysics.RigidBodyAPI.Apply(root_prim)
            mass = UsdPhysics.MassAPI.Apply(root_prim)
            mass.CreateMassAttr(float(prop.get("mass_kg", 0.4)))
            PhysxSchema.PhysxRigidBodyAPI.Apply(root_prim)

        default_color = prop.get("color")
        for index, child in enumerate(prop["children"]):
            shape = str(child.get("shape", "cube"))
            if shape not in self._CHILD_SHAPES:
                raise ValueError(
                    f"prop {name}: child {index} has unknown shape {shape!r}; "
                    f"one of {self._CHILD_SHAPES}"
                )
            child_name = _prim_name(str(child.get("name", f"part_{index}")))
            child_path = f"{prim_path}/{child_name}"
            if shape == "cube":
                geom = UsdGeom.Cube.Define(stage, child_path)
                geom.CreateSizeAttr(1.0)
                extent = 0.5
            elif shape == "sphere":
                geom = UsdGeom.Sphere.Define(stage, child_path)
                geom.CreateRadiusAttr(float(child.get("radius", 0.05)))
                extent = float(child.get("radius", 0.05))
            else:
                define = {"cylinder": UsdGeom.Cylinder, "capsule": UsdGeom.Capsule,
                          "cone": UsdGeom.Cone}[shape]
                geom = define.Define(stage, child_path)
                geom.CreateRadiusAttr(float(child.get("radius", 0.05)))
                geom.CreateHeightAttr(float(child.get("height", 0.1)))
                geom.CreateAxisAttr(str(child.get("axis", "Z")))
                extent = max(float(child.get("radius", 0.05)),
                             float(child.get("height", 0.1)) / 2.0)
            # An implicit prim keeps its default extent unless told otherwise, which
            # leaves bounds - and anything computed from them - describing a unit shape.
            geom.CreateExtentAttr([Gf.Vec3f(-extent, -extent, -extent),
                                   Gf.Vec3f(extent, extent, extent)])

            xform = UsdGeom.Xformable(geom.GetPrim())
            xform.AddTranslateOp().Set(Gf.Vec3d(*to_vec3(child.get("position"))))
            rotation = child.get("rotation_xyz_deg")
            if rotation:
                xform.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in rotation]))
            if shape == "cube":
                scale = child.get("scale") or (child.get("size", 0.05),) * 3
                xform.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in scale]))

            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            colour = child.get("color", default_color)
            if colour is not None:
                geom.CreateDisplayColorAttr(
                    [Gf.Vec3f(*[float(v) for v in colour])])

        LOGGER.info("prop %s: composite of %d shapes, %s",
                    name, len(prop["children"]), "fixed" if fixed else "dynamic")

    def _require_booted(self) -> None:
        if not self._booted.is_set():
            raise RuntimeError(
                "isaac sim world is not running - configure a "
                "erh:isaac-sim:world component and depend on it"
            )
        if self._boot_error is not None:
            raise RuntimeError(f"isaac sim failed to boot: {self._boot_error}")

    # ------------------------------------------------------------------
    # world controls (used by the world component's DoCommand)
    # ------------------------------------------------------------------

    def play(self) -> None:
        self._require_booted()
        if not self.mock:
            self.run(lambda: self.world.play())

    def pause(self) -> None:
        self._require_booted()
        if not self.mock:
            self.run(lambda: self.world.pause())

    def reset(self) -> None:
        self._require_booted()
        if not self.mock:
            self.run(lambda: self.world.reset())

    def _add_ground(self, cfg: "SimConfig") -> None:
        """Author whatever floor `ground_plan` decided on, before props spawn."""
        kind, kwargs = ground_plan(cfg.ground, cfg.usd_stage)
        if kind == "skip":
            LOGGER.warning("ground config ignored: %s", kwargs["reason"])
            return
        if kind == "none":
            return
        if kind == "plane":
            import numpy as np

            # PreviewSurface calls color.tolist(), so the colour has to be an array
            kwargs["color"] = np.array(kwargs["color"], dtype=float)
            try:
                self.world.scene.add_ground_plane(
                    name=GROUND_PLANE_NAME,
                    prim_path=GROUND_PLANE_PRIM_PATH,
                    z_position=0.0,
                    **kwargs,
                )
                if ground_is_matte(cfg.ground):
                    self._make_ground_matte()
            except Exception:
                LOGGER.exception("could not add the ground plane; falling back to the grid")
                self.world.scene.add_default_ground_plane()
            return
        if not cfg.usd_stage:
            self.world.scene.add_default_ground_plane()

    def _make_ground_matte(self) -> None:
        """Flag the plane's prims as RTX matte objects: invisible to primary rays, still
        catching shadows. Best-effort, so a render-settings change cannot block boot."""
        try:
            import carb
            from pxr import Sdf, Usd, UsdGeom

            stage = self._isaac.omni_usd.get_context().get_stage()
            root = stage.GetPrimAtPath(GROUND_PLANE_PRIM_PATH)
            for prim in (list(Usd.PrimRange(root)) if root.IsValid() else []):
                if prim == root or prim.IsA(UsdGeom.Mesh):
                    UsdGeom.PrimvarsAPI(prim).CreatePrimvar(
                        "isMatteObject", Sdf.ValueTypeNames.Bool).Set(True)
            settings = carb.settings.get_settings()
            settings.set(MATTE_OBJECT_SETTING, True)
            settings.set(SHADOW_CATCHER_SETTING, True)
        except Exception:
            LOGGER.exception("failed to make the ground plane matte")

    def _apply_viewport_grid(self, show_grid: bool) -> None:
        """Toggle the viewport's grid overlay. Best-effort; never raises."""
        try:
            import carb

            carb.settings.get_settings().set(VIEWPORT_GRID_SETTING, show_grid)
            LOGGER.info("set %s to %s", VIEWPORT_GRID_SETTING, show_grid)
        except Exception:
            LOGGER.exception("failed to apply render.viewport_grid")

    def _apply_lighting(self, lighting: Dict[str, Any]) -> None:
        """Dome and key lights. Best-effort, so bad lighting cannot block boot.

        A dome is what stops a scene reading as objects on a void: it lights every surface
        from every direction the way a room does, and an untextured one still beats the
        single default light.
        """
        if not lighting:
            return
        try:
            from pxr import Gf, Sdf, UsdGeom, UsdLux

            stage = self._isaac.omni_usd.get_context().get_stage()
            dome = lighting.get("dome")
            if dome is not None:
                light = UsdLux.DomeLight.Define(stage, DOME_LIGHT_PRIM_PATH)
                light.CreateIntensityAttr(
                    float(dome.get("intensity", DEFAULT_DOME_INTENSITY)))
                light.CreateColorAttr(Gf.Vec3f(
                    *[float(v) for v in dome.get("color", DEFAULT_DOME_COLOR)]))
                texture = dome.get("texture")
                if texture:
                    light.CreateTextureFileAttr(Sdf.AssetPath(str(texture)))
                    light.CreateTextureFormatAttr(
                        str(dome.get("texture_format", "latlong")))
                rotation = dome.get("rotation_deg")
                if rotation is not None:
                    xformable = UsdGeom.Xformable(light)
                    # clear first, so a re-apply replaces rather than stacks
                    xformable.ClearXformOpOrder()
                    xformable.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(rotation)))
            sphere = lighting.get("sphere_intensity")
            if sphere is not None:
                prim = stage.GetPrimAtPath(SPHERE_LIGHT_PRIM_PATH)
                if prim.IsValid():
                    UsdLux.SphereLight(prim).GetIntensityAttr().Set(float(sphere))
        except Exception:
            LOGGER.exception("failed to apply scene lighting")

    def prop_obstacles(self) -> List[Dict[str, Any]]:
        """Every prop as a box the viam motion service can treat as an obstacle.

        Props are spawned into isaac and exist nowhere in viam's frame system, so the
        planner cannot see them: asked to put a flange inside the belt it plans straight
        there, and the arm drives in until the collider stalls it. Callers have to pass
        these in `world_state` on every Move.

        Reported rather than published as component geometry because `fixed` is not a
        static property of the scene. A fixed belt is an obstacle forever; a dynamic part
        is an obstacle while it sits on the belt and becomes part of the *moving arm* once
        it is grasped - the same prop, two roles, switching mid-round. Only the caller
        knows which, so it gets the facts and decides.

        Lengths are millimetres, because that is what viam's geometry messages use. A
        cube prop's edge is `size` scaled per axis, so dims are size * scale.
        """
        cfg = self.cfg
        if cfg is None:
            return []
        out: List[Dict[str, Any]] = []

        # The floor counts. boot() calls add_default_ground_plane() for any scene without
        # its own usd_stage, so there is a collider at z=0 that the planner knows nothing
        # about - and an arm on a pedestal has plenty of configurations that reach below
        # its own base. Left unreported, the planner routes an elbow into the ground and
        # the arm stalls pressing against it, which reads as a mysterious failure to
        # settle rather than as a collision.
        if not cfg.usd_stage:
            out.append({
                "label": "ground",
                "fixed": True,
                "center_mm": [0.0, 0.0, -50.0],
                "dims_mm": [20000.0, 20000.0, 100.0],
            })

        for prop in cfg.props:
            if str(prop.get("type", "cube")) != "cube":
                # A usd prop's extent is whatever the asset says; the module does not
                # know it without reading the stage, so do not guess a box for it.
                continue
            size = float(prop.get("size", 0.05))
            scale = prop.get("scale") or (1.0, 1.0, 1.0)
            scale = [float(v) for v in scale]
            position = [float(v) for v in (prop.get("position") or (0.0, 0.0, 0.0))]
            out.append({
                "label": str(prop.get("name", "")),
                "fixed": bool(prop.get("fixed", False)),
                "center_mm": [v * 1000.0 for v in position],
                "dims_mm": [size * scale[i] * 1000.0 for i in range(3)],
            })
        return out

    def prop_poses(self, names: Optional[List[str]] = None) -> Dict[str, Dict[str, Any]]:
        """Where every prop actually is right now, as the sim sees it.

        Config says where a prop was *spawned*; this says where it *is*. Those differ the
        moment anything moves - a dynamic prop settles under gravity, slides, or gets
        carried by an arm - so anything reasoning about the current scene has to ask
        rather than read the config back.

        This is what lets a vision service be checked against ground truth instead of
        against itself: the part's true pose comes from the simulator, and a detector's
        answer can be scored against it without a human labelling frames.

        Positions are millimetres, to match viam's geometry and pose messages.
        Orientation is the prim's (w,x,y,z) quaternion.
        """
        self._require_booted()
        wanted = set(names) if names else None
        cfg = self.cfg
        configured = {str(p.get("name")): p for p in (cfg.props if cfg else [])}
        chosen = [n for n in configured if wanted is None or n in wanted]

        if self.mock:
            # No stage to read, so report where the prop was put. Honest for a scene
            # where nothing moves, and it keeps callers working without isaac.
            return {
                name: {
                    "position_mm": [float(v) * 1000.0
                                    for v in (configured[name].get("position")
                                              or (0.0, 0.0, 0.0))],
                    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "live": False,
                }
                for name in chosen
            }

        def _read() -> Dict[str, Dict[str, Any]]:
            out: Dict[str, Dict[str, Any]] = {}
            for name in chosen:
                prim_path = f"/World/{_prim_name(name)}"
                # A simulated body's live pose is in the physics view, not necessarily in
                # USD: reading the xform can hand back the last value written to the
                # stage while the body has long since moved. That is not a small error -
                # it made a part look like it never left the belt while an arm carried it
                # away. Ask the rigid-body view first and only fall back to the xform.
                reader = None
                if self._isaac.SingleRigidPrim is not None:
                    try:
                        reader = self._isaac.SingleRigidPrim(prim_path)
                    except Exception:
                        reader = None
                if reader is None:
                    reader = self._isaac.SingleXFormPrim(prim_path)
                try:
                    position, quat = reader.get_world_pose()
                except Exception:
                    LOGGER.exception("could not read pose of prop %s", name)
                    continue
                out[name] = {
                    "position_mm": [float(v) * 1000.0 for v in position],
                    "orientation_wxyz": [float(v) for v in quat],
                    "live": True,
                }
            return out

        return self.run(_read)

    def prim_poses(self, paths: List[str]) -> Dict[str, Dict[str, Any]]:
        """World pose of arbitrary prims, for when config and reality disagree.

        Props are addressed by name; this takes prim paths, which is what you need when
        the question is about something the module authored rather than something the
        config named - a gripper's attachment point, an arm link, a joint. Positions are
        millimetres.
        """
        self._require_booted()
        if self.mock:
            return {}

        def _read() -> Dict[str, Dict[str, Any]]:
            out: Dict[str, Dict[str, Any]] = {}
            for path in paths:
                try:
                    position, quat = self._isaac.SingleXFormPrim(path).get_world_pose()
                except Exception as exc:  # noqa: BLE001
                    out[path] = {"error": str(exc)[:120]}
                    continue
                out[path] = {
                    "position_mm": [float(v) * 1000.0 for v in position],
                    "orientation_wxyz": [float(v) for v in quat],
                }
            return out

        return self.run(_read)

    def status(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "booted": self._booted.is_set(),
            "mock": self.mock,
            "error": str(self._boot_error) if self._boot_error else "",
        }
        if self._booted.is_set() and not self.mock:
            out["playing"] = self.run(lambda: bool(self.world.is_playing()))
            out["sim_time"] = self.run(lambda: float(self.world.current_time))
        return out

    def add_usd_reference(
        self,
        usd_path: str,
        prim_path: str,
        position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self._require_booted()
        if self.mock:
            return

        def _add():
            self._isaac.add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            prim = self._isaac.SingleXFormPrim(prim_path)
            prim.set_world_pose(position=list(position))

        self.run(_add, timeout=60.0)

    # ------------------------------------------------------------------
    # component factories
    # ------------------------------------------------------------------

    # attributes that only affect the viam-side model, not the spawned prim
    _RUNTIME_KEYS = frozenset(
        {"world", "move_timeout_sec", "max_linear_mps", "max_angular_rps"}
    )

    def _cached_handle(
        self, kind: str, name: str, attrs: Dict[str, Any], factory: Callable[[], Any]
    ) -> Any:
        if name in self._handles:
            old_attrs, handle = self._handles[name]
            strip = lambda a: {k: v for k, v in a.items() if k not in self._RUNTIME_KEYS}
            if strip(old_attrs) != strip(attrs):
                LOGGER.warning(
                    "%s %r: spawn config changed but the prim is already in the "
                    "stage; restart the module to apply",
                    kind,
                    name,
                )
            return handle
        handle = factory()
        self._handles[name] = (dict(attrs), handle)
        return handle

    def _usd_exists(self, path: str) -> Optional[bool]:
        """True/False if we can check, None if omni.client is unavailable."""
        client = getattr(self._isaac, "client", None)
        if client is None:
            return None
        try:
            result, _ = client.stat(path)
            return result == client.Result.OK
        except Exception:
            return None

    def _resolve_usd(self, attrs: Dict[str, Any]) -> Tuple[Optional[str], Dict[str, Any]]:
        """Return (absolute usd path or None, known-asset metadata)."""
        meta: Dict[str, Any] = {}
        usd = attrs.get("usd_path")
        asset = attrs.get("asset")
        if asset:
            if asset not in KNOWN_ASSETS:
                raise ValueError(
                    f"unknown asset {asset!r}; known: {sorted(KNOWN_ASSETS)} "
                    "(or set usd_path directly)"
                )
            meta = KNOWN_ASSETS[asset]
            if not usd:
                root = self._isaac.get_assets_root_path()
                if root is None:
                    raise RuntimeError("could not reach the isaac sim assets server")
                candidates = meta["usd"]
                for rel in candidates:
                    if self._usd_exists(root + rel) is not False:
                        usd = root + rel
                        break
                if usd is None:
                    raise ValueError(
                        f"asset {asset!r}: none of {candidates} exist under {root}; "
                        "the asset layout may have changed in this isaac release"
                    )
        # a USD reference to a missing file "succeeds" but leaves an empty
        # prim, which later fails with confusing physics-tensor errors -
        # catch it here instead
        if usd and self._usd_exists(usd) is False:
            raise ValueError(f"usd not found: {usd}")
        return usd, meta

    def create_arm(self, name: str, attrs: Dict[str, Any]) -> "ArmHandle":
        self._require_booted()
        if self.mock:
            factory = lambda: MockArmHandle(name, attrs)
        else:
            factory = lambda: self.run(
                lambda: self._create_arm_isaac(name, attrs), timeout=120.0
            )
        return self._cached_handle("arm", name, attrs, factory)

    def _create_arm_isaac(self, name: str, attrs: Dict[str, Any]) -> "IsaacArmHandle":
        from .spatial import quat_mul, to_vec3

        usd, meta = self._resolve_usd(attrs)
        prim_path = attrs.get("prim_path") or f"/World/{_prim_name(name)}"
        if usd:
            self._isaac.add_reference_to_stage(usd_path=usd, prim_path=prim_path)

        # Set the base pose directly on the USD prim. The `position=` /
        # `orientation=` kwargs on SingleArticulation don't persist across the
        # world.reset() below when another articulation is already in the
        # scene, so every arm silently ends up at the world origin and their
        # bodies penetrate. Writing to USD persists across resets.
        position = to_vec3(attrs.get("position"))
        # The frame config orients the arm's *kinematic base*, which for some assets is
        # not the USD's root prim (see _UR_BASE_ROTATION_WXYZ). Fold the asset's fixed
        # offset in here, on the isaac side only: the frame system must keep describing
        # the base the kinematics file describes, or the planner and the simulation stop
        # agreeing about which way the arm faces.
        orientation = attrs.get("orientation_wxyz")
        orientation = (
            tuple(float(v) for v in orientation) if orientation is not None
            else (1.0, 0.0, 0.0, 0.0)
        )
        base_rotation = meta.get("base_rotation_wxyz")
        if attrs.get("ignore_base_rotation"):
            base_rotation = None
        if base_rotation is not None:
            orientation = quat_mul(orientation, tuple(float(v) for v in base_rotation))

        # Write the pose onto the USD prim rather than passing it to
        # SingleArticulation: those kwargs do not survive the world.reset() below once
        # another articulation is in the scene, so every arm silently lands on the origin
        # and the bodies interpenetrate. USD state persists across resets.
        pose_kwargs: Dict[str, Any] = {"position": list(position)}
        if orientation != (1.0, 0.0, 0.0, 0.0):
            pose_kwargs["orientation"] = list(orientation)
        try:
            self._isaac.SingleXFormPrim(prim_path).set_world_pose(**pose_kwargs)
        except Exception:
            LOGGER.exception("failed to set base pose for %s", name)

        art = self._isaac.SingleArticulation(prim_path=prim_path, name=name)
        self.world.scene.add(art)
        self.world.reset()

        ee = None
        ee_path = attrs.get("end_effector_prim")
        if ee_path:
            ee = self._isaac.SingleXFormPrim(ee_path)
        return IsaacArmHandle(self, art, ee)

    def create_gripper(self, name: str, attrs: Dict[str, Any]) -> "GripperHandle":
        self._require_booted()
        if self.mock:
            factory = lambda: MockGripperHandle(name, attrs)  # noqa: E731
        else:
            factory = lambda: self.run(  # noqa: E731
                lambda: self._create_gripper_isaac(name, attrs), timeout=120.0
            )
        return self._cached_handle("gripper", name, attrs, factory)

    def _create_gripper_isaac(self, name: str, attrs: Dict[str, Any]) -> "IsaacGripperHandle":
        """Author the suction rig through the shared surface_gripper module.

        The authoring itself lives in `surface_gripper.py`, vendored from
        DTCurrie/viam-isaac-sim so the two trees carry one implementation rather than two
        to reconcile. This method's job is only to translate *this* module's config into
        the frame that module expects.

        That translation is the whole subtlety. It wants a tool frame whose **+Z is the
        cup axis**; our `offset` is in the parent prim's frame, where isaac's UR flange
        puts the tool along **+x**. Same cup, two conventions - getting it wrong aims the
        plugin's raycast sideways, and it then grips only when geometry happens to line
        up, which is exactly the intermittency this replaced.
        """
        import math

        from .asset_catalog import CUP_APPROACH_GAP_MM
        from .spatial import _cross, _dot, _norm, quat_from_axis_angle, quat_mul
        from .surface_gripper import GripperLimits, author_attachment_rig

        stage = self._isaac.omni_usd.get_context().get_stage()
        parent = attrs.get("parent_prim")
        if not parent:
            raise ValueError(
                f"gripper {name}: needs parent_prim, the prim it hangs off "
                "(e.g. an arm's flange)"
            )
        parent_prim = stage.GetPrimAtPath(parent)
        if not parent_prim.IsValid():
            raise ValueError(f"gripper {name}: parent_prim {parent!r} is not in the stage")

        from pxr import UsdGeom, UsdPhysics

        # The joints must hang from a rigid body, and the rig's scope must sit OUTSIDE
        # the articulation - a rigid body nested under a link is an error. So walk up to
        # the body for the anchor, and put the scope at world level.
        body_prim = parent_prim
        while body_prim.IsValid() and not body_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body_prim = body_prim.GetParent()
        if not body_prim.IsValid():
            raise ValueError(
                f"gripper {name}: no rigid body at or above {parent!r}; suction needs a "
                "body to pull against"
            )
        body_path = body_prim.GetPath().pathString

        cache = UsdGeom.XformCache()
        to_body = (cache.GetLocalToWorldTransform(parent_prim)
                   * cache.GetLocalToWorldTransform(body_prim).GetInverse())
        offset = [float(v) for v in (attrs.get("offset") or (0.0, 0.0, 0.0))]
        from pxr import Gf as _Gf

        tip = to_body.Transform(_Gf.Vec3d(*offset))
        tool_pos = (float(tip[0]), float(tip[1]), float(tip[2]))

        # Where the cup points, in the body's frame: the offset direction carried through
        # the same transform. Then the rotation that takes +Z onto it.
        direction = to_body.TransformDir(_Gf.Vec3d(*offset))
        direction = (float(direction[0]), float(direction[1]), float(direction[2]))
        length = _norm(direction)
        if length < 1e-9:
            raise ValueError(
                f"gripper {name}: offset is zero, so the cup has no axis to point along"
            )
        direction = tuple(v / length for v in direction)
        z_axis = (0.0, 0.0, 1.0)
        dot = max(-1.0, min(1.0, _dot(z_axis, direction)))
        if dot > 1.0 - 1e-9:
            tool_quat = (1.0, 0.0, 0.0, 0.0)
        elif dot < -1.0 + 1e-9:
            tool_quat = quat_from_axis_angle((1.0, 0.0, 0.0), math.pi)
        else:
            tool_quat = quat_from_axis_angle(_cross(z_axis, direction), math.acos(dot))

        position, orientation = self._isaac.SingleXFormPrim(body_path).get_world_pose()
        body_pose = (tuple(float(v) for v in position),
                     tuple(float(v) for v in orientation))

        limits = GripperLimits(
            max_grip_distance_m=float(attrs.get("max_grip_distance", 0.02)),
            # The plugin's coaxial check reads ONE physics step with no averaging window,
            # and a linear move is waypoints a couple of millimetres apart, each a step
            # into stiff drives. A real limit here fires on those spikes and drops a part
            # that is not actually slipping. Authored as 0, which turns that check off and
            # leaves shear - whose locked-axis reading works - to the plugin.
            coaxial_force_limit_n=float(attrs.get("coaxial_force_limit", 0.0)),
            shear_force_limit_n=float(attrs.get("shear_force_limit", 50.0)),
            retry_interval_s=float(attrs.get("retry_interval_sec", 0.5)),
        )

        rig = author_attachment_rig(
            self._isaac.gripper_modules,
            stage,
            scope_path=f"/World/{_prim_name(name)}_rig",
            body0_path=body_path,
            body0_world_pose=body_pose,
            tool_pose_in_body0=(tool_pos, tool_quat),
            points_tool_m=[(0.0, 0.0, 0.0)],
            clearance_offset_m=float(
                attrs.get("clearance_offset_m", CUP_APPROACH_GAP_MM / 1000.0)),
            limits=limits,
            compliance=None,  # one cup: lock every axis, which welds it to the payload
        )

        # The plugin only looks for grippers on the first physics frame after play, so a
        # rig authored after the last reset is never seen.
        self.world.reset()

        view = self._isaac.GripperView(paths=rig.gripper_path)
        LOGGER.info("gripper %s: rig %s on body %s, cup axis %s in its frame",
                    name, rig.scope_path, body_path,
                    [round(v, 3) for v in direction])
        return IsaacGripperHandle(self, view, rig.gripper_path)

    def create_camera(self, name: str, attrs: Dict[str, Any]) -> "CameraHandle":
        self._require_booted()
        if self.mock:
            factory = lambda: MockCameraHandle(name, attrs)
        else:
            factory = lambda: self.run(
                lambda: self._create_camera_isaac(name, attrs), timeout=120.0
            )
        return self._cached_handle("camera", name, attrs, factory)

    def _create_camera_isaac(self, name: str, attrs: Dict[str, Any]) -> "IsaacCameraHandle":
        import math

        from .spatial import quat_from_euler_deg, to_vec3

        parent = attrs.get("parent_prim")
        if parent:
            self._require_prim(parent)
            prim_path = f"{parent.rstrip('/')}/{_prim_name(name)}"
        else:
            prim_path = attrs.get("prim_path") or f"/World/{_prim_name(name)}"
        width = int(attrs.get("width", 640))
        height = int(attrs.get("height", 480))

        kwargs: Dict[str, Any] = dict(
            prim_path=prim_path,
            name=name,
            resolution=(width, height),
        )
        if attrs.get("position") is not None:
            kwargs["position"] = list(to_vec3(attrs.get("position")))
        if attrs.get("orientation_rpy_deg") is not None:
            r, p, y = to_vec3(attrs.get("orientation_rpy_deg"))
            kwargs["orientation"] = list(quat_from_euler_deg(r, p, y))

        cam = self._isaac.Camera(**kwargs)
        cam.initialize()

        if parent:
            # camera rides a (possibly moving) link; pose is local to it.
            # default: small standoff along the link, looking out the +Z
            # (tool) axis - 180deg about X flips the usd camera's -Z forward.
            local_pos = to_vec3(attrs.get("local_position"), default=(0.0, 0.0, 0.05))
            r, p, y = to_vec3(
                attrs.get("local_orientation_rpy_deg"), default=(180.0, 0.0, 0.0)
            )
            quat = quat_from_euler_deg(r, p, y)
            cam.set_local_pose(list(local_pos), list(quat), camera_axes="usd")
        elif attrs.get("target") is not None:
            # aim at a target point (world axes: +X forward, +Z up)
            from .spatial import look_at_quat

            position = to_vec3(attrs.get("position"), default=(3.0, 3.0, 2.5))
            quat = look_at_quat(position, to_vec3(attrs.get("target")))
            cam.set_world_pose(list(position), list(quat), camera_axes="world")
        elif attrs.get("orientation_wxyz") is not None:
            position = to_vec3(attrs.get("position"))
            quat = tuple(float(v) for v in attrs["orientation_wxyz"])
            cam.set_world_pose(list(position), list(quat), camera_axes="world")

        # newly created cameras default to a 70 degree horizontal FOV - the
        # usd default lens is ~24 degrees, which reads as "zoomed way in".
        # computed via the aperture so usd unit conventions cancel out.
        if not attrs.get("prim_path"):
            fov = float(attrs.get("fov_deg", 70.0))
            aperture = cam.get_horizontal_aperture()
            cam.set_focal_length(aperture / (2.0 * math.tan(math.radians(fov) / 2.0)))
        elif attrs.get("fov_deg"):
            fov = float(attrs["fov_deg"])
            aperture = cam.get_horizontal_aperture()
            cam.set_focal_length(aperture / (2.0 * math.tan(math.radians(fov) / 2.0)))
        return IsaacCameraHandle(self, cam)

    def _require_prim(self, prim_path: str) -> None:
        """Raise a helpful error if prim_path doesn't exist in the stage."""
        get_prim = getattr(self._isaac, "get_prim_at_path", None)
        if get_prim is None:
            return
        prim = get_prim(prim_path)
        if prim is None or not prim.IsValid():
            parent_path = prim_path.rsplit("/", 1)[0] or "/"
            hint = ""
            parent = get_prim(parent_path)
            if parent is not None and parent.IsValid():
                children = [c.GetName() for c in parent.GetChildren()]
                hint = f"; children of {parent_path}: {children}"
            raise ValueError(f"prim not found: {prim_path}{hint}")

    def create_base(self, name: str, attrs: Dict[str, Any]) -> "BaseHandle":
        self._require_booted()
        if self.mock:
            factory = lambda: MockBaseHandle(name, attrs)
        else:
            factory = lambda: self.run(
                lambda: self._create_base_isaac(name, attrs), timeout=120.0
            )
        return self._cached_handle("base", name, attrs, factory)

    def _create_base_isaac(self, name: str, attrs: Dict[str, Any]) -> "IsaacBaseHandle":
        from .spatial import to_vec3

        usd, meta = self._resolve_usd(attrs)
        prim_path = attrs.get("prim_path") or f"/World/{_prim_name(name)}"
        wheel_joints = attrs.get("wheel_joints") or meta.get("wheel_joints")
        if not wheel_joints or len(wheel_joints) != 2:
            raise ValueError(
                "base needs wheel_joints: [left_joint_name, right_joint_name] "
                "(known assets like 'jetbot' provide defaults)"
            )
        wheel_radius = float(attrs.get("wheel_radius", meta.get("wheel_radius", 0.05)))
        wheel_base = float(attrs.get("wheel_base", meta.get("wheel_base", 0.3)))
        position = to_vec3(attrs.get("position"))

        base_kwargs: Dict[str, Any] = dict(
            prim_path=prim_path,
            name=name,
            wheel_dof_names=list(wheel_joints),
            create_robot=usd is not None,
            usd_path=usd,
            position=list(position),
        )
        if attrs.get("orientation_wxyz") is not None:
            base_kwargs["orientation"] = [float(v) for v in attrs["orientation_wxyz"]]
        robot = self._isaac.WheeledRobot(**base_kwargs)
        self.world.scene.add(robot)
        self.world.reset()

        controller = self._isaac.DifferentialController(
            name=f"{name}_controller",
            wheel_radius=wheel_radius,
            wheel_base=wheel_base,
        )
        handle = IsaacBaseHandle(self, robot, controller, wheel_radius, wheel_base)
        self.world.add_physics_callback(f"{name}_drive", handle._on_physics_step)
        return handle


def _local_ip() -> str:
    """Best-effort primary local IP (no traffic is actually sent)."""
    import socket

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        finally:
            s.close()
    except Exception:
        return ""


def _prim_name(name: str) -> str:
    """Component names may contain characters USD prim names can't."""
    return "".join(c if c.isalnum() or c == "_" else "_" for c in name)


def _import_isaac():
    """Import everything we need from isaac sim, tolerating the module
    renames across releases (isaacsim.* in >=4.5, omni.isaac.* before)."""

    class NS:
        pass

    ns = NS()

    try:
        from isaacsim.core.api import World
    except ImportError:
        from omni.isaac.core import World
    ns.World = World

    try:
        from isaacsim.core.utils.stage import add_reference_to_stage, open_stage
    except ImportError:
        from omni.isaac.core.utils.stage import add_reference_to_stage, open_stage
    ns.add_reference_to_stage = add_reference_to_stage
    ns.open_stage = open_stage

    try:
        from isaacsim.storage.native import get_assets_root_path
    except ImportError:
        from omni.isaac.core.utils.nucleus import get_assets_root_path
    ns.get_assets_root_path = get_assets_root_path

    try:
        from isaacsim.core.prims import SingleRigidPrim
        ns.SingleRigidPrim = SingleRigidPrim
    except ImportError:
        ns.SingleRigidPrim = None

    try:
        from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
    except ImportError:
        from omni.isaac.core.articulations import Articulation as SingleArticulation
        from omni.isaac.core.prims import XFormPrim as SingleXFormPrim
    ns.SingleArticulation = SingleArticulation
    ns.SingleXFormPrim = SingleXFormPrim

    try:
        from isaacsim.core.utils.types import ArticulationAction
    except ImportError:
        from omni.isaac.core.utils.types import ArticulationAction
    ns.ArticulationAction = ArticulationAction

    # Surface gripper: isaac's suction primitive. It fabricates a joint between the
    # gripper and whatever is within reach of its attachment points, which is exactly
    # what a vacuum tool does, and unlike a kinematic re-parent it can drop a part and
    # can fail to pick one up.
    try:
        import omni.usd

        from isaacsim.robot.surface_gripper import GripperView
        from usd.schema.isaac import robot_schema
        ns.GripperView = GripperView
        ns.robot_schema = robot_schema
        ns.omni_usd = omni.usd
        from . import compat as _compat
        ns.gripper_modules = _compat.import_surface_gripper()
    except ImportError:
        ns.GripperView = None
        ns.robot_schema = None
        ns.omni_usd = None
        ns.gripper_modules = None

    try:
        import omni.client
        ns.client = omni.client
    except ImportError:
        ns.client = None

    try:
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    except ImportError:
        from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
    ns.DynamicCuboid = DynamicCuboid
    ns.FixedCuboid = FixedCuboid

    try:
        from isaacsim.core.utils.prims import get_prim_at_path
    except ImportError:
        try:
            from omni.isaac.core.utils.prims import get_prim_at_path
        except ImportError:
            get_prim_at_path = None
    ns.get_prim_at_path = get_prim_at_path

    try:
        from isaacsim.sensors.camera import Camera
    except ImportError:
        from omni.isaac.sensor import Camera
    ns.Camera = Camera

    try:
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
            DifferentialController,
        )
    except ImportError:
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        from omni.isaac.wheeled_robots.controllers.differential_controller import (
            DifferentialController,
        )
    ns.WheeledRobot = WheeledRobot
    ns.DifferentialController = DifferentialController

    return ns


# ======================================================================
# Handles - the interface component models talk to. All public methods are
# safe to call from any thread.
# ======================================================================


class GripperHandle:
    """A suction gripper, as the module's components see it."""

    def close(self) -> None:
        raise NotImplementedError

    def open(self) -> None:
        raise NotImplementedError

    def status(self) -> str:
        """"Open", "Closing" or "Closed"."""
        raise NotImplementedError

    def gripped_objects(self) -> List[str]:
        raise NotImplementedError


class IsaacGripperHandle(GripperHandle):
    """Drives isaac's surface gripper.

    The suction is real physics, not a re-parent: isaac fabricates a joint to whatever
    body is within the attachment point's reach when the gripper closes. So a grab can
    fail because nothing was close enough, and a held part can be pulled off by its own
    weight or by driving it into something - which is the behaviour a vacuum cup has and
    a kinematic attach does not.
    """

    # The action value's sign is what matters, not its magnitude.
    _CLOSE = 0.5
    _OPEN = -0.5

    def __init__(self, sim: "SimManager", view: Any, prim_path: str) -> None:
        self._sim = sim
        self._view = view
        self._prim_path = prim_path

    def close(self) -> None:
        self._sim.run(lambda: self._view.apply_gripper_action([self._CLOSE]))

    def open(self) -> None:
        self._sim.run(lambda: self._view.apply_gripper_action([self._OPEN]))

    # isaac returns these as numbers, not the "Open"/"Closing"/"Closed" strings its
    # docstring advertises - measured: "0" open, "2" closed. Map both spellings so a
    # future build that starts returning words does not silently read as open.
    _STATUS = {"0": "Open", "1": "Closing", "2": "Closed",
               "Open": "Open", "Closing": "Closing", "Closed": "Closed"}

    def status(self) -> str:
        def _status():
            values = self._view.get_surface_gripper_status()
            if not len(values):
                return "Open"
            return self._STATUS.get(str(values[0]).strip(), str(values[0]))

        return self._sim.run(_status)

    def gripped_objects(self) -> List[str]:
        def _objects():
            got = self._view.get_gripped_objects()
            if not len(got):
                return []
            first = got[0]
            # One gripper, so one entry - which is itself a list of prim paths.
            return [str(p) for p in (first if isinstance(first, (list, tuple)) else [first]) if p]

        return self._sim.run(_objects)


class MockGripperHandle(GripperHandle):
    """Tracks open/closed with no physics, so the module runs without isaac.

    It reports nothing gripped, deliberately. A mock that always claimed success would
    let a round pass in mock and fail the moment it met a simulator.
    """

    def __init__(self, name: str, attrs: Dict[str, Any]) -> None:
        self.name = name
        self._closed = False

    def close(self) -> None:
        self._closed = True

    def open(self) -> None:
        self._closed = False

    def status(self) -> str:
        return "Closed" if self._closed else "Open"

    def gripped_objects(self) -> List[str]:
        return []


class ArmHandle:
    def get_joint_positions(self) -> List[float]:  # radians
        raise NotImplementedError

    def set_joint_targets(self, positions: List[float]) -> None:
        raise NotImplementedError

    def is_moving(self) -> bool:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def get_end_pose(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        """((x,y,z) meters, (w,x,y,z) quaternion) of the end effector."""
        raise NotImplementedError


class IsaacArmHandle(ArmHandle):
    def __init__(self, sim: SimManager, articulation, ee_prim) -> None:
        self._sim = sim
        self._art = articulation
        self._ee = ee_prim

    def get_joint_positions(self) -> List[float]:
        return self._sim.run(lambda: [float(v) for v in self._art.get_joint_positions()])

    def set_joint_targets(self, positions: List[float]) -> None:
        import numpy as np

        def _apply():
            action = self._sim._isaac.ArticulationAction(
                joint_positions=np.array(positions, dtype=float)
            )
            self._art.apply_action(action)

        self._sim.run(_apply)

    def is_moving(self) -> bool:
        def _check():
            vels = self._art.get_joint_velocities()
            if vels is None:
                return False
            return bool(max(abs(float(v)) for v in vels) > 1e-2)

        return self._sim.run(_check)

    def stop(self) -> None:
        # hold the current position
        current = self.get_joint_positions()
        self.set_joint_targets(current)

    def get_end_pose(self):
        if self._ee is None:
            raise NotImplementedError(
                "set end_effector_prim in the arm config to report end position"
            )

        def _pose():
            pos, quat = self._ee.get_world_pose()
            return (
                (float(pos[0]), float(pos[1]), float(pos[2])),
                (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
            )

        return self._sim.run(_pose)


class MockArmHandle(ArmHandle):
    """Joints move linearly toward their targets at a fixed speed."""

    SPEED = 1.0  # rad/s per joint

    def __init__(self, name: str, attrs: Dict[str, Any]) -> None:
        self.name = name
        dof = int(attrs.get("mock_dof", 6))
        self._lock = threading.Lock()
        self._start = [0.0] * dof
        self._target = [0.0] * dof
        self._t0 = time.monotonic()

    def _positions_at(self, now: float) -> List[float]:
        out = []
        dt = max(0.0, now - self._t0)
        for s, t in zip(self._start, self._target):
            delta = t - s
            travel = self.SPEED * dt
            if abs(delta) <= travel:
                out.append(t)
            else:
                out.append(s + math.copysign(travel, delta))
        return out

    def get_joint_positions(self) -> List[float]:
        with self._lock:
            return self._positions_at(time.monotonic())

    def set_joint_targets(self, positions: List[float]) -> None:
        with self._lock:
            now = time.monotonic()
            self._start = self._positions_at(now)
            if len(positions) != len(self._start):
                raise ValueError(
                    f"expected {len(self._start)} joint positions, got {len(positions)}"
                )
            self._target = list(positions)
            self._t0 = now

    def is_moving(self) -> bool:
        with self._lock:
            pos = self._positions_at(time.monotonic())
            return any(abs(p - t) > 1e-9 for p, t in zip(pos, self._target))

    def stop(self) -> None:
        with self._lock:
            now = time.monotonic()
            self._start = self._positions_at(now)
            self._target = list(self._start)
            self._t0 = now

    def get_end_pose(self):
        # a fixed, deterministic pose for testing
        return ((0.3, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0))


class CameraHandle:
    def get_rgb(self):
        """Return an (H, W, 3) uint8 numpy array."""
        raise NotImplementedError


class IsaacCameraHandle(CameraHandle):
    def __init__(self, sim: SimManager, camera) -> None:
        self._sim = sim
        self._cam = camera

    def get_rgb(self):
        def _grab():
            frame = self._cam.get_rgba()
            if frame is None or frame.size == 0:
                raise RuntimeError(
                    "no frame available yet - is the simulation playing?"
                )
            return frame[:, :, :3].copy()

        return self._sim.run(_grab)


class MockCameraHandle(CameraHandle):
    def __init__(self, name: str, attrs: Dict[str, Any]) -> None:
        self.name = name
        self._w = int(attrs.get("width", 640))
        self._h = int(attrs.get("height", 480))

    def get_rgb(self):
        import numpy as np

        # gradient background with a time-based moving bar so images change
        img = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        img[:, :, 0] = np.linspace(0, 255, self._w, dtype=np.uint8)[None, :]
        img[:, :, 1] = np.linspace(0, 255, self._h, dtype=np.uint8)[:, None]
        x = int((time.monotonic() * 60) % self._w)
        img[:, max(0, x - 5) : x + 5, :] = 255
        return img


class BaseHandle:
    def set_velocity(self, linear_mps: float, angular_rps: float) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def is_moving(self) -> bool:
        raise NotImplementedError


class IsaacBaseHandle(BaseHandle):
    def __init__(self, sim: SimManager, robot, controller, wheel_radius: float, wheel_base: float) -> None:
        self._sim = sim
        self._robot = robot
        self._controller = controller
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self._cmd = (0.0, 0.0)
        self._lock = threading.Lock()

    def _on_physics_step(self, step_size: float) -> None:
        # runs on the sim thread every physics step
        with self._lock:
            lin, ang = self._cmd
        try:
            self._robot.apply_wheel_actions(self._controller.forward(command=[lin, ang]))
        except Exception:
            LOGGER.exception("error driving base")

    def set_velocity(self, linear_mps: float, angular_rps: float) -> None:
        with self._lock:
            self._cmd = (float(linear_mps), float(angular_rps))

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0)

    def is_moving(self) -> bool:
        with self._lock:
            return self._cmd != (0.0, 0.0)


class MockBaseHandle(BaseHandle):
    def __init__(self, name: str, attrs: Dict[str, Any]) -> None:
        self.name = name
        self.wheel_radius = float(attrs.get("wheel_radius", 0.05))
        self.wheel_base = float(attrs.get("wheel_base", 0.3))
        self._cmd = (0.0, 0.0)
        self._lock = threading.Lock()

    def set_velocity(self, linear_mps: float, angular_rps: float) -> None:
        with self._lock:
            self._cmd = (float(linear_mps), float(angular_rps))

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0)

    def is_moving(self) -> bool:
        with self._lock:
            return self._cmd != (0.0, 0.0)
