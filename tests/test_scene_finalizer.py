import threading
import time
from concurrent.futures import Future
from types import SimpleNamespace

from grpclib.const import Status
from grpclib.exceptions import GRPCError
import pytest
from viam.proto.app.robot import ComponentConfig
from viam.utils import dict_to_struct

import isaac_module.models.scene_finalizer as finalizer_model
import isaac_module.models.world as world_model
from isaac_module.models.scene_finalizer import IsaacSceneFinalizer
from isaac_module.models.world import IsaacWorld
from isaac_module.sim_manager import IsaacBaseHandle, SimConfig, SimManager


def _config(name: str, attrs: dict) -> ComponentConfig:
    return ComponentConfig(name=name, attributes=dict_to_struct(attrs))


def _queued_task(manager: SimManager, operation: str, fn) -> None:
    manager._tasks.put((operation, False, time.monotonic(), fn, Future()))


def test_scene_finalizer_declares_scene_populators_and_signals_manager(monkeypatch):
    finalizer = IsaacSceneFinalizer("scene-ready")
    calls = []
    monkeypatch.setattr(
        finalizer_model.SimManager,
        "get",
        lambda: SimpleNamespace(finalize_scene=calls.append),
    )

    config = _config(
        "scene-ready",
        {"world": "sim-world", "resources": ["robot", "camera"]},
    )
    assert IsaacSceneFinalizer.validate_config(config) == (
        ["sim-world", "robot", "camera"],
        [],
    )
    finalizer.reconfigure(config, {})

    assert calls == ["scene-ready"]


def test_scene_finalizer_rejects_missing_populators():
    with pytest.raises(ValueError, match="resources"):
        IsaacSceneFinalizer.validate_config(
            _config("scene-ready", {"world": "sim-world"})
        )


def test_scene_finalizer_rejects_missing_world():
    with pytest.raises(ValueError, match="world"):
        IsaacSceneFinalizer.validate_config(
            _config("scene-ready", {"resources": ["robot"]})
        )


def test_world_configures_named_scene_finalizer(monkeypatch):
    configured = []
    monkeypatch.setattr(
        world_model.SimManager,
        "get",
        lambda: SimpleNamespace(ensure_booted=configured.append),
    )

    IsaacWorld("sim-world").reconfigure(
        _config("sim-world", {"scene_finalizer": "scene-ready"}), {}
    )

    assert configured[0].scene_finalizer == "scene-ready"


def test_direct_finalizer_signal_precedes_queued_work():
    events = []
    first_started = threading.Event()
    release_first = threading.Event()
    ordinary_completed = threading.Event()
    rendered = threading.Event()

    class World:
        def __init__(self):
            self.render_count = 0

        def step(self, *, render):
            assert render is True
            self.render_count += 1
            events.append("render")
            rendered.set()

    def first_request():
        events.append("first-start")
        first_started.set()
        release_first.wait(timeout=1)
        events.append("first-end")

    manager = SimManager()
    manager.cfg = SimConfig(scene_finalizer="scene-ready")
    manager._renderer_ready.clear()
    manager._boot = lambda: setattr(manager, "world", World())
    manager._boot_requested.set()
    _queued_task(manager, "first request", first_request)
    _queued_task(
        manager,
        "ordinary request",
        lambda: (events.append("ordinary"), ordinary_completed.set()),
    )
    thread = threading.Thread(target=manager.main_loop, daemon=True)
    thread.start()
    try:
        assert first_started.wait(timeout=1)
        manager.finalize_scene("scene-ready")
        release_first.set()
        assert rendered.wait(timeout=1)
        assert ordinary_completed.wait(timeout=1)
        assert manager._renderer_ready.wait(timeout=1)
        assert events[:4] == ["first-start", "first-end", "render", "ordinary"]
    finally:
        release_first.set()
        manager.request_stop()
        thread.join(timeout=1)
        assert not thread.is_alive()


def test_run_rejects_operations_but_allows_setup_during_renderer_warmup():
    manager = SimManager()
    manager.cfg = SimConfig(scene_finalizer="scene-ready")
    manager._scene_finalized.set()
    manager._renderer_ready.clear()
    manager._sim_thread_id = threading.get_ident()

    with pytest.raises(GRPCError) as error:
        manager.run(lambda: "operation", operation="read camera RGB")

    assert error.value.status is Status.UNAVAILABLE
    assert "read camera RGB" in error.value.message
    assert manager._tasks.empty()
    assert manager.run(
        lambda: "setup",
        operation="create camera",
        allow_during_renderer_warmup=True,
    ) == "setup"
    assert manager.status() == {
        "booted": False,
        "mock": False,
        "error": "",
        "renderer_state": "warming",
        "warmup_renders_completed": 0,
        "warmup_renders_required": 3,
        "renderer_rejected_calls": 1,
    }
    manager._mark_renderer_ready()
    assert manager.run(lambda: "operation", operation="read camera RGB") == "operation"


def test_world_without_finalizer_renders_immediately():
    rendered = threading.Event()

    class World:
        def step(self, *, render):
            assert render is True
            rendered.set()

    manager = SimManager()
    manager.cfg = SimConfig()
    manager._boot = lambda: setattr(manager, "world", World())
    manager._boot_requested.set()
    thread = threading.Thread(target=manager.main_loop, daemon=True)
    thread.start()
    try:
        assert rendered.wait(timeout=1)
    finally:
        manager.request_stop()
        thread.join(timeout=1)
        assert not thread.is_alive()


def test_base_motion_is_rejected_during_warmup_but_stop_is_allowed():
    manager = SimManager()
    manager.cfg = SimConfig(scene_finalizer="scene-ready")
    manager._scene_finalized.set()
    manager._renderer_ready.clear()
    base = IsaacBaseHandle(
        manager,
        robot=None,
        controller=None,
        wheel_radius=0.1,
        wheel_base=0.2,
    )

    with pytest.raises(GRPCError) as error:
        base.set_velocity(1.0, 0.5)

    assert error.value.status is Status.UNAVAILABLE
    assert not base.is_moving()
    base._cmd = (1.0, 0.5)
    base.stop()
    assert not base.is_moving()
