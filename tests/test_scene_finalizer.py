import threading
import time
from concurrent.futures import Future
from types import SimpleNamespace

import pytest
from viam.proto.app.robot import ComponentConfig
from viam.utils import dict_to_struct

import isaac_module.models.scene_finalizer as finalizer_model
from isaac_module.models.scene_finalizer import IsaacSceneFinalizer
import isaac_module.models.world as world_model
from isaac_module.models.world import IsaacWorld
from isaac_module.sim_manager import SimConfig, SimManager


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

    assert IsaacSceneFinalizer.validate_config(
        _config("scene-ready", {"resources": ["robot", "camera"]})
    ) == (["robot", "camera"], [])
    finalizer.reconfigure(_config("scene-ready", {"resources": ["robot", "camera"]}), {})

    assert calls == ["scene-ready"]


def test_scene_finalizer_rejects_missing_populators():
    with pytest.raises(ValueError, match="resources"):
        IsaacSceneFinalizer.validate_config(_config("scene-ready", {}))


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


def test_finalizer_signal_precedes_first_render_even_with_queued_work():
    events = []
    ordinary_completed = threading.Event()
    rendered = threading.Event()

    class World:
        def step(self, *, render):
            assert render is True
            events.append("render")
            rendered.set()

    manager = SimManager()
    manager.cfg = SimConfig(scene_finalizer="scene-ready")
    manager._boot = lambda: setattr(manager, "world", World())
    manager._boot_requested.set()
    _queued_task(
        manager,
        "finalize scene",
        lambda: (events.append("finalize"), manager.finalize_scene("scene-ready")),
    )
    _queued_task(
        manager,
        "ordinary request",
        lambda: (events.append("ordinary"), ordinary_completed.set()),
    )
    thread = threading.Thread(target=manager.main_loop, daemon=True)
    thread.start()
    try:
        assert rendered.wait(timeout=1)
        assert ordinary_completed.wait(timeout=1)
        assert events[:3] == ["finalize", "render", "ordinary"]
    finally:
        manager.request_stop()
        thread.join(timeout=1)
        assert not thread.is_alive()


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
