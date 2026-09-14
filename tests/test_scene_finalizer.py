import threading
from concurrent.futures import Future
from types import SimpleNamespace

import pytest
from viam.proto.app.robot import ComponentConfig
from viam.utils import dict_to_struct

import isaac_module.models.scene_finalizer as finalizer_model
from isaac_module.models.scene_finalizer import IsaacSceneFinalizer
from isaac_module.models.world import IsaacWorld
from isaac_module.sim_manager import SimConfig, SimManager


def _config(name: str, attrs: dict) -> ComponentConfig:
    return ComponentConfig(name=name, attributes=dict_to_struct(attrs))


def test_finalizer_model_signals_manager(monkeypatch):
    calls = []
    monkeypatch.setattr(
        finalizer_model.SimManager,
        "get",
        lambda: SimpleNamespace(finalize_scene=lambda: calls.append(True)),
    )

    finalizer = IsaacSceneFinalizer.new(_config("scene-ready", {}), {})

    assert finalizer.name == "scene-ready"
    assert calls == [True]


def test_world_validates_wait_for_finalizer():
    with pytest.raises(ValueError, match="wait_for_finalizer must be a boolean"):
        IsaacWorld.validate_config(_config("sim-world", {"wait_for_finalizer": "yes"}))


def test_manager_drains_setup_without_stepping_until_finalized():
    setup_completed = threading.Event()
    rendered = threading.Event()

    class World:
        def step(self, *, render):
            assert render is True
            rendered.set()

    manager = SimManager()
    manager.cfg = SimConfig(wait_for_finalizer=True)
    manager._scene_finalized.clear()
    manager._boot = lambda: setattr(manager, "world", World())
    manager._boot_requested.set()
    manager._tasks.put((setup_completed.set, Future()))

    thread = threading.Thread(target=manager.main_loop, daemon=True)
    thread.start()
    try:
        assert setup_completed.wait(timeout=1)
        assert not rendered.is_set()

        manager.finalize_scene()
        assert rendered.wait(timeout=1)
    finally:
        manager.request_stop()
        thread.join(timeout=1)
        assert not thread.is_alive()
