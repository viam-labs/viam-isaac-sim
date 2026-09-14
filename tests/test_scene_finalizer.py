import threading
from concurrent.futures import Future
from types import SimpleNamespace

from grpclib.const import Status
from grpclib.exceptions import GRPCError
import pytest
from viam.proto.app.robot import ComponentConfig
from viam.utils import dict_to_struct

import isaac_module.models.scene_finalizer as finalizer_model
from isaac_module.models.scene_finalizer import IsaacSceneFinalizer
from isaac_module.models.world import IsaacWorld
from isaac_module.sim_manager import IsaacBaseHandle, SimConfig, SimManager


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


def test_manager_becomes_ready_after_three_completed_steps():
    permits = threading.Semaphore(0)
    second_step = threading.Event()

    class World:
        def __init__(self):
            self.steps = 0

        def step(self, *, render):
            assert render is True
            assert permits.acquire(timeout=1)
            self.steps += 1
            if self.steps == 2:
                second_step.set()

    manager = SimManager()
    world = World()
    manager.cfg = SimConfig(wait_for_finalizer=True)
    manager._scene_finalized.clear()
    manager._ready.clear()
    manager._boot = lambda: setattr(manager, "world", world)
    manager._boot_requested.set()

    thread = threading.Thread(target=manager.main_loop, daemon=True)
    thread.start()
    try:
        manager.finalize_scene()
        permits.release()
        permits.release()
        assert second_step.wait(timeout=1)
        assert not manager._ready.is_set()

        permits.release()
        assert manager._ready.wait(timeout=1)
    finally:
        manager.request_stop()
        permits.release()
        thread.join(timeout=1)
        assert not thread.is_alive()


def test_run_rejects_operations_during_initialization():
    manager = SimManager()
    manager._ready.clear()
    manager._sim_thread_id = threading.get_ident()

    with pytest.raises(GRPCError) as error:
        manager.run(lambda: "operation")

    assert error.value.status is Status.UNAVAILABLE
    assert "initializing" in error.value.message
    assert manager._tasks.empty()
    assert manager.run(lambda: "setup", allow_during_initialization=True) == "setup"


def test_base_motion_is_rejected_but_stop_is_allowed_during_initialization():
    manager = SimManager()
    manager._ready.clear()
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
    base._cmd = (1.0, 0.5)
    base.stop()
    assert not base.is_moving()
