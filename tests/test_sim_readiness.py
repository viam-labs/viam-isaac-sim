import threading

import pytest

from isaac_module.sim_manager import SimConfig, SimManager


def _start(manager: SimManager) -> threading.Thread:
    thread = threading.Thread(target=manager.main_loop, daemon=True)
    thread.start()
    return thread


def _stop(manager: SimManager, thread: threading.Thread) -> None:
    manager.request_stop()
    thread.join(timeout=1)
    assert not thread.is_alive()


def test_world_readiness_waits_for_fast_step():
    manager = SimManager()
    thread = _start(manager)
    try:
        manager.ensure_booted(
            SimConfig(mock=True, ready_step_max=0.1, ready_step_timeout=1.0)
        )
        assert manager._fast_step_ready.is_set()
    finally:
        _stop(manager, thread)


def test_world_readiness_times_out_without_fast_step():
    manager = SimManager()
    thread = _start(manager)
    try:
        with pytest.raises(TimeoutError, match="did not complete a world step"):
            manager.ensure_booted(
                SimConfig(mock=True, ready_step_max=0.001, ready_step_timeout=0.05)
            )
        assert not manager._fast_step_ready.is_set()
    finally:
        _stop(manager, thread)
