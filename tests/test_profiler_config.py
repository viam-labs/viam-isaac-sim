import sys

import pytest
from viam.proto.app.robot import ComponentConfig
from viam.utils import dict_to_struct

from isaac_module.models.world import IsaacWorld
from isaac_module.sim_manager import SimManager


def _config(attrs: dict) -> ComponentConfig:
    return ComponentConfig(name="sim-world", attributes=dict_to_struct(attrs))


def test_profile_trace_path_must_be_absolute():
    with pytest.raises(ValueError, match="absolute path"):
        IsaacWorld.validate_config(_config({"profile_trace_path": "boot.gz"}))
    with pytest.raises(ValueError, match="absolute path string"):
        IsaacWorld.validate_config(_config({"profile_trace_path": True}))

    IsaacWorld.validate_config(_config({"profile_trace_path": "/tmp/boot.gz"}))


def test_world_timing_values_must_be_positive():
    for key in ("physics_dt", "rendering_dt", "boot_timeout_sec"):
        with pytest.raises(ValueError, match=f"{key} must be positive"):
            IsaacWorld.validate_config(_config({key: 0}))


def test_file_profiler_prepares_trace_and_enables_kit_flags(monkeypatch, tmp_path):
    argv = ["src/main.py"]
    monkeypatch.setattr(sys, "argv", argv)
    trace_path = tmp_path / "profiles" / "startup.gz"

    SimManager._configure_file_profiler(str(trace_path))

    assert trace_path.is_file()
    assert argv == [
        "src/main.py",
        "--/app/profilerBackend=cpu",
        "--/app/profileFromStart=1",
        "--/plugins/carb.profiler-cpu.plugin/saveProfile=1",
        "--/plugins/carb.profiler-cpu.plugin/compressProfile=1",
        f"--/plugins/carb.profiler-cpu.plugin/filePath={trace_path}",
    ]
