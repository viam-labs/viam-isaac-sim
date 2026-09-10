from types import SimpleNamespace

import pytest
from viam.proto.app.robot import ComponentConfig
from viam.utils import dict_to_struct

import isaac_module.models.camera as camera_model
import isaac_module.sim_manager as sim_manager
from isaac_module.models.camera import IsaacCamera
from isaac_module.sim_manager import CameraFrameUnavailable, CameraHandle


def _config(attrs: dict) -> ComponentConfig:
    return ComponentConfig(name="camera", attributes=dict_to_struct(attrs))


def test_camera_resource_waits_for_initial_frame(monkeypatch):
    class Handle(CameraHandle):
        def __init__(self):
            self.timeouts = []

        def wait_for_rgb_frame(self, timeout_sec):
            self.timeouts.append(timeout_sec)

    handle = Handle()
    manager = SimpleNamespace(create_camera=lambda name, attrs: handle)
    monkeypatch.setattr(camera_model.SimManager, "get", lambda: manager)

    camera = IsaacCamera("camera")
    camera.reconfigure(_config({"world": "sim-world", "ready_timeout_sec": 4}), {})

    assert handle.timeouts == [4.0]
    assert camera._h() is handle


def test_wait_for_rgb_frame_retries_unavailable_frames(monkeypatch):
    class Handle(CameraHandle):
        def __init__(self):
            self.calls = 0

        def get_rgb(self):
            self.calls += 1
            if self.calls < 3:
                raise CameraFrameUnavailable("not rendered")
            return object()

    monkeypatch.setattr(sim_manager.time, "sleep", lambda _: None)
    handle = Handle()
    handle.wait_for_rgb_frame(1.0)

    assert handle.calls == 3

def test_isaac_camera_handle_rejects_malformed_frame():
    class Sim:
        def run(self, fn, *, operation):
            return fn()

    class Camera:
        def get_rgba(self):
            return SimpleNamespace(ndim=2, shape=(1, 3), size=3)

    handle = sim_manager.IsaacCameraHandle(Sim(), Camera())
    with pytest.raises(CameraFrameUnavailable, match="no valid camera frame"):
        handle.get_rgb()


def test_camera_ready_timeout_must_be_positive():
    with pytest.raises(ValueError, match="ready_timeout_sec must be positive"):
        IsaacCamera.validate_config(_config({"world": "sim-world", "ready_timeout_sec": 0}))
