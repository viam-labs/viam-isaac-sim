from types import SimpleNamespace

import numpy as np
import pytest
from viam.proto.app.robot import ComponentConfig
from viam.utils import dict_to_struct

import isaac_module.models.camera as camera_model
import isaac_module.sim_manager as sim_manager
from isaac_module.models.camera import IsaacCamera
from isaac_module.sim_manager import CameraFrameUnavailable


def _config(attrs: dict) -> ComponentConfig:
    return ComponentConfig(name="camera", attributes=dict_to_struct(attrs))


def test_camera_resource_does_not_wait_for_a_frame(monkeypatch):
    handle = object()
    manager = SimpleNamespace(create_camera=lambda name, attrs: handle)
    monkeypatch.setattr(camera_model.SimManager, "get", lambda: manager)

    camera = IsaacCamera("camera")
    camera.reconfigure(_config({"world": "sim-world"}), {})

    assert camera._h() is handle


def test_camera_read_retries_transient_unavailable_frames(monkeypatch):
    expected_rgba = np.arange(12, dtype=np.uint8).reshape(1, 1, 12)[:, :, :4]

    class Sim:
        def run(self, fn, *, operation):
            assert operation == "read camera RGB"
            return fn()

    class Camera:
        def __init__(self):
            self.frames = [None, SimpleNamespace(ndim=2, shape=(1, 3), size=3), expected_rgba]

        def get_rgba(self):
            return self.frames.pop(0)

    monkeypatch.setattr(sim_manager.time, "sleep", lambda _: None)
    camera = Camera()
    frame = sim_manager.IsaacCameraHandle(Sim(), camera).get_rgb()

    assert np.array_equal(frame, expected_rgba[:, :, :3])
    assert camera.frames == []


def test_camera_read_fails_after_unavailable_frame_deadline(monkeypatch):
    class Sim:
        def run(self, fn, *, operation):
            return fn()

    class Camera:
        def get_rgba(self):
            return None

    monkeypatch.setattr(sim_manager, "_CAMERA_FRAME_TIMEOUT_SEC", 0.0)
    handle = sim_manager.IsaacCameraHandle(Sim(), Camera())
    with pytest.raises(CameraFrameUnavailable, match="no valid camera frame"):
        handle.get_rgb()
