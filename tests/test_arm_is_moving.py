from isaac_module.sim_manager import IsaacArmHandle


class _FakeArt:
    def __init__(self, vels):
        self._vels = vels

    def get_joint_velocities(self):
        return self._vels


class _FakeSim:
    def run(self, fn, timeout=None, **kwargs):
        return fn()


def test_settled_arm_is_not_moving():
    # residual velocities Isaac's PD drives produce while holding pose
    handle = IsaacArmHandle(_FakeSim(), _FakeArt([5e-3, -2e-3, 1e-3, 0, 0, 0]), None)
    assert handle.is_moving() is False


def test_actively_moving_arm_reports_moving():
    handle = IsaacArmHandle(_FakeSim(), _FakeArt([0.2, 0, 0, 0, 0, 0]), None)
    assert handle.is_moving() is True
