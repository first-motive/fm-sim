"""Unit-test the ROS-free sim logic with a fake mujoco module.

No rclpy, no native mujoco wheel — the stepper takes an injected module, so the
core physics-to-JointSample mapping is verified anywhere pytest runs.
"""

from fm_sim_core.stepper import JointSample, MujocoStepper


class _FakeModel:
    njnt = 2


class _FakeData:
    def __init__(self):
        self.qpos = [0.1, 0.2]
        self.qvel = [1.0, 2.0]


class _FakeMujoco:
    """Minimal stand-in for the mujoco module the stepper depends on."""

    class mjtObj:
        mjOBJ_JOINT = 3

    class MjModel:
        @staticmethod
        def from_xml_string(_xml):
            return _FakeModel()

        @staticmethod
        def from_xml_path(_path):
            return _FakeModel()

    def __init__(self):
        self.steps = 0

    @staticmethod
    def MjData(_model):
        return _FakeData()

    def mj_id2name(self, _model, _objtype, i):
        return f"joint{i}"

    def mj_step(self, _model, _data):
        self.steps += 1


def test_default_model_names_joints():
    stepper = MujocoStepper(mujoco=_FakeMujoco())
    assert stepper.njoints == 2
    assert stepper.joint_names == ["joint0", "joint1"]


def test_step_returns_joint_sample():
    fake = _FakeMujoco()
    stepper = MujocoStepper(mujoco=fake)

    sample = stepper.step()

    assert isinstance(sample, JointSample)
    assert fake.steps == 1
    assert sample.names == ["joint0", "joint1"]
    assert sample.positions == [0.1, 0.2]
    assert sample.velocities == [1.0, 2.0]


def test_step_copies_arrays():
    stepper = MujocoStepper(mujoco=_FakeMujoco())
    sample = stepper.step()
    sample.positions.append(99.9)
    # Mutating the sample must not corrupt the underlying sim data.
    assert stepper.data.qpos == [0.1, 0.2]
