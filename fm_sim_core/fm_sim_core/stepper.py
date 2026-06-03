"""MuJoCo stepping logic, kept free of ROS so it unit-tests without a node.

``MujocoStepper`` owns the model and advances it one step at a time, returning a
plain ``JointSample``. The node (``sim_loop``) handles ROS comms only; this class
handles physics only. The ``mujoco`` module is injected (defaulting to a lazy
import) so tests can pass a fake and run without the native wheel installed.
"""

from __future__ import annotations

from dataclasses import dataclass

# A minimal MJCF so the stepper runs with zero assets — replace via model_path.
_DEFAULT_MJCF = """
<mujoco>
  <worldbody>
    <body name="link" pos="0 0 0">
      <joint name="joint0" type="hinge" axis="0 0 1"/>
      <geom type="capsule" size="0.02 0.1"/>
    </body>
  </worldbody>
</mujoco>
"""


@dataclass
class JointSample:
    """One physics step's joint state — the data a JointState message needs."""

    names: list[str]
    positions: list[float]
    velocities: list[float]


class MujocoStepper:
    """Load a MuJoCo model and advance it one step per ``step()`` call.

    Args:
        model_path: Path to an MJCF file. Empty string loads the built-in 1-DOF
            fallback model.
        mujoco: The ``mujoco`` module, injected for testing. ``None`` lazily
            imports the real one, so the package loads without mujoco installed.
    """

    def __init__(self, model_path: str = "", *, mujoco=None) -> None:
        if mujoco is None:
            import mujoco  # lazy: package imports without the native wheel
        self._mj = mujoco

        if model_path:
            self.model = mujoco.MjModel.from_xml_path(model_path)
        else:
            self.model = mujoco.MjModel.from_xml_string(_DEFAULT_MJCF)
        self.data = mujoco.MjData(self.model)

        self.joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]

    @property
    def njoints(self) -> int:
        """Number of joints in the loaded model."""
        return self.model.njnt

    def step(self) -> JointSample:
        """Advance the model one step and return the resulting joint state."""
        self._mj.mj_step(self.model, self.data)
        return JointSample(
            names=list(self.joint_names),
            positions=list(self.data.qpos),
            velocities=list(self.data.qvel),
        )
