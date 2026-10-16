"""Every backend publishes the description and hands it to its controller_manager."""

import importlib.util
import os

from fm_sim_backends import description
from launch import LaunchContext
from launch.utilities import perform_substitutions
import pytest

LAUNCH_DIR = os.path.join(os.path.dirname(__file__), '..', 'launch')
BACKENDS = ('mujoco', 'gazebo', 'isaac')
# The launches that host the controller_manager in a ros2_control_node; gazebo's
# lives inside the gz_ros2_control world plugin instead.
CONTROLLER_MANAGER_PACKAGES = {
    'mujoco': 'mujoco_ros2_control',
    'isaac': 'controller_manager',
}
DESCRIPTION = '<robot name="probe"/>'


def _load(backend):
    path = os.path.join(LAUNCH_DIR, f'{backend}.launch.py')
    spec = importlib.util.spec_from_file_location(f'{backend}_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context():
    context = LaunchContext()
    context.launch_configurations['robot_description'] = DESCRIPTION
    return context


class _NodeRecorder:
    """Stand-in for launch_ros Node that keeps the keyword arguments it got."""

    def __init__(self):
        self.calls = []

    def __call__(self, **kwargs):
        self.calls.append(kwargs)
        return kwargs


def test_robot_state_publisher_node_carries_the_description(monkeypatch):
    recorder = _NodeRecorder()
    monkeypatch.setattr(description, 'Node', recorder)
    description.robot_state_publisher_node('xml')
    (call,) = recorder.calls
    assert call['package'] == 'robot_state_publisher'
    assert call['parameters'] == [{'robot_description': 'xml'}]


@pytest.mark.parametrize('backend', BACKENDS)
def test_backend_publishes_the_robot_description_argument(backend, monkeypatch):
    module = _load(backend)
    if backend == 'mujoco':
        # The engine-presence guard is for a live launch, not for reading actions.
        monkeypatch.setattr(module, '_require_backend', lambda: None)
    published = []
    monkeypatch.setattr(
        module,
        'robot_state_publisher_node',
        lambda robot_description: published.append(robot_description) or {},
    )
    monkeypatch.setattr(module, 'Node', _NodeRecorder())
    module.generate_launch_description()
    (robot_description,) = published
    assert robot_description.evaluate(_context()) == DESCRIPTION


@pytest.mark.parametrize('backend', sorted(CONTROLLER_MANAGER_PACKAGES))
def test_controller_manager_keeps_the_description_parameter(backend, monkeypatch):
    # Jazzy ignores it; fm-app's Humble fm_bringup still relies on it.
    module = _load(backend)
    if backend == 'mujoco':
        monkeypatch.setattr(module, '_require_backend', lambda: None)
    recorder = _NodeRecorder()
    monkeypatch.setattr(module, 'Node', recorder)
    module.generate_launch_description()
    (call,) = [
        c for c in recorder.calls
        if c['package'] == CONTROLLER_MANAGER_PACKAGES[backend]
    ]
    parameters, controllers_file, *extra = call['parameters']
    assert parameters['robot_description'].evaluate(_context()) == DESCRIPTION
    context = _context()
    context.launch_configurations['controllers_file'] = '/c.yaml'
    assert perform_substitutions(context, [controllers_file]) == '/c.yaml'
    if backend == 'mujoco':
        (plugins_file,) = extra
        assert plugins_file.endswith('/fm_sim_backends/config/mujoco_plugins.yaml')
        assert os.path.isfile(plugins_file)
    else:
        assert extra == []
