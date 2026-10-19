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
        monkeypatch.setattr(module, '_require_backend', lambda renderer: None)
    published = []
    monkeypatch.setattr(
        module,
        'robot_state_publisher_node',
        lambda robot_description: published.append(robot_description) or {},
    )
    if backend != 'gazebo':
        # gazebo's nodes start from an OpaqueFunction (fm_sim_backends.gazebo).
        monkeypatch.setattr(module, 'Node', _NodeRecorder())
    else:
        assert not hasattr(module, 'Node')
    module.generate_launch_description()
    (robot_description,) = published
    assert robot_description.evaluate(_context()) == DESCRIPTION


@pytest.mark.parametrize('backend', sorted(CONTROLLER_MANAGER_PACKAGES))
def test_controller_manager_keeps_the_description_parameter(backend, monkeypatch):
    # Jazzy ignores it; fm-app's Humble fm_bringup still relies on it.
    module = _load(backend)
    if backend == 'mujoco':
        monkeypatch.setattr(module, '_require_backend', lambda renderer: None)
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


def test_mujoco_controller_manager_runs_on_sim_time(monkeypatch):
    # Its clock stamps /joint_states and so the robot's /tf; on wall time they never
    # line up with the sim-time /sim/objects (fm-skills' gt perception drops them).
    _, call = _engine_node(monkeypatch)
    (parameters,) = [p for p in call['parameters'] if isinstance(p, dict)]
    assert parameters['use_sim_time'] is True


def _engine_node(monkeypatch, system='Linux', mujoco_gl=None):
    module = _load('mujoco')
    monkeypatch.setattr(module.platform, 'system', lambda: system)
    if mujoco_gl is None:
        monkeypatch.delenv('MUJOCO_GL', raising=False)
    else:
        monkeypatch.setenv('MUJOCO_GL', mujoco_gl)
    monkeypatch.setattr(module, '_require_backend', lambda renderer: None)
    recorder = _NodeRecorder()
    monkeypatch.setattr(module, 'Node', recorder)
    module.generate_launch_description()
    (call,) = [c for c in recorder.calls if c['package'] == 'mujoco_ros2_control']
    return module, call


def test_mujoco_renders_cameras_through_egl_by_default(monkeypatch):
    _, call = _engine_node(monkeypatch)
    # No display: the CameraPlugin's GLFW init fails and it takes EGL.
    assert call['additional_env'] == {'DISPLAY': '', 'WAYLAND_DISPLAY': ''}
    assert call['prefix'] == ''


def test_mujoco_osmesa_forces_mesa_software_egl(monkeypatch):
    module, call = _engine_node(monkeypatch, mujoco_gl='osmesa')
    assert call['additional_env'] == {
        'DISPLAY': '', 'WAYLAND_DISPLAY': '',
        '__EGL_VENDOR_LIBRARY_FILENAMES': module.MESA_EGL_VENDOR, 'LIBGL_ALWAYS_SOFTWARE': '1',
    }
    assert call['prefix'] == ''


def test_mujoco_glfw_runs_under_xvfb_on_linux_only(monkeypatch):
    _, call = _engine_node(monkeypatch, mujoco_gl='glfw')
    assert call['additional_env'] == {}
    assert 'xvfb-run -a' in call['prefix']
    _, call = _engine_node(monkeypatch, system='Darwin')
    assert (call['additional_env'], call['prefix']) == ({}, '')


@pytest.mark.parametrize('system, mujoco_gl, message', [
    ('Linux', 'vulkan', "MUJOCO_GL='vulkan' is not a renderer"),
    ('Linux', '', "MUJOCO_GL='' is not a renderer"),
    ('Darwin', 'egl', 'macOS does not have'),
])
def test_mujoco_refuses_a_renderer_it_cannot_use(monkeypatch, system, mujoco_gl, message):
    with pytest.raises(RuntimeError, match=message):
        _engine_node(monkeypatch, system=system, mujoco_gl=mujoco_gl)
