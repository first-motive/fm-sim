"""The gazebo backend's wiring without gz-sim, and the process group it runs in.

The process-group test runs the shipped wrapper around a shell that, like the
``sh -c`` ros_gz_sim used, would leave its child behind: SIGINT to the wrapper
must end the whole group.
"""

import json
import os
import platform
import signal
import subprocess
import sys
import time

from fm_sim_backends import cameras, gazebo, process_group
from fm_sim_scenarios import gazebo_world
from launch.actions import ExecuteProcess
import pytest
import yaml

INTRINSICS = {
    'width': 640, 'height': 480, 'fx': 500.0, 'fy': 510.0, 'cx': 320.0, 'cy': 240.0,
    'distortion_model': 'equidistant', 'distortion': [0.1, -0.02, 0.0, 0.0],
}
CAMERAS = {'cameras': {'chest': {'frame': 'cam', 'rate_hz': 10, 'intrinsics': INTRINSICS}}}
SCENARIO = {
    'name': 'one_box', 'objects': [{
        'name': 'box', 'shape': 'box', 'size': [0.1, 0.1, 0.1], 'mass': 0.2,
        'friction': 0.5, 'color': [1.0, 0.0, 0.0, 1.0], 'pose': {'xyz': [0.4, 0.0, 0.05]},
    }],
}
URDF = """<robot name="probe">
  <link name="world"/><link name="cam"/>
  <joint name="cam_joint" type="fixed"><parent link="world"/><child link="cam"/></joint>
</robot>"""


@pytest.fixture
def nodes(monkeypatch):
    """Stand in for launch_ros Node in the gazebo and camera-relay wiring: kwargs dicts."""
    def record(**kwargs):
        return kwargs

    monkeypatch.setattr(gazebo, 'Node', record)
    monkeypatch.setattr(cameras, 'Node', record)


LOCAL = {
    'GZ_IP': '127.0.0.1',
    'GZ_PARTITION': gazebo.DEFAULT_GZ_PARTITION,
    'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
}


@pytest.fixture(autouse=True)
def no_partition(monkeypatch):
    """Run each test as a sim host does: no GZ_PARTITION of its own."""
    monkeypatch.delenv('GZ_PARTITION', raising=False)


def test_a_partition_already_set_is_kept(monkeypatch):
    monkeypatch.setenv('GZ_PARTITION', 'fm_sim_test_1')
    assert gazebo.host_local_gz_env()['GZ_PARTITION'] == 'fm_sim_test_1'
    assert gazebo.gz_environment()['GZ_PARTITION'] == 'fm_sim_test_1'


def test_server_stays_on_this_host():
    environment = gazebo.gz_environment()
    assert {k: environment[k] for k in LOCAL} == LOCAL
    assert os.environ.get('LD_LIBRARY_PATH', '') in environment['GZ_SIM_SYSTEM_PLUGIN_PATH']


def test_server_runs_gz_sim_on_the_world_in_a_process_group():
    command = gazebo.server_command('-s -r -v 3', '/w/world.sdf')
    assert command == process_group.command('gz', 'sim', '-s', '-r', '-v', '3', '/w/world.sdf')


@pytest.mark.parametrize('text', ['{}', '[["only prefix"]]', '[[1, 2]]', 'x'])
def test_mesh_roots_refuse_anything_but_prefix_directory_pairs(text):
    with pytest.raises(ValueError):
        gazebo.parse_mesh_roots(text)


def test_mesh_roots_parse():
    assert gazebo.parse_mesh_roots('') == []
    assert gazebo.parse_mesh_roots('[["assets/", "/s/a"]]') == [('assets/', '/s/a')]


def test_without_a_world_the_robot_spawns_from_its_description(nodes):
    server, spawn, bridge = gazebo.run_actions('-r', '', '<robot/>', 'arm', '', [])
    assert isinstance(server, ExecuteProcess)
    for node in (spawn, bridge):
        assert node['additional_env'] == LOCAL
    assert spawn['arguments'] == ['-name', 'arm', '-topic', 'robot_description']
    assert bridge['arguments'] == ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']


def test_a_world_gets_the_robot_half_beside_it(nodes, tmp_path):
    cameras_file = tmp_path / 'cameras.yaml'
    cameras_file.write_text(yaml.safe_dump(CAMERAS))
    run_dir = tmp_path / 'fm_sim_world_test'
    run_dir.mkdir()
    world = gazebo_world.write('/unused.xml', [0, 0, 0], SCENARIO, 0, str(run_dir))
    server, spawn, bridge, *relays = gazebo.run_actions(
        '-s -r', world, URDF, 'probe', str(cameras_file), []
    )
    assert isinstance(server, ExecuteProcess)
    robot_file = str(run_dir / 'probe.urdf')
    assert os.path.isfile(robot_file)
    assert spawn['arguments'] == ['-name', 'probe', '-file', robot_file]
    assert spawn['additional_env'] == bridge['additional_env'] == LOCAL
    assert all(r['additional_env'] == {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
               for r in relays)
    assert bridge['parameters'] == [{'config_file': str(run_dir / 'bridge.yaml')}]
    assert [r['executable'] for r in relays] == ['republish', 'camera_info']
    # Gazebo renders no distortion, so the camera_info it is answered with has none.
    (parameters,) = relays[1]['parameters']
    intrinsics = json.loads(parameters['cameras'])['chest']['intrinsics']
    assert intrinsics['distortion_model'] == 'plumb_bob'
    assert intrinsics['distortion'] == [0.0] * 5
    assert intrinsics['fx'] == 500.0


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    # A killed child orphaned to a PID 1 that never reaps (CI's container) stays a
    # zombie, which signal 0 still finds.
    try:
        with open(f'/proc/{pid}/stat') as stat:
            return stat.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


@pytest.mark.skipif(platform.system() != 'Linux', reason='the wrapper is checked on Linux')
def test_sigint_to_the_wrapper_ends_the_whole_group(tmp_path):
    child_pid = tmp_path / 'child.pid'
    # A foreground child of sh, which defers SIGINT while it waits, as ros_gz_sim's
    # did; the child records its own pid.
    child = (
        'import os, sys, time; open(sys.argv[1], "w").write(str(os.getpid())); time.sleep(300)'
    )
    script = f"{sys.executable} -c '{child}' {child_pid}; true"
    wrapper = subprocess.Popen(process_group.command('sh', '-c', script))
    deadline = time.monotonic() + 10.0
    while not child_pid.exists() or not child_pid.read_text().strip():
        assert time.monotonic() < deadline, 'the wrapped shell never started its child'
        time.sleep(0.05)
    sleeper = int(child_pid.read_text())
    wrapper.send_signal(signal.SIGINT)
    wrapper.wait(timeout=10.0)
    deadline = time.monotonic() + 5.0
    while _alive(sleeper) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not _alive(sleeper), 'the wrapped child outlived the wrapper'


@pytest.mark.skipif(platform.system() != 'Linux', reason='the wrapper is checked on Linux')
def test_a_spared_child_outlives_the_signal_the_others_get(tmp_path):
    # MuJoCo's xvfb-run spares Xvfb so the node stops first; here a 'sleep' is spared.
    spared_pid, child_pid = tmp_path / 'spared.pid', tmp_path / 'child.pid'
    child = (
        'import os, sys, time; open(sys.argv[1], "w").write(str(os.getpid())); time.sleep(300)'
    )
    script = (
        f'sleep 300 & echo $! > {spared_pid}; '
        f"{sys.executable} -c '{child}' {child_pid}; true"
    )
    wrapper = subprocess.Popen(process_group.command('sh', '-c', script, spare='sleep'))
    deadline = time.monotonic() + 10.0
    while not all(p.exists() and p.read_text().strip() for p in (spared_pid, child_pid)):
        assert time.monotonic() < deadline, 'the wrapped shell never started its children'
        time.sleep(0.05)
    spared, other = int(spared_pid.read_text()), int(child_pid.read_text())
    try:
        wrapper.send_signal(signal.SIGTERM)
        wrapper.wait(timeout=10.0)
        deadline = time.monotonic() + 5.0
        while _alive(other) and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not _alive(other), 'the signal did not reach the unspared child'
        assert _alive(spared), 'the spared child got the signal'
    finally:
        if _alive(spared):
            os.kill(spared, signal.SIGKILL)


@pytest.mark.skipif(platform.system() != 'Linux', reason='the wrapper is checked on Linux')
def test_a_child_that_hangs_on_shutdown_is_killed(tmp_path):
    # gz-sim can hang on its way out; the watchdog kills the group, even with the
    # wrapper itself SIGKILLed (as launch does last), which nothing can forward.
    child_pid = tmp_path / 'child.pid'
    child = (
        'import os, signal, sys, time; '
        '[signal.signal(s, signal.SIG_IGN) for s in (signal.SIGINT, signal.SIGTERM)]; '
        'open(sys.argv[1], "w").write(str(os.getpid())); time.sleep(300)'
    )
    wrapper = subprocess.Popen(
        process_group.command(sys.executable, '-c', child, str(child_pid))
    )
    deadline = time.monotonic() + 10.0
    while not child_pid.exists() or not child_pid.read_text().strip():
        assert time.monotonic() < deadline, 'the child never started'
        time.sleep(0.05)
    stuck = int(child_pid.read_text())
    try:
        wrapper.send_signal(signal.SIGINT)
        time.sleep(0.5)
        assert _alive(stuck), 'the child was meant to ignore SIGINT'
        wrapper.kill()
        wrapper.wait(timeout=5.0)
        deadline = time.monotonic() + process_group.KILL_AFTER_S + 5.0
        while _alive(stuck) and time.monotonic() < deadline:
            time.sleep(0.1)
        assert not _alive(stuck), 'a child that ignores shutdown outlived the wrapper'
    finally:
        if _alive(stuck):
            os.kill(stuck, signal.SIGKILL)


@pytest.mark.skipif(platform.system() != 'Linux', reason='the wrapper is checked on Linux')
def test_a_job_that_exits_on_the_signal_keeps_its_status(tmp_path):
    # The wrapper returns the job's own status, and its output closes without
    # waiting out the watchdog, which would otherwise hold launch's pipes open.
    started = tmp_path / 'started'
    child = (
        'import signal, sys, time; '
        'signal.signal(signal.SIGINT, lambda *_: sys.exit(3)); '
        'open(sys.argv[1], "w").write("1"); time.sleep(300)'
    )
    wrapper = subprocess.Popen(
        process_group.command(sys.executable, '-c', child, str(started)),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    deadline = time.monotonic() + 10.0
    while not started.exists():
        assert time.monotonic() < deadline, 'the child never started'
        time.sleep(0.05)
    wrapper.send_signal(signal.SIGINT)
    wrapper.communicate(timeout=process_group.KILL_AFTER_S - 1.0)
    assert wrapper.returncode == 3


def test_spare_must_be_a_process_name():
    with pytest.raises(ValueError, match='process name'):
        process_group.command('true', spare='x; rm -rf /')
