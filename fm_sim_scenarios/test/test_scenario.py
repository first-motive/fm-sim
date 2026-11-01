"""Seeded spawns, scenario checks and the object_in_region predicate."""

import math

from fm_sim_scenarios import scenario as sc
import pytest


def _rpy_of(quat):
    x, y, z, w = quat
    return (
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )


def test_spawn_is_the_same_for_the_same_seed(can_on_table):
    assert sc.spawn_poses(can_on_table, 42) == sc.spawn_poses(can_on_table, 42)
    assert sc.spawn_poses(can_on_table, 42) != sc.spawn_poses(can_on_table, 43)


@pytest.mark.parametrize('seed', [0, 1, 7, 2 ** 32 - 1])
def test_spawn_stays_in_its_ranges(can_on_table, seed):
    (xyz, quat) = sc.spawn_poses(can_on_table, seed)['can']
    assert 0.35 <= xyz[0] <= 0.41
    assert -0.17 <= xyz[1] <= -0.12
    assert xyz[2] == 0.2755
    roll, pitch, yaw = _rpy_of(quat)
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9
    assert -3.14159 <= yaw <= 3.14159


def test_spawn_leaves_static_objects_and_keeps_unranged_ones(can_on_table):
    poses = sc.spawn_poses(can_on_table, 3)
    assert set(poses) == {'can', 'ball'}
    assert poses['ball'] == ((0.6, 0.2, 0.228), (0.0, 0.0, 0.0, 1.0))
    assert sc.static_poses(can_on_table) == {
        'table': ((0.5, 0.0, 0.187), (0.0, 0.0, 0.0, 1.0)),
        'paper': ((0.40, -0.30, 0.2075), (0.0, 0.0, 0.0, 1.0)),
    }


@pytest.mark.parametrize('seed', [-1, 2 ** 32, 1.5, True, '3'])
def test_seed_must_be_uint32(can_on_table, seed):
    with pytest.raises(sc.ScenarioError, match='seed'):
        sc.spawn_poses(can_on_table, seed)


def test_quat_from_rpy_matches_known_rotations():
    assert sc.quat_from_rpy(0, 0, 0) == pytest.approx((0, 0, 0, 1))
    half = math.sqrt(0.5)
    assert sc.quat_from_rpy(0, 0, math.pi / 2) == pytest.approx((0, 0, half, half))
    assert sc.quat_from_rpy(math.pi / 2, 0, 0) == pytest.approx((half, 0, 0, half))
    assert _rpy_of(sc.quat_from_rpy(0.1, -0.2, 0.3)) == pytest.approx((0.1, -0.2, 0.3))


def test_check_accepts_the_fixture(can_on_table):
    sc.check(can_on_table)


def _object(scenario, name):
    (obj,) = [obj for obj in scenario['objects'] if obj['name'] == name]
    return obj


@pytest.mark.parametrize('change, match', [
    (lambda s: s.update(name='../x'), "scenario name '../x'"),
    (lambda s: s['regions'][0].update(name='can'), 'names used twice: can'),
    (lambda s: _object(s, 'can')['spawn'].update(x=[0.1, -0.1]), 'spawn x range .* reversed'),
    (
        lambda s: _object(s, 'can')['pose']['xyz'].__setitem__(0, float('nan')),
        r'numbers must be finite: can_on_table\.objects\[2\]\.pose\.xyz\[0\]',
    ),
    (
        lambda s: s['success'].update(timeout_s=float('inf')),
        r'numbers must be finite: can_on_table\.success\.timeout_s',
    ),
    (lambda s: s['success'].update(object='cup'), "success object 'cup'"),
    (lambda s: s['success'].update(region='shelf'), "success region 'shelf'"),
])
def test_check_rejects(can_on_table, change, match):
    change(can_on_table)
    with pytest.raises(sc.ScenarioError, match=match):
        sc.check(can_on_table)


REGION = {'name': 'r', 'size': [0.2, 0.1, 0.02], 'pose': {'xyz': [1.0, 0.0, 0.5]}}


@pytest.mark.parametrize('position, tolerance, inside', [
    ((1.0, 0.0, 0.5), 0.0, True),
    ((1.0999, 0.0499, 0.5099), 0.0, True),  # just inside the corner
    ((1.11, 0.0, 0.5), 0.0, False),
    ((1.105, 0.0, 0.5), 0.01, True),
    ((1.0, 0.0, 0.53), 0.01, False),
])
def test_in_region_axis_aligned(position, tolerance, inside):
    assert sc.in_region(position, REGION, tolerance) is inside


def test_in_region_follows_the_region_yaw():
    turned = {**REGION, 'pose': {'xyz': [1.0, 0.0, 0.5], 'rpy': [0.0, 0.0, math.pi / 2]}}
    # Long side (0.2) now runs along world y.
    assert sc.in_region((1.0, 0.09, 0.5), turned, 0.0)
    assert not sc.in_region((1.09, 0.0, 0.5), turned, 0.0)


CAN_IN = (0.40, -0.30, 0.2745)
CAN_OUT = (0.38, -0.15, 0.2745)


@pytest.fixture
def hold_only(can_on_table):
    """can_on_table without the at-rest check, for the hold alone."""
    del can_on_table['success']['max_speed']
    return can_on_table


def test_success_after_hold(hold_only):
    monitor = sc.EpisodeMonitor(hold_only)
    monitor.start(10.0)
    assert monitor.observe(11.0, CAN_IN) is None
    assert monitor.observe(12.9, CAN_IN) is None
    verdict = monitor.observe(13.0, CAN_IN)
    assert verdict.success
    assert verdict.reason == 'can in target for 2 s'
    assert verdict.elapsed_s == pytest.approx(3.0)
    assert monitor.observe(14.0, CAN_IN) is None   # one verdict per episode


def test_a_held_object_lowered_through_the_region_does_not_count(can_on_table):
    # Regression: the place lowers the held can through the target at 20 mm/s.
    monitor = sc.EpisodeMonitor(can_on_table)
    monitor.start(0.0)
    for i in range(60):
        x, y, z = CAN_IN
        assert monitor.observe(i * 0.05, (x, y, z + 0.01 - 0.02 * i * 0.05)) is None


def feed(monitor, start, end, position, dt=0.05):
    """Observe ``position`` every dt from start to end; the first verdict's time, or None."""
    for i in range(round((end - start) / dt) + 1):
        if monitor.observe(start + i * dt, position) is not None:
            return start + i * dt
    return None


def test_a_settled_object_scores_at_rest(can_on_table):
    monitor = sc.EpisodeMonitor(can_on_table)
    monitor.start(0.0)
    assert monitor.observe(1.0, CAN_IN) is None    # no earlier pose: speed unknown
    monitor.observe(1.05, CAN_IN)                  # hold starts here
    assert feed(monitor, 1.1, 3.2, CAN_IN) == pytest.approx(3.05, abs=0.02)


def test_the_at_rest_reason_says_so(can_on_table):
    monitor = sc.EpisodeMonitor(can_on_table)
    monitor.start(0.0)
    verdict = None
    for i in range(50):
        verdict = verdict or monitor.observe(i * 0.05, CAN_IN)
    assert verdict.reason == 'can at rest in target for 2 s'


def test_without_max_speed_a_moving_object_still_counts(hold_only):
    monitor = sc.EpisodeMonitor(hold_only)
    monitor.start(0.0)
    x, y, z = CAN_IN
    monitor.observe(1.0, (x, y, z + 0.005))
    verdict = monitor.observe(3.0, (x, y, z - 0.005))
    assert verdict.success
    assert verdict.reason == 'can in target for 2 s'


def test_a_missing_pose_resets_the_speed(can_on_table):
    monitor = sc.EpisodeMonitor(can_on_table)
    monitor.start(0.0)
    assert feed(monitor, 1.0, 1.5, CAN_IN) is None
    monitor.observe(1.55, None)
    monitor.observe(1.6, CAN_IN)                   # speed unknown again
    assert feed(monitor, 1.65, 4.0, CAN_IN) == pytest.approx(3.65, abs=0.02)


def test_a_repeated_stamp_gives_no_speed(can_on_table):
    monitor = sc.EpisodeMonitor(can_on_table)
    monitor.start(0.0)
    monitor.observe(1.0, CAN_IN)
    monitor.observe(1.0, CAN_IN)
    assert monitor.observe(3.0, CAN_IN) is None    # gap too long: the hold never started


def test_a_move_across_a_stalled_pose_stream_is_not_at_rest(can_on_table):
    # 5 mm in 2 s reads as 2.5 mm/s, under max_speed, if the gap is trusted.
    monitor = sc.EpisodeMonitor(can_on_table)
    monitor.start(0.0)
    x, y, z = CAN_IN
    monitor.observe(1.0, (x, y, z + 0.005))
    monitor.observe(3.0, CAN_IN)                   # no speed: the hold does not start
    assert feed(monitor, 3.05, 6.0, CAN_IN) == pytest.approx(5.05, abs=0.02)


def test_leaving_the_region_restarts_the_hold(hold_only):
    monitor = sc.EpisodeMonitor(hold_only)
    monitor.start(0.0)
    monitor.observe(1.0, CAN_IN)
    monitor.observe(2.5, CAN_OUT)
    assert monitor.observe(3.0, CAN_IN) is None
    assert monitor.observe(4.5, CAN_IN) is None
    assert monitor.observe(5.0, CAN_IN).success


def test_a_missing_pose_breaks_the_hold(can_on_table):
    monitor = sc.EpisodeMonitor(can_on_table)
    monitor.start(0.0)
    monitor.observe(1.0, CAN_IN)
    assert monitor.observe(2.0, None) is None
    assert monitor.observe(3.5, CAN_IN) is None


def test_timeout_fails_once(can_on_table):
    monitor = sc.EpisodeMonitor(can_on_table, timeout_s=5.0)
    assert monitor.tick(100.0) is None             # no episode yet
    monitor.start(1.0)
    assert monitor.tick(5.9) is None
    verdict = monitor.tick(6.0)
    assert not verdict.success
    assert verdict.reason == 'timeout after 5 s: can not held in target'
    assert verdict.elapsed_s == pytest.approx(5.0)
    assert monitor.tick(7.0) is None
    assert monitor.observe(7.0, CAN_IN) is None


def test_timeout_defaults_to_the_scenario(can_on_table):
    assert sc.EpisodeMonitor(can_on_table).timeout_s == 120.0
    assert sc.EpisodeMonitor(can_on_table, timeout_s=0.0).timeout_s == 120.0


def test_start_opens_a_new_episode(can_on_table):
    monitor = sc.EpisodeMonitor(can_on_table, timeout_s=1.0)
    monitor.start(0.0)
    assert monitor.tick(1.0) is not None
    monitor.start(2.0)
    assert monitor.running
    assert monitor.tick(2.5) is None
    assert monitor.tick(3.0) is not None


def test_unknown_success_type_is_refused(can_on_table):
    can_on_table['success']['type'] = 'object_upright'
    with pytest.raises(sc.ScenarioError, match="unknown success type 'object_upright'"):
        sc.EpisodeMonitor(can_on_table)


def test_a_scenario_without_usd_objects_is_not_refused(can_on_table):
    sc.refuse_usd_objects(can_on_table, 'mujoco')


def test_a_usd_object_is_refused_naming_the_object_and_the_backend(can_on_table):
    can_on_table['objects'][-1]['usd'] = 'pkg/prop.usd'
    name = can_on_table['objects'][-1]['name']
    with pytest.raises(sc.ScenarioError, match=f"{name} are USD assets.*backend 'mock'"):
        sc.refuse_usd_objects(can_on_table, 'mock')
