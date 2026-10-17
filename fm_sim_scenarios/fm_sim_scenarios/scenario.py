"""Backend-neutral scenario logic: seeded spawns and the success predicate.

A scenario is a catalog entry (``fm_sim.scenarios``) already checked against
``fm_sim_bringup/schema/catalog.schema.json``. Nothing here imports ROS or an
engine, so every backend's generator and the scenario node share one source of
truth for where objects spawn and when an episode ends.
"""

import math
import random
import re

# Spawn offsets are drawn in this order, object by object, so one seed always gives
# the same poses whichever backend asks.
SPAWN_KEYS = ('x', 'y', 'z', 'yaw')
SEED_MAX = 2 ** 32 - 1
# Names that become MJCF bodies, files, markers or frames (the schema's frame_name).
# Use fullmatch: with match, '$' also accepts a trailing newline.
FRAME_NAME = re.compile(r'[A-Za-z][A-Za-z0-9_]*')


class ScenarioError(ValueError):
    """A scenario whose parts do not fit together (beyond what the schema checks)."""


def is_static(obj):
    return obj.get('static', False)


def _non_finite(value, path):
    """Yield the path of every number in ``value`` that is not finite (NaN, inf)."""
    if isinstance(value, dict):
        for key, item in value.items():
            yield from _non_finite(item, f'{path}.{key}')
    elif isinstance(value, list):
        for index, item in enumerate(value):
            yield from _non_finite(item, f'{path}[{index}]')
    elif isinstance(value, float) and not math.isfinite(value):
        yield path


def check(scenario):
    """Check what the schema cannot: finite numbers, unique names, ranges, references.

    Raises ScenarioError naming the problem.
    """
    if not FRAME_NAME.fullmatch(str(scenario.get('name', ''))):
        raise ScenarioError(f'scenario name {scenario.get("name")!r} is not a valid name')
    bad = list(_non_finite(scenario, scenario['name']))
    if bad:
        raise ScenarioError(f'numbers must be finite: {", ".join(bad)}')
    names = [obj['name'] for obj in scenario['objects']]
    names += [region['name'] for region in scenario.get('regions', [])]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ScenarioError(f'names used twice: {", ".join(duplicates)}')
    for obj in scenario['objects']:
        for key, (low, high) in obj.get('spawn', {}).items():
            if low > high:
                raise ScenarioError(
                    f'{obj["name"]}: spawn {key} range [{low}, {high}] is reversed'
                )
    success = scenario.get('success')
    if success is None:
        return
    objects = {obj['name'] for obj in scenario['objects']}
    regions = {region['name'] for region in scenario.get('regions', [])}
    if success['object'] not in objects:
        raise ScenarioError(f'success object {success["object"]!r} is not one of the objects')
    if success['region'] not in regions:
        raise ScenarioError(f'success region {success["region"]!r} is not one of the regions')


def check_seed(seed):
    """Return ``seed`` if it fits the uint32 the Reset service carries."""
    if not isinstance(seed, int) or isinstance(seed, bool) or not 0 <= seed <= SEED_MAX:
        raise ScenarioError(f'seed must be an integer in [0, {SEED_MAX}], got {seed!r}')
    return seed


def quat_from_rpy(roll, pitch, yaw):
    """Quaternion ``(x, y, z, w)`` of fixed-axis roll, pitch, yaw (ROS convention)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _rotation_from_rpy(roll, pitch, yaw):
    """Rows of R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def nominal_pose(item):
    """``(xyz, rpy)`` of an object's or region's yaml pose, rpy defaulting to zero."""
    pose = item['pose']
    return tuple(pose['xyz']), tuple(pose.get('rpy', (0.0, 0.0, 0.0)))


def spawn_poses(scenario, seed):
    """Return ``{name: (xyz, quat_xyzw)}`` for every object that is not static.

    Each object's pose is its yaml pose plus uniform offsets drawn from its
    ``spawn`` ranges with ``random.Random(seed)``: objects in yaml order, and per
    object in x, y, z, yaw order (a key with no range draws nothing), so the same
    seed always gives the same poses.
    """
    rng = random.Random(check_seed(seed))
    poses = {}
    for obj in scenario['objects']:
        if is_static(obj):
            continue
        (x, y, z), (roll, pitch, yaw) = nominal_pose(obj)
        spawn = obj.get('spawn', {})
        offset = {key: rng.uniform(*spawn[key]) if key in spawn else 0.0 for key in SPAWN_KEYS}
        poses[obj['name']] = (
            (x + offset['x'], y + offset['y'], z + offset['z']),
            quat_from_rpy(roll, pitch, yaw + offset['yaw']),
        )
    return poses


def static_poses(scenario):
    """Return ``{name: (xyz, quat_xyzw)}`` for every static object, from its yaml."""
    poses = {}
    for obj in scenario['objects']:
        if is_static(obj):
            xyz, rpy = nominal_pose(obj)
            poses[obj['name']] = (xyz, quat_from_rpy(*rpy))
    return poses


def in_region(position, region, tolerance):
    """Return True when ``position`` lies in ``region``'s box grown by ``tolerance``."""
    centre, rpy = nominal_pose(region)
    rotation = _rotation_from_rpy(*rpy)
    delta = [p - c for p, c in zip(position, centre)]
    # Into the region frame: R^T delta, column by column.
    local = [sum(rotation[row][col] * delta[row] for row in range(3)) for col in range(3)]
    return all(abs(d) <= size / 2 + tolerance for d, size in zip(local, region['size']))


class Verdict:
    """How an episode ended."""

    def __init__(self, success, reason, elapsed_s):
        self.success = success
        self.reason = reason
        self.elapsed_s = elapsed_s

    def __repr__(self):
        return (
            f'Verdict(success={self.success}, reason={self.reason!r}, '
            f'elapsed_s={self.elapsed_s})'
        )


class EpisodeMonitor:
    """Evaluate a scenario's ``success`` predicate over one episode at a time.

    Times are sim seconds. ``start`` opens an episode; ``observe`` feeds the
    object's position (or None when it has no fresh pose, which never counts as
    inside); ``tick`` checks the timeout between observations. Each returns a
    Verdict once per episode, then None until the next ``start``.
    """

    def __init__(self, scenario, timeout_s=None):
        success = scenario['success']
        if success['type'] != 'object_in_region':
            raise ScenarioError(f'unknown success type {success["type"]!r}')
        self.object = success['object']
        (self._region,) = [r for r in scenario['regions'] if r['name'] == success['region']]
        self._tolerance = success['tolerance']
        self._hold_s = success['hold_s']
        self.timeout_s = timeout_s if timeout_s else success['timeout_s']
        self._start = None
        self._inside_since = None
        self._done = False

    @property
    def running(self):
        return self._start is not None and not self._done

    def start(self, t):
        self._start = t
        self._inside_since = None
        self._done = False

    def _finish(self, success, reason, t):
        self._done = True
        return Verdict(success, reason, t - self._start)

    def observe(self, t, position):
        if not self.running:
            return None
        region = self._region['name']
        if position is not None and in_region(position, self._region, self._tolerance):
            if self._inside_since is None:
                self._inside_since = t
            if t - self._inside_since >= self._hold_s:
                return self._finish(
                    True, f'{self.object} in {region} for {self._hold_s:g} s', t
                )
        else:
            self._inside_since = None
        return self.tick(t)

    def tick(self, t):
        if not self.running or t - self._start < self.timeout_s:
            return None
        return self._finish(
            False,
            f'timeout after {self.timeout_s:g} s: {self.object} not held in '
            f'{self._region["name"]}',
            t,
        )
