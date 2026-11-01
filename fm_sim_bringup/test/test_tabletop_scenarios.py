"""The tabletop scenarios: one layout, three targets, no spawn that starts wrong."""

import copy
import itertools
import math

from fm_sim_bringup import catalog
import pytest

TARGETS = {'tabletop_can': 'can', 'tabletop_block': 'block', 'tabletop_puck': 'puck'}
PAPER_TOP = 0.208
# Where the right arm reaches on the table, for an object's centre.
REACH_X = (0.20, 0.43)
REACH_Y = (-0.25, -0.03)  # Past the puck's -0.04: that spawn edge is -0.03999... in floats.
# A top-down grasp's reach at table height, for the block and puck. REACH_X's far end is the
# can's side grasp.
TOP_DOWN_MAX_X = 0.30
# Where a top-down release plans, from plan-only sweeps through the motion node in MuJoCo.
TOP_DOWN_RELEASE_MAX_X = 0.28
TOP_DOWN_RELEASE_MIN_Y = -0.38


@pytest.fixture(params=TARGETS, ids=str)
def entry(request):
    return catalog.load('scenarios', request.param)


def _objects(entry):
    return {obj['name']: obj for obj in entry['objects']}


def _free(entry):
    return [obj for obj in entry['objects'] if not obj.get('static', False)]


def _height(obj):
    return obj['size'][2] if obj['shape'] == 'box' else obj['size'][1]


def _bounding_radius(obj):
    """Return the radius of the plan-view circle the object stays inside at any yaw."""
    return math.hypot(*obj['size'][:2]) / 2 if obj['shape'] == 'box' else obj['size'][0]


def _max_jitter(obj):
    """Return the furthest the object's centre can spawn from its nominal position, in plan."""
    return math.hypot(*(max(map(abs, obj['spawn'][key])) for key in 'xy'))


def _centre_range(obj, axis):
    """Return the lowest and highest coordinate of the object's spawned centre on ``axis``."""
    nominal = obj['pose']['xyz'][{'x': 0, 'y': 1}[axis]]
    low, high = obj['spawn'][axis]
    return nominal + low, nominal + high


def _footprint(body, grow=0.0):
    """Return the plan-view bounds of a box body or region, per axis."""
    x, y = body['pose']['xyz'][:2]
    half_x, half_y = body['size'][0] / 2 + grow, body['size'][1] / 2 + grow
    return {'x': (x - half_x, x + half_x), 'y': (y - half_y, y + half_y)}


def _clear(range_a, range_b):
    return range_a[1] < range_b[0] or range_b[1] < range_a[0]


def _layout(entry):
    """Return the entry without the fields each scenario documents as its own."""
    layout = copy.deepcopy(entry)
    for key in ('name', 'description'):
        del layout[key]
    del layout['success']['object']
    del layout['regions'][0]['pose']['xyz'][2]
    return layout


def test_scenarios_share_one_layout():
    (first, *others) = [_layout(catalog.load('scenarios', name)) for name in TARGETS]
    assert all(other == first for other in others)


def test_table_matches_can_on_table(entry):
    assert _objects(entry)['table'] == _objects(catalog.load('scenarios', 'can_on_table'))['table']


def test_the_sheets_centre_is_within_a_top_down_release(entry):
    # The task releases over the sheet's centre.
    x, y = _objects(entry)['paper']['pose']['xyz'][:2]
    assert x <= TOP_DOWN_RELEASE_MAX_X and y >= TOP_DOWN_RELEASE_MIN_Y


def test_success_object_is_the_one_the_name_says(entry):
    assert entry['success']['object'] == TARGETS[entry['name']]
    assert entry['success']['object'] in _objects(entry)


def test_target_z_is_the_success_object_resting_height(entry):
    (target,) = entry['regions']
    obj = _objects(entry)[entry['success']['object']]
    assert target['pose']['xyz'][2] == pytest.approx(PAPER_TOP + _height(obj) / 2)


def test_objects_spawn_with_their_base_level_with_the_paper(entry):
    for obj in _free(entry):
        assert obj['pose']['xyz'][2] - _height(obj) / 2 == pytest.approx(PAPER_TOP), obj['name']


def test_no_two_objects_can_overlap(entry):
    for first, second in itertools.combinations(_free(entry), 2):
        distance = math.dist(first['pose']['xyz'][:2], second['pose']['xyz'][:2])
        clearance = (distance - _max_jitter(first) - _max_jitter(second)
                     - _bounding_radius(first) - _bounding_radius(second))
        assert clearance > 0, f'{first["name"]} and {second["name"]}'


def test_no_object_can_start_on_the_paper(entry):
    paper = _footprint(_objects(entry)['paper'])
    for obj in _free(entry):
        radius = _bounding_radius(obj)
        low_x, high_x = _centre_range(obj, 'x')
        low_y, high_y = _centre_range(obj, 'y')
        assert (_clear((low_x - radius, high_x + radius), paper['x'])
                or _clear((low_y - radius, high_y + radius), paper['y'])), obj['name']


def test_no_object_can_start_as_a_success(entry):
    # The predicate reads the origin, so the centre's range is what counts here.
    (target,) = entry['regions']
    grown = _footprint(target, grow=entry['success']['tolerance'])
    for obj in _free(entry):
        assert (_clear(_centre_range(obj, 'x'), grown['x'])
                or _clear(_centre_range(obj, 'y'), grown['y'])), obj['name']


def test_objects_spawn_reachable_and_on_the_table(entry):
    table = _footprint(_objects(entry)['table'])
    for obj in _free(entry):
        radius = _bounding_radius(obj)
        low_x, high_x = _centre_range(obj, 'x')
        low_y, high_y = _centre_range(obj, 'y')
        assert REACH_X[0] <= low_x and high_x <= REACH_X[1], obj['name']
        assert REACH_Y[0] <= low_y and high_y <= REACH_Y[1], obj['name']
        assert table['x'][0] < low_x - radius and high_x + radius < table['x'][1], obj['name']
        assert table['y'][0] < low_y - radius and high_y + radius < table['y'][1], obj['name']


def test_colours_are_opaque_and_clearly_apart(entry):
    objects = entry['objects']
    assert all(obj['color'][3] == 1.0 for obj in objects)
    for first, second in itertools.combinations(objects, 2):
        # 0.4 in RGB is well above what lighting shifts a flat colour by in a render.
        assert math.dist(first['color'][:3], second['color'][:3]) > 0.4, (
            f'{first["name"]} and {second["name"]}')


def test_top_down_objects_spawn_within_a_top_down_grasps_reach(entry):
    objects = _objects(entry)
    assert all(_centre_range(objects[name], 'x')[1] <= TOP_DOWN_MAX_X for name in ('block', 'puck'))


def test_the_puck_spawns_at_a_fixed_yaw(entry):
    # Guards the config only: at one drawn yaw the fingers passed through the puck in MuJoCo,
    # which only a run shows.
    assert 'yaw' not in _objects(entry)['puck']['spawn']


def test_the_sheet_is_a5(entry):
    assert _objects(entry)['paper']['size'] == [0.210, 0.148, 0.001]
