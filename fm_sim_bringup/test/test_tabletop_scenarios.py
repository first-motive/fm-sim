"""The tabletop scenarios: one layout, three targets, no spawn that starts wrong."""

import copy
import itertools
import math

from fm_sim_bringup import catalog
from fm_sim_scenarios import gazebo_world, isaac_stage, mujoco_world, scenario as sc
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


@pytest.fixture
def simready():
    return catalog.load('scenarios', 'tabletop_simready')


def test_simready_keeps_the_tabletop_table_and_sheet(simready):
    tabletop = _objects(catalog.load('scenarios', 'tabletop_can'))
    for name in ('table', 'paper'):
        assert _objects(simready)[name] == tabletop[name]


def test_simready_uses_the_tabletop_slots_and_spawn_boxes(simready):
    tabletop = _objects(catalog.load('scenarios', 'tabletop_can'))
    objects = _objects(simready)
    for name, slot in (('can', 'can'), ('boxed_food', 'block'), ('canned_food', 'puck')):
        assert objects[name]['pose']['xyz'][:2] == tabletop[slot]['pose']['xyz'][:2], name
        assert objects[name]['spawn'] == tabletop[slot]['spawn'], name


def test_simready_scores_perception_only(simready):
    assert 'success' not in simready and 'regions' not in simready


def test_simready_objects_are_the_chosen_packages_with_their_base_on_the_sheet(simready):
    # A SimReady asset's origin is at its base: pose z is the sheet's top, not a centre.
    packages = {
        'can': 'food_beverage_can_e01',
        'canned_food': 'food_canned_beans_a02',
        'boxed_food': 'food_box_gelatin_a01',
    }
    assert {obj['name'] for obj in _free(simready)} == set(packages)
    for obj in _free(simready):
        assert obj['usd'].startswith(f'{packages[obj["name"]]}/simready_usd/'), obj['name']
        assert obj['pose']['xyz'][2] == pytest.approx(PAPER_TOP), obj['name']


def test_simready_objects_fit_the_gripper_across_their_grasp(simready):
    # The gripper opens to 2 x 0.045 m; each object's narrowest plan width is its grasp.
    for obj in _free(simready):
        width = min(obj['size'][:2]) if obj['shape'] == 'box' else 2 * obj['size'][0]
        assert width < 0.080, obj['name']


def test_simready_objects_cannot_overlap_or_start_on_the_sheet(simready):
    for first, second in itertools.combinations(_free(simready), 2):
        distance = math.dist(first['pose']['xyz'][:2], second['pose']['xyz'][:2])
        assert distance > (_max_jitter(first) + _max_jitter(second)
                           + _bounding_radius(first) + _bounding_radius(second))
    paper = _footprint(_objects(simready)['paper'])
    for obj in _free(simready):
        radius = _bounding_radius(obj)
        low_y, high_y = _centre_range(obj, 'y')
        low_x, high_x = _centre_range(obj, 'x')
        assert (_clear((low_x - radius, high_x + radius), paper['x'])
                or _clear((low_y - radius, high_y + radius), paper['y'])), obj['name']


def test_simready_objects_spawn_reachable_and_on_the_table(simready):
    table = _footprint(_objects(simready)['table'])
    for obj in _free(simready):
        radius = _bounding_radius(obj)
        low_x, high_x = _centre_range(obj, 'x')
        low_y, high_y = _centre_range(obj, 'y')
        assert REACH_X[0] <= low_x and high_x <= REACH_X[1], obj['name']
        assert REACH_Y[0] <= low_y and high_y <= REACH_Y[1], obj['name']
        assert table['x'][0] < low_x - radius and high_x + radius < table['x'][1], obj['name']
        assert table['y'][0] < low_y - radius and high_y + radius < table['y'][1], obj['name']


def test_only_isaac_builds_simready(simready):
    specs = {spec['name']: spec for spec in isaac_stage.generate(simready, 1)['objects']}
    assert specs['can']['usd'] == _objects(simready)['can']['usd']
    with pytest.raises(sc.ScenarioError, match="only the isaac backend loads; backend 'mujoco'"):
        mujoco_world.generate('/robot.xml', [0.0, 0.0, 0.0], simready, 1)
    with pytest.raises(sc.ScenarioError, match="only the isaac backend loads; backend 'gazebo'"):
        gazebo_world.generate_world(simready, 1)


@pytest.fixture
def tags():
    return catalog.load('scenarios', 'tabletop_tags')


def _charuco_boards(entry):
    return [obj for obj in entry['objects'] if 'charuco' in obj]


def _without_boards(entry):
    """Return the entry without its boards and the fields each scenario documents as its own."""
    layout = copy.deepcopy(entry)
    layout['objects'] = [obj for obj in layout['objects'] if 'charuco' not in obj]
    for key in ('name', 'description'):
        del layout[key]
    return layout


def test_tags_is_tabletop_can_with_boards(tags):
    assert _without_boards(tags) == _without_boards(catalog.load('scenarios', 'tabletop_can'))
    assert len(_charuco_boards(tags)) == 3


def test_tags_boards_lie_flat_on_the_table_with_the_paper(tags):
    objects = _objects(tags)
    for board in _charuco_boards(tags):
        assert board['pose']['xyz'][2] == objects['paper']['pose']['xyz'][2], board['name']
        assert board['size'][2] == objects['paper']['size'][2], board['name']
        assert board['pose'].get('rpy', [0.0, 0.0, 0.0])[:2] == [0.0, 0.0], board['name']


def _plan_bounds(body):
    """Return the plan-view bounds of a box turned only about z."""
    x, y = body['pose']['xyz'][:2]
    yaw = body['pose'].get('rpy', [0.0, 0.0, 0.0])[2]
    half_x = (abs(math.cos(yaw)) * body['size'][0] + abs(math.sin(yaw)) * body['size'][1]) / 2
    half_y = (abs(math.sin(yaw)) * body['size'][0] + abs(math.cos(yaw)) * body['size'][1]) / 2
    return {'x': (x - half_x, x + half_x), 'y': (y - half_y, y + half_y)}


def test_tags_boards_stay_on_the_table_and_off_one_another(tags):
    table = _footprint(_objects(tags)['table'])
    boards = _charuco_boards(tags)
    for board in boards:
        bounds = _plan_bounds(board)
        for axis in 'xy':
            low, high = bounds[axis]
            assert table[axis][0] < low and high < table[axis][1], board['name']
    for first, second in itertools.combinations(boards, 2):
        a, b = _plan_bounds(first), _plan_bounds(second)
        assert _clear(a['x'], b['x']) or _clear(a['y'], b['y']), (first['name'], second['name'])


def test_tags_boards_keep_out_of_the_pick_workspace(tags):
    # Clear of the paper, every object's worst-case spawn and the right arm's reach.
    objects = _objects(tags)
    keep_out = [_footprint(objects['paper']), {'x': REACH_X, 'y': REACH_Y}]
    for obj in _free(tags):
        radius = _bounding_radius(obj)
        low_x, high_x = _centre_range(obj, 'x')
        low_y, high_y = _centre_range(obj, 'y')
        keep_out.append(
            {'x': (low_x - radius, high_x + radius), 'y': (low_y - radius, high_y + radius)}
        )
    for board in _charuco_boards(tags):
        bounds = _plan_bounds(board)
        for area in keep_out:
            assert _clear(bounds['x'], area['x']) or _clear(bounds['y'], area['y']), board['name']


def test_tags_boards_are_published_at_their_declared_poses(tags):
    # /sim/objects carries static objects from sc.static_poses.
    poses = sc.static_poses(tags)
    for board in _charuco_boards(tags):
        assert poses[board['name']][0] == tuple(board['pose']['xyz']), board['name']


def test_only_mujoco_builds_tags(tags):
    with pytest.raises(sc.ScenarioError, match="only the mujoco backend draws; backend 'gazebo'"):
        gazebo_world.generate_world(tags, 1)
    with pytest.raises(sc.ScenarioError, match="only the mujoco backend draws; backend 'isaac'"):
        isaac_stage.generate(tags, 1)
