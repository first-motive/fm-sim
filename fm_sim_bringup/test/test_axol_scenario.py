"""axol_can_on_table: can_on_table for the Axol, every object and region 0.10 m lower, the
can 0.12 m nearer the robot, and nothing else changed, tunables included, so the two stay
one task at two table heights."""

import copy

from fm_sim_bringup import catalog
import pytest

DROP = 0.10
# The can's centre moves from world x 0.38 to 0.26, clear of the resting right hand's fingers.
CAN_NEARER = 0.12
# The resting right fingers reach a can centred past this world x: a MoveIt start-state check
# found every plan's start in collision beyond it.
RESTING_HAND_X = 0.30
# Recorded from fm-skills: the Axol's base stands 0.86 m above the world, and
# axol_skills_config specs.yaml puts its table top at table_z -0.753 in that frame.
AXOL_BASE_Z = 0.86
AXOL_TABLE_Z = -0.753


def _lowered(entry):
    """entry with every pose's z raised back by DROP, the can's x moved back by CAN_NEARER, its
    spawn_x cap restored, and its name and description dropped."""
    out = copy.deepcopy(entry)
    for key in ('name', 'description'):
        out.pop(key, None)
    for item in [*out['objects'], *out.get('regions', [])]:
        item['pose']['xyz'][2] = round(item['pose']['xyz'][2] + DROP, 6)
        if item['name'] == 'can':
            item['pose']['xyz'][0] = round(item['pose']['xyz'][0] + CAN_NEARER, 6)
    out['params']['spawn_x']['max'] = 0.1
    return out


def test_it_is_can_on_table_lowered():
    axol, base = catalog.load('scenarios', 'axol_can_on_table'), catalog.load('scenarios', 'can_on_table')
    expected = copy.deepcopy(base)
    for key in ('name', 'description'):
        expected.pop(key, None)
    assert _lowered(axol) == expected


def test_its_table_top_matches_the_recorded_axol_table():
    table = next(o for o in catalog.load('scenarios', 'axol_can_on_table')['objects'] if o['name'] == 'table')
    top = table['pose']['xyz'][2] + table['size'][2] / 2
    assert top - AXOL_BASE_Z == pytest.approx(AXOL_TABLE_Z)


def test_the_can_spawns_clear_of_the_resting_right_hand():
    entry = catalog.load('scenarios', 'axol_can_on_table')
    can = next(o for o in entry['objects'] if o['name'] == 'can')
    assert can['pose']['xyz'][0] + max(can['spawn']['x']) < RESTING_HAND_X
    # The tunable range cannot put it back under the hand either.
    assert can['pose']['xyz'][0] + entry['params']['spawn_x']['max'] < RESTING_HAND_X
