"""axol_can_on_table: can_on_table for the Axol, every object and region 0.10 m lower and
nothing else changed, tunables included, so the two stay one task at two table heights."""

import copy

from fm_sim_bringup import catalog
import pytest

DROP = 0.10
# Recorded from fm-skills: the Axol's base stands 0.86 m above the world, and
# axol_skills_config specs.yaml puts its table top at table_z -0.753 in that frame.
AXOL_BASE_Z = 0.86
AXOL_TABLE_Z = -0.753


def _lowered(entry):
    """entry with every pose's z raised back by DROP, its name and description dropped."""
    out = copy.deepcopy(entry)
    for key in ('name', 'description'):
        out.pop(key, None)
    for item in [*out['objects'], *out.get('regions', [])]:
        item['pose']['xyz'][2] = round(item['pose']['xyz'][2] + DROP, 6)
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
