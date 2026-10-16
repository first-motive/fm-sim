from __future__ import annotations

import pytest

from fm_sim_agent import logline


@pytest.mark.parametrize(
    ("line", "expected"),
    [
        (
            "[sim_loop-1] [ERROR] [1760000000.25] [sim_loop]: step failed\n",
            {"t": 1760000000.25, "level": "error", "node": "sim_loop", "msg": "step failed"},
        ),
        (
            "[INFO] [launch]: All log files can be found below /tmp/.ros/log",
            {
                "t": 9.0,
                "level": "info",
                "node": "launch",
                "msg": "All log files can be found below /tmp/.ros/log",
            },
        ),
        (
            "[WARNING] [launch.user]: careful",
            {"t": 9.0, "level": "warn", "node": "launch.user", "msg": "careful"},
        ),
        (
            "[ros2_control_node-2] plain print",
            {"t": 9.0, "level": "info", "node": "ros2_control_node", "msg": "plain print"},
        ),
        ("\x1b[33mcoloured\x1b[0m", {"t": 9.0, "level": "info", "node": "launch", "msg": "coloured"}),
    ],
)
def test_each_launch_shape_parses(line, expected):
    assert logline.parse(line, now=9.0) == expected
