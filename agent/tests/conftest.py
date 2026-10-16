from __future__ import annotations

import pytest

from fm_sim_agent.agent import SimAgent
from fm_sim_agent.runs import RunStore
from tests.fakes import FakeRunner


@pytest.fixture
def store(tmp_path):
    return RunStore(tmp_path / "runs")


@pytest.fixture
def runner(store):
    return FakeRunner(store)


@pytest.fixture
def logs():
    return []


@pytest.fixture
def agent(store, runner, logs):
    agent = SimAgent("fm_ws_01", runner, store, log_sink=lambda run_id, record: logs.append((run_id, record)))
    agent.ready_poll_s = 0.01
    agent.min_free_bytes = 0  # the free-space check has its own tests
    return agent
