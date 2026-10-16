"""A runner that hosts no containers, for driving the agent's real lifecycle."""

from __future__ import annotations

import queue
import threading
import time

from fm_sim_agent.runner import RunnerError
from fm_sim_agent.runs import RunStore


def entry(name: str, available: bool = True) -> dict:
    return {"name": name, "package": "fm_sim_bringup", "description": name, "available": available}


CATALOG = {
    "schema_version": 1,
    "backends": [entry("mujoco"), entry("gazebo"), entry("isaac", available=False)],
    "robots": [
        {
            **entry("openarm"),
            "robot_profile": "fm_openarm_v2",
            "controller_roles": {
                "broadcasters": ["joint_state_broadcaster"],
                "right_arm": ["openarm_right_arm_controller"],
            },
        },
        entry("bare"),
    ],
    "scenarios": [entry("empty"), entry("pick_cube")],
    "controllers": [entry("idle")],
}

SPEC = {
    "backend": "mujoco",
    "robot": "openarm",
    "scenario": "empty",
    "controller": "idle",
    "seed": 7,
    "record": False,
}

_EXIT = object()


class FakeRunner:
    """Records every call. Failures and slowness are switched on per test."""

    def __init__(self, store: RunStore) -> None:
        self.store = store
        self.calls: list[tuple] = []
        self.image = "sha256:one"
        self.catalog_doc: object = CATALOG
        self.catalog_fetches = 0
        self.outcome_value: dict | None = {
            "scenario": "empty", "seed": 7, "success": True, "reason": "placed", "elapsed_s": 4.5,
        }  # fmt: skip
        self.fail: dict[str, str] = {}
        self.start_gate: threading.Event | None = None
        self.reset_gate: threading.Event | None = None
        self.outcome_gate: threading.Event | None = None
        self.ready = threading.Event()
        self.ready.set()
        self.log_lines: list[str] = []
        self.controllers = {"joint_state_broadcaster": "active", "openarm_right_arm_controller": "inactive"}
        self.serves_reset = True
        self._logs: queue.Queue = queue.Queue()

    def _maybe_fail(self, name: str) -> None:
        if name in self.fail:
            raise RunnerError(self.fail[name])

    def image_id(self) -> str:
        self._maybe_fail("image_id")
        return self.image

    def catalog(self) -> object:
        self.catalog_fetches += 1
        self._maybe_fail("catalog")
        return self.catalog_doc

    def start(self, run_id: str, spec: dict) -> None:
        self.calls.append(("start", run_id, dict(spec)))
        if self.start_gate is not None:
            self.start_gate.wait(5)
        self._maybe_fail("start")
        for line in self.log_lines:
            self._logs.put(line)

    def list_controllers(self, run_id: str) -> dict[str, str]:
        self.calls.append(("list_controllers", run_id))
        if not self.ready.is_set():
            raise RunnerError("controller_manager not up yet")
        self._maybe_fail("list_controllers")
        return dict(self.controllers)

    def has_service(self, run_id: str, name: str) -> bool:
        self.calls.append(("has_service", run_id, name))
        return self.serves_reset

    def reset(self, run_id: str, seed: int, timeout_s: float) -> str:
        self.calls.append(("reset", run_id, seed, timeout_s))
        if self.reset_gate is not None:
            self.reset_gate.wait(5)
        self._maybe_fail("reset")
        return "reset"

    def start_recording(self, run_id: str) -> None:
        self.calls.append(("start_recording", run_id))
        self._maybe_fail("start_recording")
        recording = self.store.recording_dir(run_id)
        recording.mkdir(parents=True, exist_ok=True)
        (recording / "recording_0.mcap").write_bytes(b"\x89MCAP0\r\n")
        (recording / "recording_1.mcap").write_bytes(b"\x89MCAP0\r\n12")

    def stop_recording(self, run_id: str) -> None:
        self.calls.append(("stop_recording", run_id))

    def outcome(self, run_id: str) -> dict | None:
        self.calls.append(("outcome", run_id))
        if self.outcome_gate is not None:
            self.outcome_gate.wait(5)
        return self.outcome_value

    def stop(self, run_id: str) -> None:
        self.calls.append(("stop", run_id))
        self.ready.set()
        self.exit()

    def exit(self) -> None:
        """The container exits: the log stream ends."""
        self._logs.put(_EXIT)

    def logs(self, run_id: str):
        self._maybe_fail("logs")
        while True:
            line = self._logs.get()
            if line is _EXIT:
                return
            yield line

    def stop_stale(self) -> list[str]:
        self.calls.append(("stop_stale",))
        return ["fm-sim-run-old"]

    def names(self) -> list[str]:
        return [call[0] for call in self.calls]


def wait_for(predicate, timeout_s: float = 5.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("condition never held")
        time.sleep(0.01)
