"""Where runs are filed on the host: one directory per run id.

    <runs_dir>/<run_id>/summary.json   the RunSummary, always
    <runs_dir>/<run_id>/bag/           mounted into the run's container; the
                                       recorder writes bag/recording/ here

The summary is written when a run starts (``ended_at`` null) and rewritten when
it ends, so a run the agent never got to finish still leaves a record. Writes go
to a temporary file and are renamed into place: a reader listing runs never sees
half a summary.

A run id starts with its UTC start time, so directory names sort in start
order and listing the newest runs reads only as many summaries as it returns.
"""

from __future__ import annotations

import json
import os
import stat
import tempfile
from pathlib import Path

import yaml

from fm_sim_agent.spec import valid_run_id

SUMMARY_NAME = "summary.json"
BAG_NAME = "bag"
#: The recorder's output directory inside ``bag/``. ``ros2 bag record -o``
#: refuses a directory that already exists, and ``bag/`` does: it is the mount.
RECORDING_NAME = "recording"
PARAMS_NAME = "params"
PARAMS_JSON = "params.json"
PARAMS_YAML = "params.yaml"

#: The keys every RunSummary carries (contracts §3). A file without them is not
#: one, whatever its name.
SUMMARY_KEYS = frozenset(("run_id", "host", "spec", "started_at", "ended_at", "outcome", "mcap", "metrics"))

#: A summary is a few hundred bytes. A file far past that is not one the agent
#: wrote, and is not read into memory to find out.
MAX_SUMMARY_BYTES = 1 << 16


def default_runs_dir() -> Path:
    """``$XDG_STATE_HOME/fm-sim/runs``, falling back to ``~/.local/state``."""
    state_home = os.environ.get("XDG_STATE_HOME", "").strip()
    base = Path(state_home) if state_home else Path.home() / ".local" / "state"
    return base / "fm-sim" / "runs"


def _write_atomic(directory: Path, name: str, text: str) -> Path:
    """Write ``directory/name`` through its own temporary file, renamed into place."""
    path = directory / name
    fd, tmp = tempfile.mkstemp(dir=directory, prefix=f".{Path(name).stem}-", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise
    return path


class RunStore:
    """The runs directory, read and written as RunSummary JSON."""

    def __init__(self, root: Path) -> None:
        self.root = root

    def run_dir(self, run_id: str) -> Path:
        if not valid_run_id(run_id):
            raise ValueError("not a run id")
        return self.root / run_id

    def bag_dir(self, run_id: str) -> Path:
        return self.run_dir(run_id) / BAG_NAME

    def recording_dir(self, run_id: str) -> Path:
        return self.bag_dir(run_id) / RECORDING_NAME

    def params_dir(self, run_id: str) -> Path:
        return self.run_dir(run_id) / PARAMS_NAME

    def write_params(self, run_id: str, params: dict, ros_params: dict) -> None:
        """Write a run's checked overrides, and its ROS params file when ``ros_params`` has nodes.

        Written before the run's container starts and never after, so the
        read-only mount sees each file whole.
        """
        directory = self.params_dir(run_id)
        directory.mkdir(parents=True, exist_ok=True)
        if not stat.S_ISDIR(os.lstat(directory).st_mode):
            raise OSError(f"{directory} is not a plain directory")
        _write_atomic(directory, PARAMS_JSON, json.dumps(params, indent=2, sort_keys=True) + "\n")
        if ros_params:
            _write_atomic(directory, PARAMS_YAML, yaml.safe_dump(ros_params, sort_keys=True))

    def recordings(self, run_id: str) -> list[tuple[str, int]]:
        """The run's MCAP files as (path relative to the runs dir, bytes), in name order.

        The bag directory is written by the container, so nothing in it is
        followed: a symlink, at any level, is skipped rather than read through.
        """
        found = []
        directory = self.recording_dir(run_id)
        try:
            for level in (self.run_dir(run_id), self.bag_dir(run_id), directory):
                if not stat.S_ISDIR(os.lstat(level).st_mode):
                    return []
            with os.scandir(directory) as entries:
                for entry in entries:
                    if entry.name.endswith(".mcap") and entry.is_file(follow_symlinks=False):
                        size = entry.stat(follow_symlinks=False).st_size
                        found.append((str(Path(entry.path).relative_to(self.root)), size))
        except OSError:
            return []
        return sorted(found)

    def exists(self, run_id: str) -> bool:
        return self.run_dir(run_id).exists()

    def write(self, summary: dict) -> Path:
        """Write a summary atomically. A run directory that is a symlink is refused.

        Each write has its own temporary file, so two writers of one run (a
        down racing a failure) never interleave into the same file.
        """
        directory = self.run_dir(summary["run_id"])
        directory.mkdir(parents=True, exist_ok=True)
        if not stat.S_ISDIR(os.lstat(directory).st_mode):
            raise OSError(f"{directory} is not a plain directory")
        return _write_atomic(directory, SUMMARY_NAME, json.dumps(summary, indent=2, sort_keys=True) + "\n")

    def read(self, run_id: str) -> dict | None:
        """One run's summary, or ``None`` if absent, a symlink, oversized or not a summary."""
        path = self.run_dir(run_id) / SUMMARY_NAME
        try:
            if not stat.S_ISDIR(os.lstat(path.parent).st_mode):
                return None
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
            with os.fdopen(fd, "rb") as handle:
                if not stat.S_ISREG(os.fstat(handle.fileno()).st_mode):
                    return None
                raw = handle.read(MAX_SUMMARY_BYTES + 1)
            if len(raw) > MAX_SUMMARY_BYTES:
                return None
            summary = json.loads(raw.decode("utf-8"))
        except (OSError, ValueError):
            return None
        if not isinstance(summary, dict) or set(summary) != SUMMARY_KEYS or summary["run_id"] != run_id:
            return None
        return summary

    def _run_ids(self) -> list[str]:
        """Every run directory's name, newest first."""
        if not self.root.is_dir():
            return []
        return sorted((e.name for e in self.root.iterdir() if valid_run_id(e.name)), reverse=True)

    def recent(self, limit: int) -> list[dict]:
        """Up to ``limit`` summaries, newest first.

        A directory whose summary does not read is skipped: one corrupt file must
        not hide every other run.
        """
        summaries: list[dict] = []
        for run_id in self._run_ids():
            if len(summaries) >= limit:
                break
            summary = self.read(run_id)
            if summary is not None:
                summaries.append(summary)
        return summaries

    def read_all(self) -> list[dict]:
        """Every readable summary, newest first."""
        return [s for s in map(self.read, self._run_ids()) if s is not None]

    def unfinished(self) -> list[dict]:
        """Summaries still marked running: runs a previous agent never ended."""
        return [summary for summary in self.read_all() if summary["ended_at"] is None]
