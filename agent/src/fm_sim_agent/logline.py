"""Turn one container log line into the record ``fm/sim/<host>/logs/<run_id>`` carries.

``ros2 launch`` prints three shapes, all handled here:

    [sim_loop-1] [INFO] [1697461234.123456789] [sim_loop]: started
    [INFO] [launch]: All log files can be found below /tmp/.ros/log/...
    [sim_loop-1] a bare print from the process

A line that matches none is still forwarded, as ``info`` from ``launch``: an
unparsed line is a line someone may need, never one to drop.
"""

from __future__ import annotations

import re
import time

#: tradeoff: a log record carries at most this many bytes of message. A ROS log
#: line is rarely past a few hundred; a dumped array or a traceback frame can be
#: kilobytes, and its tail is lost here rather than letting one line cost the
#: fabric a megabyte. The runner reads at most this much of a line at a time.
MAX_LINE_BYTES = 4096

#: Terminal colour codes ``ros2 launch`` adds when it thinks it has a TTY.
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

PROCESS_PREFIX = re.compile(r"^\[(?P<process>[^\]\s]+-\d+)\] ?(?P<rest>.*)$")
ROS_LINE = re.compile(
    r"^\[(?P<level>DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\] "
    r"(?:\[(?P<stamp>\d+(?:\.\d+)?)\] )?"
    r"\[(?P<node>[^\]]+)\]: ?(?P<msg>.*)$"
)

LEVELS = {
    "DEBUG": "debug",
    "INFO": "info",
    "WARN": "warn",
    "WARNING": "warn",
    "ERROR": "error",
    "FATAL": "fatal",
}


def parse(line: str, now: float | None = None) -> dict:
    """``{"t", "level", "node", "msg"}`` for one line. ``t`` is Unix seconds."""
    text = ANSI.sub("", line).rstrip("\r\n")
    text = text.encode("utf-8")[:MAX_LINE_BYTES].decode("utf-8", "ignore")
    received = time.time() if now is None else now
    node = "launch"
    prefixed = PROCESS_PREFIX.match(text)
    if prefixed:
        node = prefixed.group("process").rsplit("-", 1)[0]
        text = prefixed.group("rest")
    ros = ROS_LINE.match(text)
    if ros is None:
        return {"t": received, "level": "info", "node": node, "msg": text}
    stamp = ros.group("stamp")
    return {
        "t": float(stamp) if stamp else received,
        "level": LEVELS[ros.group("level")],
        "node": ros.group("node"),
        "msg": ros.group("msg"),
    }
