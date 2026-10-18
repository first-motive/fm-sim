"""Run an engine as a bash job, so launch shutdown reaches all of it.

An engine started through a wrapper (``xvfb-run``'s ``/bin/sh``, the ``sh -c``
ros_gz_sim runs ``ruby gz sim`` under) can outlive its launch: the shell defers
launch's SIGINT and, killed by the SIGTERM that follows, leaves its children
running. Here bash runs the command as a job (``set -m``: a process group of its
own, and the command does not start with SIGINT ignored), and traps pass each
shutdown signal on; bash then waits for the job and returns its exit status.

By default the signal goes to the job's whole process group (gz-sim), and a
watchdog then sends that group SIGKILL if it is still alive ``KILL_AFTER_S``
later. gz-sim can hang on its way out (seen with the cameras rendering on
software GL): it then outlived the launch's SIGKILL, which reaches only this
wrapper. The watchdog is a job of its own, so that SIGKILL does not stop it. With
``spare``, it goes to the job's children except those named ``spare`` (MuJoCo's
xvfb-run spares ``Xvfb``): the node stops first, and the wrapper then stops what
it spared, as on a normal exit. Signalling Xvfb in the same instant would cut the
node's X connection in the middle of its shutdown.

Checked with GNU bash 5.2 (the Jazzy image) on Linux; ``spare`` reads
``/proc/<pid>/comm``. macOS is not checked: there ``command`` returns the argv
unchanged, as MuJoCo's launch runs its node with no prefix there.
"""

import platform
import re
import shlex

# Lands with launch's SIGTERM (5 s after its SIGINT): a server that ignored SIGINT
# is killed then, before launch's SIGKILL, which reaches only this wrapper.
KILL_AFTER_S = 5
# The watchdog starts once, writes nowhere (so it holds none of launch's output pipes), and
# is stopped when the job exits in time.
_GROUP = (
    'set -m; watch=; "$@" & job=$!; '
    'stop() {{ kill -$1 -$job 2>/dev/null; [ -n "$watch" ] || {{ '
    '( sleep {kill_after}; kill -KILL -$job 2>/dev/null ) >/dev/null 2>&1 & watch=$!; }}; }}; '
    'trap "stop INT" INT; trap "stop TERM" TERM; '
    'wait $job; wait $job; status=$?; '
    '[ -z "$watch" ] || kill -- -$watch 2>/dev/null; exit $status'
).format(kill_after=KILL_AFTER_S)
_SPARING = (
    'set -m; "$@" & job=$!; '
    'forward() {{ for pid in $(pgrep -P $job); do '
    '[ "$(cat /proc/$pid/comm)" = {spare} ] || kill -$1 $pid; done; }}; '
    'trap "forward INT" INT; trap "forward TERM" TERM; '
    'wait $job; wait $job'
)
# A process name as /proc/<pid>/comm shows it; it lands in the script unquoted.
_COMM = re.compile(r'[A-Za-z0-9_.-]{1,15}')


def command(*argv, spare=None):
    """Return the argv that runs ``argv`` as a job that shutdown signals reach (Linux)."""
    if platform.system() == 'Darwin':
        return list(argv)
    if spare is None:
        script = _GROUP
    elif _COMM.fullmatch(spare):
        script = _SPARING.format(spare=spare)
    else:
        raise ValueError(f'spare must be a process name, got {spare!r}')
    return ['bash', '-c', script, 'fm_sim_process_group', *argv]


def prefix(*argv, spare=None):
    """Return ``command(*argv)`` as a launch ``prefix`` string (the node's argv follows)."""
    return shlex.join(command(*argv, spare=spare))
