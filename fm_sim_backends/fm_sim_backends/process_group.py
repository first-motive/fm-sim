"""Run an engine as a bash job, so launch shutdown reaches all of it.

An engine started through a wrapper (``xvfb-run``'s ``/bin/sh``, the ``sh -c``
ros_gz_sim runs ``ruby gz sim`` under) can outlive its launch: the shell defers
launch's SIGINT and, killed by the SIGTERM that follows, leaves its children
running. Here bash runs the command as a job (``set -m``: a process group of its
own, and the command does not start with SIGINT ignored), and traps pass each
shutdown signal on; bash then waits for the job and returns its exit status.

By default the signal goes to the job's whole process group (gz-sim). With
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

_GROUP = (
    'set -m; "$@" & job=$!; '
    'trap "kill -INT -$job" INT; trap "kill -TERM -$job" TERM; '
    'wait $job; wait $job'
)
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
