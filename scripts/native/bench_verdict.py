"""The go/no-go verdict scripts/native/bench.sh prints from two bench_probe.py results.

The native runtime goes when every camera runs at least ``CAMERA_RATIO`` times the
container's rate on the same topic and the native sim keeps real time (``MIN_RTF``:
one /clock tick of noise below 1.0). A camera that published nothing on either side,
a result with an error, or two results that name different cameras give no verdict.

Usage: python3 bench_verdict.py <container.json> <native.json>
Exit 0 on GO, 3 on NO-GO, 2 on no verdict. NO-GO has a code of its own so that a
crash (1) can never read as a NO-GO.
"""

import json
import sys

CAMERA_RATIO = 2.0
MIN_RTF = 0.99

GO, NO_VERDICT, NO_GO = 0, 2, 3


def verdict(container, native):
    """Return ``(outcome, reason)`` for two probe results; outcome is GO, NO_GO or NO_VERDICT."""
    for name, result in (('container', container), ('native', native)):
        if 'error' in result:
            return NO_VERDICT, f'{name}: {result["error"]}'
    if set(container['camera_hz']) != set(native['camera_hz']):
        return NO_VERDICT, 'the runtimes published different cameras'
    if not container['camera_hz']:
        return NO_VERDICT, 'no cameras'
    rates = [(container['camera_hz'][t], native['camera_hz'][t]) for t in sorted(container['camera_hz'])]
    if any(c <= 0 or n <= 0 for c, n in rates):
        return NO_VERDICT, 'a camera published nothing'
    ratio = min(n / c for c, n in rates)
    go = ratio >= CAMERA_RATIO and native['rtf'] >= MIN_RTF
    reason = (
        f'slowest camera {ratio:.1f}x container (>= {CAMERA_RATIO}), '
        f'native rtf {native["rtf"]:.3f} (>= {MIN_RTF})'
    )
    return (GO if go else NO_GO), reason


def row(name, result):
    if 'error' in result:
        return f'{name:10} error: {result["error"]}'
    cameras = '   '.join(f'{topic} {rate:5.2f} Hz' for topic, rate in result['camera_hz'].items())
    return f'{name:10} ready {result["ready_s"]:6.1f} s   rtf {result["rtf"]:5.3f}   {cameras}'


def load(path):
    try:
        lines = open(path).read().strip().splitlines()
        return json.loads(lines[-1]) if lines else {'error': 'no output'}
    except (OSError, ValueError) as error:
        return {'error': f'unreadable result: {error}'}


def main():
    container, native = load(sys.argv[1]), load(sys.argv[2])
    print(row('container', container))
    print(row('native', native))
    outcome, reason = verdict(container, native)
    print(f'gate: {reason} -> ' + {GO: 'GO', NO_GO: 'NO-GO', NO_VERDICT: 'no verdict'}[outcome])
    sys.exit(outcome)


if __name__ == '__main__':
    main()
