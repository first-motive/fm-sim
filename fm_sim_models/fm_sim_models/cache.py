"""Where the generated MJCF models are written, one file per content hash.

A generated model is named by its text's hash and replaced atomically, so a caller
never reads a half-written or stale copy, and one that differs from the text
(edited, truncated) is rewritten. Old hashes are left in place, so a sim still
running from one never loses its model.
"""

import hashlib
import os
import tempfile


def cache_dir():
    """Return ``$XDG_CACHE_HOME/fm_sim_models`` (default ``~/.cache``)."""
    base = os.environ.get('XDG_CACHE_HOME') or os.path.join(os.path.expanduser('~'), '.cache')
    return os.path.join(base, 'fm_sim_models')


def write(text, stem):
    """Write ``text`` to ``<cache_dir>/<stem>_<hash>.xml`` unless it is there; return the path."""
    directory = cache_dir()
    path = os.path.join(directory, f'{stem}_{hashlib.sha256(text.encode()).hexdigest()[:16]}.xml')
    try:
        with open(path) as cached:
            if cached.read() == text:
                return path
    except FileNotFoundError:
        pass
    os.makedirs(directory, exist_ok=True)
    handle, partial = tempfile.mkstemp(dir=directory, prefix=f'.{stem}-', suffix='.tmp')
    try:
        with os.fdopen(handle, 'w') as model_file:
            model_file.write(text)
        os.chmod(partial, 0o644)
        os.replace(partial, path)
    except BaseException:
        os.unlink(partial)
        raise
    return path
