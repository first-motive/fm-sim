#!/usr/bin/env python3
"""Download named SimReady packages, at a pinned commit, into a read-only directory.

    uv run --no-project tools/fetch_simready.py --assets-dir /data/fm-assets \\
        food_beverage_can_e01 food_can_tomatoSoup_a01 food_box_gelatin_a01

The packages live in the Hugging Face dataset ``nvidia/simready-assets``. Everything
the host sends is hostile until checked: every package name and listed path, every
redirect, and every file's bytes. A package is built in a private staging directory
and renamed into place only once each file matches the hash the host lists for it at
that commit, so a failed fetch leaves nothing behind. Run again, it verifies what is
there instead of downloading.

Standard library only, so a sim host needs nothing installed to run it.
"""

import argparse
import hashlib
import json
import os
import re
import shutil
import stat
import sys
import tempfile
import urllib.error
import urllib.parse
import urllib.request

DATASET = 'nvidia/simready-assets'
# A commit of the dataset: a branch or a tag could move under a scenario's recorded bounds.
PINNED_REVISION = '4763fbfaa4f8e97658e68b57d01b4c86659e08b5'
HOST = 'https://huggingface.co'
# Large files redirect from HOST to its content stores.
ALLOWED_HOST_SUFFIXES = ('.huggingface.co', '.hf.co')
LICENSE = 'CC-BY-4.0'
LICENSE_URL = 'https://creativecommons.org/licenses/by/4.0/'
NOTICE_NAME = 'NOTICE.txt'
PACKAGING_FILE = 'com.nvidia.simready.packaging.json'
STAGING_PREFIX = '.staging-'
READ_ONLY_FILE = 0o444
READ_ONLY_DIR = 0o555

REVISION = re.compile(r'[0-9a-f]{40}')
SHA256 = re.compile(r'[0-9a-f]{64}')
PACKAGE_NAME = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]*')
# The characters real packages use: no separator, no control character, no shell or
# docker meaning. A segment of dots alone is refused apart.
PATH_SEGMENT = re.compile(r'[A-Za-z0-9_.@+=-]+')
NOTICE_REVISION = re.compile(r'^Revision: ([0-9a-f]{40})$', re.M)
NEXT_LINK = re.compile(r'<([^>]+)>;\s*rel="next"')

# Far above the largest food package (28 MB): these bound a hostile listing, not a real one.
MAX_FILE_BYTES = 2 ** 30
MAX_PACKAGE_BYTES = 4 * 2 ** 30
MAX_FILES = 5000
MAX_LISTING_PAGES = 100
MAX_JSON_BYTES = 16 * 2 ** 20
TIMEOUT_S = 60
CHUNK_BYTES = 2 ** 20


class FetchError(Exception):
    """Something the tool refuses: the message says what, and nothing was installed."""


def check_revision(revision):
    """Return ``revision`` if it is a full commit SHA, the only thing that cannot move."""
    if not isinstance(revision, str) or not REVISION.fullmatch(revision):
        raise FetchError(f'revision {revision!r} is not pinned: give a full 40-character commit SHA')
    return revision


def check_package_name(name):
    if not PACKAGE_NAME.fullmatch(name):
        raise FetchError(f'package name {name!r} is not a plain directory name')
    return name


def check_url(url):
    """Return ``url`` if it is https to the dataset's host or its content stores."""
    parts = urllib.parse.urlsplit(url)
    host = parts.hostname or ''
    allowed = host == urllib.parse.urlsplit(HOST).hostname or host.endswith(ALLOWED_HOST_SUFFIXES)
    if parts.scheme != 'https' or not allowed or parts.username or parts.password:
        raise FetchError(f'refusing to fetch {url!r}: only https to {HOST} and its content stores')
    return url


class _CheckedRedirects(urllib.request.HTTPRedirectHandler):
    """Follow a redirect only to where a first request could have gone."""

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        check_url(newurl)
        return super().redirect_request(req, fp, code, msg, headers, newurl)


class Hub:
    """Reads from the host. ``opener`` is the network boundary, which the tests replace."""

    def __init__(self, opener=None):
        self._opener = opener or urllib.request.build_opener(_CheckedRedirects)

    def open(self, url):
        try:
            return self._opener.open(check_url(url), timeout=TIMEOUT_S)
        except (urllib.error.URLError, OSError) as error:
            raise FetchError(f'cannot read {url}: {error}') from error

    def read_json(self, url):
        """Return ``(document, the next page's URL or None)``."""
        with self.open(url) as response:
            body = response.read(MAX_JSON_BYTES + 1)
            link = response.headers.get('Link') or ''
        if len(body) > MAX_JSON_BYTES:
            raise FetchError(f'{url} answered with over {MAX_JSON_BYTES} bytes')
        try:
            document = json.loads(body)
        except ValueError as error:
            raise FetchError(f'{url} did not answer with JSON: {error}') from error
        found = NEXT_LINK.search(link)
        return document, found.group(1) if found else None


def check_commit(hub, revision):
    """Refuse a revision the host does not resolve to itself, or a dataset under another licence."""
    url = f'{HOST}/api/datasets/{DATASET}/revision/{revision}'
    document, _ = hub.read_json(url)
    if not isinstance(document, dict) or document.get('sha') != revision:
        got = document.get('sha') if isinstance(document, dict) else document
        raise FetchError(f'{DATASET} at {revision} answered as commit {got!r}')
    card = document.get('cardData')
    license_id = card.get('license') if isinstance(card, dict) else None
    if str(license_id).lower() != LICENSE.lower():
        raise FetchError(f'{DATASET} at {revision} is licensed {license_id!r}, not {LICENSE}')


def relative_parts(path, package):
    """Return a listed path's segments below ``package``, refusing any that could leave it."""
    if not isinstance(path, str):
        raise FetchError(f'{package}: listed path {path!r} is not a string')
    package_root, *parts = path.split('/')
    if package_root != package or not parts:
        raise FetchError(f'{package}: listed path {path!r} is outside the package')
    for part in parts:
        if not PATH_SEGMENT.fullmatch(part) or not part.strip('.'):
            raise FetchError(f'{package}: listed path {path!r} has an unsafe segment {part!r}')
    return parts


def _checked_file(entry, package):
    """Return ``(parts, size, algorithm, expected digest)`` of one listed file."""
    parts = relative_parts(entry.get('path'), package)
    size = entry.get('size')
    if not isinstance(size, int) or isinstance(size, bool) or not 0 <= size <= MAX_FILE_BYTES:
        raise FetchError(f'{package}: {entry.get("path")!r} lists size {size!r}')
    lfs = entry.get('lfs')
    # A large file is listed by the SHA-256 of its bytes, a small one by its git blob hash.
    if isinstance(lfs, dict) and isinstance(lfs.get('oid'), str) and SHA256.fullmatch(lfs['oid']):
        return parts, size, 'sha256', lfs['oid']
    oid = entry.get('oid')
    if lfs is None and isinstance(oid, str) and REVISION.fullmatch(oid):
        return parts, size, 'blob', oid
    raise FetchError(f'{package}: {entry.get("path")!r} lists no hash to verify it against')


def list_files(hub, revision, package):
    """Return every file of ``package`` at ``revision``, each checked by ``_checked_file``."""
    prefix = f'/api/datasets/{DATASET}/tree/{revision}/{package}'
    url = f'{HOST}{prefix}?recursive=true&limit=1000'
    files = []
    pages = 0
    while url:
        pages += 1
        if pages > MAX_LISTING_PAGES:
            raise FetchError(f'{package}: the listing runs past {MAX_LISTING_PAGES} pages')
        if not urllib.parse.urlsplit(check_url(url)).path == prefix:
            raise FetchError(f'{package}: the listing continues at {url!r}, another path')
        entries, url = hub.read_json(url)
        if not isinstance(entries, list):
            raise FetchError(f'{package}: the listing is not a list')
        for entry in entries:
            kind = entry.get('type') if isinstance(entry, dict) else None
            if kind == 'file':
                files.append(_checked_file(entry, package))
            elif kind != 'directory':
                raise FetchError(f'{package}: listing entry {entry!r} is neither file nor directory')
        if len(files) > MAX_FILES:
            raise FetchError(f'{package}: lists over {MAX_FILES} files')
    if not files:
        raise FetchError(f'{package}: no such package in {DATASET} at {revision}')
    if sum(size for _, size, _, _ in files) > MAX_PACKAGE_BYTES:
        raise FetchError(f'{package}: lists over {MAX_PACKAGE_BYTES} bytes')
    return files


def _hasher(algorithm, size):
    if algorithm == 'sha256':
        return hashlib.sha256()
    # What git hashes for a blob: a header, then the bytes.
    return hashlib.sha1(f'blob {size}\0'.encode())


def _check_digest(hasher, expected, what):
    if hasher.hexdigest() != expected:
        raise FetchError(f'{what}: hash {hasher.hexdigest()} is not the listed {expected}')


def _download(hub, revision, package, listed, staging):
    """Write one file into ``staging``, refusing bytes that are not what the listing says."""
    parts, size, algorithm, expected = listed
    quoted = '/'.join(urllib.parse.quote(part) for part in (package, *parts))
    what = f'{package}/{"/".join(parts)}'
    path = os.path.join(staging, *parts)
    os.makedirs(os.path.dirname(path), mode=0o700, exist_ok=True)
    hasher = _hasher(algorithm, size)
    received = 0
    # O_EXCL and O_NOFOLLOW: a listing naming a path twice cannot overwrite or write through it.
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(descriptor, 'wb') as out, hub.open(
        f'{HOST}/datasets/{DATASET}/resolve/{revision}/{quoted}'
    ) as response:
        while chunk := response.read(CHUNK_BYTES):
            received += len(chunk)
            if received > size:
                raise FetchError(f'{what}: the host sent more than the listed {size} bytes')
            hasher.update(chunk)
            out.write(chunk)
    if received != size:
        raise FetchError(f'{what}: received {received} bytes of the listed {size}')
    _check_digest(hasher, expected, what)


def _check_package_license(root, package):
    try:
        with open(os.path.join(root, PACKAGING_FILE), 'rb') as packaging_file:
            license_id = json.load(packaging_file).get('license')
    except (OSError, ValueError, AttributeError) as error:
        raise FetchError(f'{package}: no readable {PACKAGING_FILE}: {error}') from error
    if license_id != LICENSE:
        raise FetchError(f'{package}: packaged under {license_id!r}, not {LICENSE}')


def _seal(root):
    """Make everything under ``root`` read-only, its top directory last for the caller to rename."""
    for parent, directories, names in os.walk(root, topdown=False):
        for name in names:
            os.chmod(os.path.join(parent, name), READ_ONLY_FILE)
        for name in directories:
            os.chmod(os.path.join(parent, name), READ_ONLY_DIR)


def _remove_staging(path):
    for parent, directories, _ in os.walk(path):
        os.chmod(parent, stat.S_IRWXU)
        for name in directories:
            os.chmod(os.path.join(parent, name), stat.S_IRWXU)
    shutil.rmtree(path)


def verify_package(root, package, files):
    """Refuse an installed package that is not exactly what the commit lists."""
    expected = {os.path.join(*parts): (size, algorithm, digest) for parts, size, algorithm, digest in files}

    def unreadable(error):
        raise FetchError(f'{package}: cannot read {error.filename}: {error.strerror}')

    for parent, directories, names in os.walk(root, onerror=unreadable, followlinks=False):
        for name in (*directories, *names):
            path = os.path.join(parent, name)
            if os.path.islink(path):
                raise FetchError(f'{package}: installed copy holds a symlink: {path}')
        for name in names:
            path = os.path.join(parent, name)
            relative = os.path.relpath(path, root)
            if relative not in expected:
                raise FetchError(f'{package}: installed copy holds {relative}, which the commit does not')
            size, algorithm, digest = expected.pop(relative)
            if os.lstat(path).st_size != size:
                raise FetchError(f'{package}: installed {relative} is not the listed {size} bytes')
            hasher = _hasher(algorithm, size)
            with open(path, 'rb') as installed:
                while chunk := installed.read(CHUNK_BYTES):
                    hasher.update(chunk)
            _check_digest(hasher, digest, f'{package}: installed {relative}')
    if expected:
        raise FetchError(f'{package}: installed copy lacks {sorted(expected)[0]}')


def fetch_package(hub, revision, package, assets_dir):
    """Install ``package`` under ``assets_dir``; return 'fetched', or 'present' if it verified."""
    check_package_name(package)
    files = list_files(hub, revision, package)
    target = os.path.join(assets_dir, package)
    try:
        mode = os.lstat(target).st_mode
    except FileNotFoundError:
        mode = None
    if mode is not None:
        if not stat.S_ISDIR(mode):
            raise FetchError(f'{target} exists and is not a directory')
        verify_package(target, package, files)
        _seal(target)
        os.chmod(target, READ_ONLY_DIR)
        return 'present'
    staging = tempfile.mkdtemp(prefix=STAGING_PREFIX, dir=assets_dir)
    try:
        for listed in files:
            _download(hub, revision, package, listed, staging)
        _check_package_license(staging, package)
        _seal(staging)
        os.rename(staging, target)
    except BaseException:
        _remove_staging(staging)
        raise
    os.chmod(target, READ_ONLY_DIR)
    return 'fetched'


def check_assets_dir(path):
    """Refuse a directory the agent's ``--assets-dir`` would refuse, before writing to it."""
    if not os.path.isabs(path):
        raise FetchError(f'--assets-dir {path} is not an absolute path')
    if ':' in path or ',' in path:
        raise FetchError(f"--assets-dir {path} has a ':' or ',', which docker mount values split on")
    try:
        mode = os.lstat(path).st_mode
    except FileNotFoundError:
        return
    if stat.S_ISLNK(mode):
        raise FetchError(f'--assets-dir {path} is a symlink')
    if not stat.S_ISDIR(mode):
        raise FetchError(f'--assets-dir {path} is not a directory')


def _notice_revision(assets_dir):
    try:
        with open(os.path.join(assets_dir, NOTICE_NAME)) as notice_file:
            found = NOTICE_REVISION.search(notice_file.read())
    except FileNotFoundError:
        return None
    return found.group(1) if found else None


def write_notice(assets_dir, revision):
    """Write the attribution CC-BY-4.0 requires, naming every package in the directory."""
    packages = sorted(
        entry.name for entry in os.scandir(assets_dir)
        if entry.is_dir(follow_symlinks=False) and not entry.name.startswith('.')
    )
    text = (
        'NVIDIA SimReady assets\n\n'
        f'Source: {HOST}/datasets/{DATASET}\n'
        f'Revision: {revision}\n'
        f'Copyright: NVIDIA Corporation\n'
        f'License: {LICENSE} ({LICENSE_URL})\n'
        'Changes: none. Each package is as published; its own licence text is in\n'
        '.metadata/LICENSE.txt, and tools/fetch_simready.py in fm-sim installed it.\n\n'
        'Packages:\n' + ''.join(f'  {package}\n' for package in packages)
    )
    descriptor, partial = tempfile.mkstemp(prefix=STAGING_PREFIX, dir=assets_dir)
    with os.fdopen(descriptor, 'w') as out:
        out.write(text)
    os.chmod(partial, READ_ONLY_FILE)
    os.replace(partial, os.path.join(assets_dir, NOTICE_NAME))


def fetch(hub, revision, packages, assets_dir):
    """Install ``packages`` and the notice; leave ``assets_dir`` read-only."""
    check_revision(revision)
    check_assets_dir(assets_dir)
    for package in packages:
        check_package_name(package)
    # The host is asked nothing until the local inputs are known good.
    check_commit(hub, revision)
    os.makedirs(assets_dir, mode=0o755, exist_ok=True)
    os.chmod(assets_dir, 0o755)
    try:
        installed = _notice_revision(assets_dir)
        if installed not in (None, revision):
            # One notice describes one commit; a mix would attribute packages wrongly.
            raise FetchError(f'{assets_dir} holds revision {installed}: use a new directory for {revision}')
        results = {}
        for package in packages:
            results[package] = fetch_package(hub, revision, package, assets_dir)
            # After each one: if a later package fails, this one is still attributed.
            write_notice(assets_dir, revision)
    finally:
        os.chmod(assets_dir, READ_ONLY_DIR)
    return results


def main(argv=None, hub=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('packages', nargs='+', metavar='PACKAGE', help='package directories of the dataset')
    parser.add_argument('--assets-dir', required=True, help='absolute path of the directory to install into')
    parser.add_argument('--revision', default=PINNED_REVISION, help='full commit SHA (default: the pinned one)')
    args = parser.parse_args(argv)
    try:
        results = fetch(hub or Hub(), args.revision, args.packages, args.assets_dir)
    except (FetchError, OSError) as error:
        print(f'fetch_simready: {error}', file=sys.stderr)
        return 1
    for package, result in results.items():
        print(f'{result}: {package}')
    print(f'{args.assets_dir}: read-only at {args.revision}; attribution in {NOTICE_NAME}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
