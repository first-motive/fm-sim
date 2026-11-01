"""The SimReady fetch tool against a fake host: everything it refuses, and what it installs."""

import hashlib
import io
import json
import os
import stat

import fetch_simready as tool
import pytest

REVISION = 'a' * 40
OTHER_REVISION = 'b' * 40
PACKAGE = 'food_box_a01'
API = f'{tool.HOST}/api/datasets/{tool.DATASET}'
RESOLVE = f'{tool.HOST}/datasets/{tool.DATASET}/resolve/{REVISION}/{PACKAGE}'
LISTING = f'{API}/tree/{REVISION}/{PACKAGE}?recursive=true&limit=1000'
PACKAGING = json.dumps({'license': tool.LICENSE}).encode()
SMALL = b'#usda 1.0\n'
LARGE = b'texture bytes'


def blob_hash(data):
    return hashlib.sha1(b'blob %d\0' % len(data) + data).hexdigest()


def small_entry(path, data):
    return {'type': 'file', 'path': f'{PACKAGE}/{path}', 'size': len(data), 'oid': blob_hash(data)}


def large_entry(path, data):
    return {
        'type': 'file', 'path': f'{PACKAGE}/{path}', 'size': len(data), 'oid': blob_hash(b'pointer'),
        'lfs': {'oid': hashlib.sha256(data).hexdigest(), 'size': len(data)},
    }


class Response(io.BytesIO):
    def __init__(self, body, link=None):
        super().__init__(body)
        self.headers = {'Link': link} if link else {}


class FakeHost:
    """The network boundary: a URL table, with a record of what was asked."""

    def __init__(self):
        self.pages = {}
        self.links = {}
        self.asked = []
        self.set_commit()
        self.set_listing([
            {'type': 'directory', 'path': f'{PACKAGE}/simready_usd', 'size': 0, 'oid': 'd' * 40},
            small_entry(tool.PACKAGING_FILE, PACKAGING),
            small_entry('simready_usd/box.usda', SMALL),
            large_entry('simready_usd/Textures/box.png', LARGE),
        ])
        self.pages[f'{RESOLVE}/{tool.PACKAGING_FILE}'] = PACKAGING
        self.pages[f'{RESOLVE}/simready_usd/box.usda'] = SMALL
        self.pages[f'{RESOLVE}/simready_usd/Textures/box.png'] = LARGE

    def set_commit(self, sha=REVISION, license_id='cc-by-4.0'):
        self.pages[f'{API}/revision/{REVISION}'] = json.dumps(
            {'sha': sha, 'cardData': {'license': license_id}}
        ).encode()

    def set_listing(self, entries):
        self.pages[LISTING] = json.dumps(entries).encode()

    def open(self, url, timeout=None):
        self.asked.append(url)
        if url not in self.pages:
            raise tool.urllib.error.URLError(f'no such page: {url}')
        return Response(self.pages[url], self.links.get(url))


@pytest.fixture
def host():
    return FakeHost()


@pytest.fixture
def assets(tmp_path):
    yield str(tmp_path / 'assets')
    # The tool leaves its tree read-only, which pytest's temporary-directory cleanup cannot remove.
    unlocked(tmp_path)


def fetch(host, assets, revision=REVISION, packages=(PACKAGE,)):
    return tool.fetch(tool.Hub(host), revision, packages, assets)


def tree_of(root):
    return sorted(
        os.path.relpath(os.path.join(parent, name), root)
        for parent, directories, names in os.walk(root) for name in (*directories, *names)
    )


def unlocked(path):
    """Open an installed read-only tree up, as an attacker with the account could."""
    for parent, directories, names in os.walk(path):
        os.chmod(parent, 0o755)
        # A tampering test plants a symlink out of the tree: chmod would follow it.
        for name in directories:
            if not os.path.islink(os.path.join(parent, name)):
                os.chmod(os.path.join(parent, name), 0o755)
        for name in names:
            if not os.path.islink(os.path.join(parent, name)):
                os.chmod(os.path.join(parent, name), 0o644)


@pytest.mark.parametrize('revision', [
    'main', 'refs/heads/main', 'v1.0', '4763fbfa', 'A' * 40, 'a' * 39, 'a' * 41, f'{"a" * 40}\n', '', None,
])
def test_an_unpinned_revision_is_refused_before_the_host_is_asked(host, assets, revision):
    with pytest.raises(tool.FetchError, match='not pinned'):
        fetch(host, assets, revision=revision)
    assert host.asked == [] and not os.path.exists(assets)


def test_the_default_revision_is_a_full_sha():
    assert tool.check_revision(tool.PINNED_REVISION) == tool.PINNED_REVISION


def test_the_command_line_refuses_an_unpinned_revision(host, assets, capsys):
    argv = ['--assets-dir', assets, '--revision', 'main', PACKAGE]
    assert tool.main(argv, tool.Hub(host)) == 1
    assert 'not pinned' in capsys.readouterr().err


def test_a_commit_the_host_answers_as_another_is_refused(host, assets):
    host.set_commit(sha=OTHER_REVISION)
    with pytest.raises(tool.FetchError, match=f'answered as commit .{OTHER_REVISION}'):
        fetch(host, assets)
    assert not os.path.exists(assets)


def test_a_dataset_under_another_licence_is_refused(host, assets):
    host.set_commit(license_id='cc-by-nc-4.0')
    with pytest.raises(tool.FetchError, match='not CC-BY-4.0'):
        fetch(host, assets)


def test_a_package_under_another_licence_is_refused(host, assets):
    packaging = json.dumps({'license': 'proprietary'}).encode()
    host.pages[f'{RESOLVE}/{tool.PACKAGING_FILE}'] = packaging
    host.set_listing([small_entry(tool.PACKAGING_FILE, packaging)])
    with pytest.raises(tool.FetchError, match="'proprietary'"):
        fetch(host, assets)
    assert tree_of(assets) == []


def test_a_package_with_no_licence_record_is_refused(host, assets):
    host.set_listing([small_entry('simready_usd/box.usda', SMALL)])
    with pytest.raises(tool.FetchError, match=tool.PACKAGING_FILE):
        fetch(host, assets)
    assert tree_of(assets) == []


@pytest.mark.parametrize('name', [
    '..', '.', '', '../evil', 'a/b', '/etc', 'a\\b', '.hidden', 'a b', 'a:b', 'a,b', 'a\nb', 'a\0b',
])
def test_a_hostile_package_name_is_refused_before_the_host_is_asked(host, assets, name):
    with pytest.raises(tool.FetchError, match='not a plain directory name'):
        fetch(host, assets, packages=(name,))
    assert host.asked == [] and not os.path.exists(assets)


@pytest.mark.parametrize('path', [
    f'{PACKAGE}/../../evil', f'{PACKAGE}/../evil', f'{PACKAGE}/a/../../../evil', '../evil', '/etc/passwd',
    f'/{PACKAGE}/evil', 'other/evil', PACKAGE, f'{PACKAGE}/', f'{PACKAGE}//evil', f'{PACKAGE}/./evil',
    f'{PACKAGE}/..', f'{PACKAGE}/...', f'{PACKAGE}/a\\..\\evil', f'{PACKAGE}/a\0b', f'{PACKAGE}/a\nb',
    f'{PACKAGE}/a b', 7, None,
])
def test_a_hostile_listed_path_is_refused_and_writes_nothing(host, assets, tmp_path, path):
    host.set_listing([{'type': 'file', 'path': path, 'size': 1, 'oid': 'c' * 40}])
    with pytest.raises(tool.FetchError, match='outside the package|unsafe segment|not a string'):
        fetch(host, assets)
    assert tree_of(tmp_path) == ['assets']


@pytest.mark.parametrize('entry', [
    {'type': 'symlink', 'path': f'{PACKAGE}/evil'},
    {'path': f'{PACKAGE}/evil'},
    'not an object',
])
def test_a_listing_entry_of_any_other_kind_is_refused(host, assets, entry):
    host.set_listing([entry])
    with pytest.raises(tool.FetchError, match='neither file nor directory'):
        fetch(host, assets)


@pytest.mark.parametrize('change', [
    {'oid': None},
    {'oid': 'not hex'},
    {'oid': 'c' * 40, 'lfs': {'oid': 'short'}},
    {'oid': 'c' * 40, 'lfs': {}},
])
def test_a_file_with_no_usable_hash_is_refused(host, assets, change):
    host.set_listing([{**small_entry('box.usda', SMALL), **change}])
    with pytest.raises(tool.FetchError, match='no hash'):
        fetch(host, assets)


@pytest.mark.parametrize('size', [-1, tool.MAX_FILE_BYTES + 1, '7', True, None])
def test_a_listed_size_out_of_bounds_is_refused(host, assets, size):
    host.set_listing([{**small_entry('box.usda', SMALL), 'size': size}])
    with pytest.raises(tool.FetchError, match='lists size'):
        fetch(host, assets)


def test_a_package_over_the_total_is_refused_before_any_download(host, assets, monkeypatch):
    monkeypatch.setattr(tool, 'MAX_PACKAGE_BYTES', 1)
    with pytest.raises(tool.FetchError, match='over 1 bytes'):
        fetch(host, assets)
    assert not any('/resolve/' in url for url in host.asked)


def test_a_listing_of_nothing_is_refused(host, assets):
    host.set_listing([])
    with pytest.raises(tool.FetchError, match='no such package'):
        fetch(host, assets)


@pytest.mark.parametrize('link', [
    f'<http://huggingface.co/api/datasets/{tool.DATASET}/tree/{REVISION}/{PACKAGE}>; rel="next"',
    f'<https://evil.example/api/datasets/{tool.DATASET}/tree/{REVISION}/{PACKAGE}>; rel="next"',
    f'<{API}/tree/{REVISION}/other>; rel="next"',
    f'<{API}/tree/{OTHER_REVISION}/{PACKAGE}>; rel="next"',
])
def test_a_listing_that_continues_elsewhere_is_refused(host, assets, link):
    host.links[LISTING] = link
    with pytest.raises(tool.FetchError, match='refusing to fetch|another path'):
        fetch(host, assets)
    assert len(host.asked) == 2


def test_a_listing_that_never_ends_is_refused(host, assets, monkeypatch):
    monkeypatch.setattr(tool, 'MAX_LISTING_PAGES', 1)
    second = f'{API}/tree/{REVISION}/{PACKAGE}?cursor=x'
    host.links[LISTING] = f'<{second}>; rel="next"'
    with pytest.raises(tool.FetchError, match='past 1 pages'):
        fetch(host, assets)


def test_a_listing_continues_on_its_next_page(host, assets):
    second = f'{API}/tree/{REVISION}/{PACKAGE}?recursive=true&limit=1000&cursor=x'
    host.links[LISTING] = f'<{second}>; rel="next"'
    host.pages[second] = json.dumps([large_entry('more.png', LARGE)]).encode()
    host.pages[f'{RESOLVE}/more.png'] = LARGE
    fetch(host, assets)
    assert os.path.isfile(os.path.join(assets, PACKAGE, 'more.png'))


@pytest.mark.parametrize('name,body,match', [
    ('simready_usd/box.usda', b'#usda 6.6\n', 'hash'),
    ('simready_usd/Textures/box.png', b'texture Bytes', 'hash'),
    ('simready_usd/box.usda', SMALL + b'more', 'more than the listed'),
    ('simready_usd/Textures/box.png', LARGE[:-1], 'received'),
])
def test_bytes_that_do_not_match_the_commit_install_nothing(host, assets, name, body, match):
    host.pages[f'{RESOLVE}/{name}'] = body
    with pytest.raises(tool.FetchError, match=match):
        fetch(host, assets)
    assert tree_of(assets) == []


def test_a_path_listed_twice_cannot_overwrite_itself(host, assets):
    host.set_listing([small_entry('box.usda', SMALL), small_entry('box.usda', SMALL)])
    host.pages[f'{RESOLVE}/box.usda'] = SMALL
    with pytest.raises(FileExistsError):
        fetch(host, assets)
    assert tree_of(assets) == []


def test_a_failed_package_keeps_the_directory_read_only(host, assets):
    host.pages[f'{RESOLVE}/simready_usd/box.usda'] = b'wrong'
    with pytest.raises(tool.FetchError):
        fetch(host, assets)
    assert stat.S_IMODE(os.lstat(assets).st_mode) == tool.READ_ONLY_DIR


@pytest.mark.parametrize('url', [
    'http://huggingface.co/x',
    'ftp://huggingface.co/x',
    'file:///etc/passwd',
    'https://evil.example/x',
    'https://huggingface.co.evil.example/x',
    'https://evilhuggingface.co/x',
    'https://user:pass@huggingface.co/x',
    'https://user@evil.example@huggingface.co/x',
])
def test_a_url_off_the_host_or_off_https_is_refused(url):
    with pytest.raises(tool.FetchError, match='refusing to fetch'):
        tool.check_url(url)


@pytest.mark.parametrize('url', [
    'https://huggingface.co/x',
    'https://cdn-lfs.huggingface.co/x',
    'https://cas-bridge.xethub.hf.co/x',
])
def test_a_url_on_the_host_or_its_content_stores_is_allowed(url):
    assert tool.check_url(url) == url


@pytest.mark.parametrize('target', ['http://cdn-lfs.huggingface.co/x', 'https://evil.example/x'])
def test_a_redirect_off_https_or_off_the_host_is_refused(target):
    redirects = tool._CheckedRedirects()
    request = tool.urllib.request.Request('https://huggingface.co/x')
    with pytest.raises(tool.FetchError, match='refusing to fetch'):
        redirects.redirect_request(request, None, 302, 'Found', {}, target)


def test_a_redirect_to_a_content_store_is_followed():
    redirects = tool._CheckedRedirects()
    request = tool.urllib.request.Request('https://huggingface.co/x')
    followed = redirects.redirect_request(
        request, None, 302, 'Found', {}, 'https://cas-bridge.xethub.hf.co/x'
    )
    assert followed.full_url == 'https://cas-bridge.xethub.hf.co/x'


@pytest.mark.parametrize('make,match', [
    (lambda root: 'relative/assets', 'not an absolute path'),
    (lambda root: f'{root}/a:b', "':' or ','"),
    (lambda root: f'{root}/a,b', "':' or ','"),
])
def test_an_assets_path_the_agent_would_refuse_is_refused(host, tmp_path, make, match):
    with pytest.raises(tool.FetchError, match=match):
        fetch(host, make(tmp_path))
    assert host.asked == []


def test_an_assets_path_that_is_a_symlink_is_refused(host, tmp_path):
    (tmp_path / 'real').mkdir()
    (tmp_path / 'link').symlink_to(tmp_path / 'real')
    with pytest.raises(tool.FetchError, match='is a symlink'):
        fetch(host, str(tmp_path / 'link'))
    assert tree_of(tmp_path / 'real') == []


def test_an_assets_path_that_is_a_file_is_refused(host, tmp_path):
    (tmp_path / 'file').write_text('x')
    with pytest.raises(tool.FetchError, match='not a directory'):
        fetch(host, str(tmp_path / 'file'))


def test_the_installed_package_matches_the_commit_and_is_read_only(host, assets):
    assert fetch(host, assets) == {PACKAGE: 'fetched'}
    root = os.path.join(assets, PACKAGE)
    with open(os.path.join(root, 'simready_usd', 'box.usda'), 'rb') as installed:
        assert installed.read() == SMALL
    with open(os.path.join(root, 'simready_usd', 'Textures', 'box.png'), 'rb') as installed:
        assert installed.read() == LARGE
    assert not [name for name in tree_of(assets) if name.startswith(tool.STAGING_PREFIX)]
    for parent, directories, names in os.walk(assets):
        assert not stat.S_IMODE(os.lstat(parent).st_mode) & 0o222, parent
        for name in names:
            assert not stat.S_IMODE(os.lstat(os.path.join(parent, name)).st_mode) & 0o222, name


def test_nothing_is_written_outside_the_assets_directory(host, tmp_path, assets):
    fetch(host, assets)
    assert os.listdir(tmp_path) == ['assets']


def test_the_notice_attributes_the_source_commit_and_licence(host, assets):
    fetch(host, assets)
    with open(os.path.join(assets, tool.NOTICE_NAME)) as notice_file:
        notice = notice_file.read()
    for expected in (tool.DATASET, f'Revision: {REVISION}', tool.LICENSE, tool.LICENSE_URL, PACKAGE):
        assert expected in notice


def test_a_second_run_verifies_without_downloading(host, assets):
    fetch(host, assets)
    host.asked.clear()
    assert fetch(host, assets) == {PACKAGE: 'present'}
    assert not any('/resolve/' in url for url in host.asked)
    assert stat.S_IMODE(os.lstat(assets).st_mode) == tool.READ_ONLY_DIR


def test_a_second_run_seals_a_copy_opened_up_since(host, assets):
    fetch(host, assets)
    root = os.path.join(assets, PACKAGE)
    unlocked(root)
    fetch(host, assets)
    for parent, _, names in os.walk(root):
        assert not stat.S_IMODE(os.lstat(parent).st_mode) & 0o222, parent
        for name in names:
            assert not stat.S_IMODE(os.lstat(os.path.join(parent, name)).st_mode) & 0o222, name


def test_a_package_before_a_failed_one_is_still_attributed(host, assets):
    with pytest.raises(tool.FetchError, match='no such package|cannot read'):
        fetch(host, assets, packages=(PACKAGE, 'food_absent_a01'))
    with open(os.path.join(assets, tool.NOTICE_NAME)) as notice_file:
        assert PACKAGE in notice_file.read()


def test_a_second_package_joins_the_first_in_the_notice(host, assets):
    fetch(host, assets)
    other = 'food_can_a01'
    other_listing = f'{API}/tree/{REVISION}/{other}?recursive=true&limit=1000'
    host.pages[other_listing] = json.dumps(
        [{**small_entry(tool.PACKAGING_FILE, PACKAGING), 'path': f'{other}/{tool.PACKAGING_FILE}'}]
    ).encode()
    host.pages[f'{tool.HOST}/datasets/{tool.DATASET}/resolve/{REVISION}/{other}/{tool.PACKAGING_FILE}'] = PACKAGING
    fetch(host, assets, packages=(other,))
    with open(os.path.join(assets, tool.NOTICE_NAME)) as notice_file:
        notice = notice_file.read()
    assert PACKAGE in notice and other in notice


def _changed_file(root):
    with open(os.path.join(root, 'simready_usd', 'box.usda'), 'wb') as installed:
        installed.write(b'#usda 6.6\n')


def _resized_file(root):
    with open(os.path.join(root, 'simready_usd', 'box.usda'), 'ab') as installed:
        installed.write(b'more')


def _added_file(root):
    with open(os.path.join(root, 'simready_usd', 'added.usda'), 'wb') as installed:
        installed.write(b'x')


def _removed_file(root):
    os.remove(os.path.join(root, 'simready_usd', 'box.usda'))


def _symlink(root):
    os.symlink('/etc', os.path.join(root, 'simready_usd', 'link'))


@pytest.mark.parametrize('tamper,match', [
    (_changed_file, 'hash'),
    (_resized_file, 'bytes'),
    (_added_file, 'added.usda'),
    (_removed_file, 'lacks'),
    (_symlink, 'symlink'),
])
def test_an_installed_copy_that_differs_from_the_commit_is_refused(host, assets, tamper, match):
    fetch(host, assets)
    root = os.path.join(assets, PACKAGE)
    unlocked(root)
    tamper(root)
    with pytest.raises(tool.FetchError, match=match):
        fetch(host, assets)


def test_a_package_path_that_is_a_symlink_is_refused(host, assets, tmp_path):
    fetch(host, assets, packages=())
    os.chmod(assets, 0o755)
    os.symlink(tmp_path, os.path.join(assets, PACKAGE))
    with pytest.raises(tool.FetchError, match='not a directory'):
        fetch(host, assets)


def test_one_directory_holds_one_commit(host, assets):
    fetch(host, assets)
    os.chmod(assets, 0o755)
    notice = os.path.join(assets, tool.NOTICE_NAME)
    os.chmod(notice, 0o644)
    with open(notice) as notice_file:
        text = notice_file.read()
    with open(notice, 'w') as notice_file:
        notice_file.write(text.replace(REVISION, OTHER_REVISION))
    with pytest.raises(tool.FetchError, match='new directory'):
        fetch(host, assets)
