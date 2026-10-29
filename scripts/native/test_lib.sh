#!/usr/bin/env bash
# Tests for lib.sh, against local repositories only: bash scripts/native/test_lib.sh
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=lib.sh
. "$here/lib.sh"

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
failures=0

check() {
  local name="$1"; shift
  if "$@"; then echo "ok   $name"; else echo "FAIL $name"; failures=$((failures + 1)); fi
}

git init --quiet "$tmp/upstream"
git -C "$tmp/upstream" -c user.name=t -c user.email=t@t commit --quiet --allow-empty -m one
first="$(git -C "$tmp/upstream" rev-parse HEAD)"
echo two > "$tmp/upstream/file"
git -C "$tmp/upstream" add file
git -C "$tmp/upstream" -c user.name=t -c user.email=t@t commit --quiet -m two
second="$(git -C "$tmp/upstream" rev-parse HEAD)"
url="file://$tmp/upstream"

fetches_the_sha() {
  fetch_at "$url" "$first" "$tmp/a" && [ "$(git -C "$tmp/a" rev-parse HEAD)" = "$first" ]
}
moves_to_a_new_sha() {
  fetch_at "$url" "$second" "$tmp/a" && [ -f "$tmp/a/file" ]
}
cleans_a_dirty_tree() {
  echo dirt > "$tmp/a/file" && touch "$tmp/a/untracked"
  fetch_at "$url" "$second" "$tmp/a" \
    && [ "$(cat "$tmp/a/file")" = two ] && [ ! -e "$tmp/a/untracked" ]
}
refuses_a_short_sha() {
  ! fetch_at "$url" "${first:0:12}" "$tmp/b" 2>/dev/null && [ ! -e "$tmp/b" ]
}
spares_an_enclosing_repo() {
  # A plain directory inside a repository at the pinned commit: refetched, and the
  # enclosing repository's untracked file survives.
  git clone --quiet "$url" "$tmp/outer"
  touch "$tmp/outer/keep"
  mkdir "$tmp/outer/inner"
  fetch_at "$url" "$second" "$tmp/outer/inner" \
    && [ -e "$tmp/outer/keep" ] && [ -d "$tmp/outer/inner/.git" ]
}
accepts_a_pinned_manifest() {
  printf 'repositories:\n  a:\n    type: git\n    version: %s\n' "$first" > "$tmp/ok.repos"
  check_pinned "$tmp/ok.repos"
}
refuses_an_unpinned_entry() {
  printf 'repositories:\n  a:\n    type: git\n    version: %s\n  b:\n    type: git\n' \
    "$first" > "$tmp/bad.repos"
  ! check_pinned "$tmp/bad.repos" 2>/dev/null
}
refuses_a_branch_version() {
  printf 'repositories:\n  a:\n    type: git\n    version: main\n' > "$tmp/branch.repos"
  ! check_pinned "$tmp/branch.repos" 2>/dev/null
}
stamp_follows_its_inputs() {
  echo a > "$tmp/s1" && echo b > "$tmp/s2"
  local before after
  before="$(pins_stamp "$tmp/s1" "$tmp/s2")"
  echo c > "$tmp/s2"
  after="$(pins_stamp "$tmp/s1" "$tmp/s2")"
  [ "${#before}" -eq 64 ] && [ "$before" != "$after" ]
}

# A checkout shaped like fm-sim's, for ws_inputs and ws_stamp.
make_checkout() {
  local repo="$tmp/checkout"
  mkdir -p "$repo/scripts/native/shims/vendor" "$repo/scripts/native/patches"
  echo build > "$repo/scripts/native/build.sh"
  echo patch > "$repo/scripts/native/patches/a.patch"
  echo shim > "$repo/scripts/native/shims/vendor/CMakeLists.txt"
  echo repos > "$repo/fm-sim.repos"
  echo lock > "$repo/pixi.lock"
  git init --quiet "$repo"
  git -C "$repo" add .
  git -C "$repo" -c user.name=t -c user.email=t@t commit --quiet -m init
  echo "$repo"
}
inputs_cover_shim_patch_repos_and_lock() {
  local repo inputs
  repo="$(make_checkout)"
  inputs="$(ws_inputs "$repo")"
  grep -qx "$repo/scripts/native/shims/vendor/CMakeLists.txt" <<< "$inputs" \
    && grep -qx "$repo/scripts/native/patches/a.patch" <<< "$inputs" \
    && grep -qx "$repo/fm-sim.repos" <<< "$inputs" \
    && grep -qx "$repo/pixi.lock" <<< "$inputs"
}
stamp_follows_a_shim_edit() {
  local repo="$tmp/checkout" head before
  head="$(git -C "$repo" rev-parse HEAD)"
  before="$(ws_stamp "$repo" "$head")"
  echo changed > "$repo/scripts/native/shims/vendor/CMakeLists.txt"
  [ "$before" != "$(ws_stamp "$repo" "$head")" ]
}
stamp_follows_the_sha() {
  local repo="$tmp/checkout" head
  head="$(git -C "$repo" rev-parse HEAD)"
  [ "$(ws_stamp "$repo" "$head")" != "$(ws_stamp "$repo" "$first")" ]
}
stamp_ignores_a_later_commit() {
  # build.sh records the stamp at stage start from the sha it archives; a commit made
  # while colcon runs leaves that record alone.
  local repo="$tmp/checkout" head before
  head="$(git -C "$repo" rev-parse HEAD)"
  before="$(ws_stamp "$repo" "$head")"
  git -C "$repo" -c user.name=t -c user.email=t@t commit --quiet --allow-empty -m next
  [ "$before" = "$(ws_stamp "$repo" "$head")" ]
}
snapshot_records_the_archived_commit() {
  # A commit after the snapshot (as while colcon builds) changes neither the record
  # nor the sources: both stay at the commit that was archived.
  local repo="$tmp/checkout" head
  head="$(git -C "$repo" rev-parse HEAD)"
  snapshot_ws "$repo" "$tmp/src" "$tmp/record"
  echo later > "$repo/scripts/native/build.sh"
  git -C "$repo" -c user.name=t -c user.email=t@t commit --quiet -am later
  [ "$(cat "$tmp/record")" != "$(ws_stamp "$repo" "$(git -C "$repo" rev-parse HEAD)")" ] \
    && [ "$(cat "$tmp/src/scripts/native/build.sh")" != later ] \
    && git -C "$repo" checkout --quiet "$head" -- scripts/native/build.sh \
    && [ "$(cat "$tmp/record")" = "$(ws_stamp "$repo" "$head")" ]
}
reads_the_overlay_shas() {
  printf 'FROM x\nARG ANVIL_SHA=%s\nARG FM_TELEOP_SHA=%s\n' "$first" "$second" > "$tmp/Dockerfile"
  [ "$(overlay_shas "$tmp/Dockerfile")" = "$first $second" ]
}
refuses_a_missing_overlay_sha() {
  printf 'FROM x\nARG ANVIL_SHA=%s\n' "$first" > "$tmp/Dockerfile.short"
  ! overlay_shas "$tmp/Dockerfile.short" 2>/dev/null
}

check "fetch_at checks out the sha" fetches_the_sha
check "fetch_at moves to a new sha" moves_to_a_new_sha
check "fetch_at resets and cleans a tree at the sha" cleans_a_dirty_tree
check "fetch_at refuses a short sha" refuses_a_short_sha
check "fetch_at never resets an enclosing repository" spares_an_enclosing_repo
check "check_pinned accepts full shas" accepts_a_pinned_manifest
check "check_pinned refuses an entry with no version" refuses_an_unpinned_entry
check "check_pinned refuses a branch" refuses_a_branch_version
check "pins_stamp changes with its inputs" stamp_follows_its_inputs
check "ws_inputs lists the shim, patch, manifest and lock" inputs_cover_shim_patch_repos_and_lock
check "ws_stamp changes with a shim edit" stamp_follows_a_shim_edit
check "ws_stamp changes with the sha" stamp_follows_the_sha
check "ws_stamp ignores a commit after the sha" stamp_ignores_a_later_commit
check "snapshot_ws records the commit it archived" snapshot_records_the_archived_commit
check "overlay_shas reads both pins" reads_the_overlay_shas
check "overlay_shas refuses a missing pin" refuses_a_missing_overlay_sha

[ "$failures" -eq 0 ]
