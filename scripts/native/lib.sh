# shellcheck shell=bash
# Helpers build.sh sources. No Darwin or pixi dependency, so test_lib.sh runs them
# anywhere.

# fetch_at <url> <sha> <dir>: <dir> holds <url> checked out at <sha>, and nothing else.
# A <dir> already at <sha> is reset and cleaned instead of fetched again; it must be a
# repository of its own, so an enclosing repository at the same commit is never reset.
fetch_at() {
  local url="$1" sha="$2" dir="$3"
  [[ "$sha" =~ ^[0-9a-f]{40}$ ]] || { echo "not a full commit sha: $sha" >&2; return 1; }
  if [ -d "$dir/.git" ] && [ "$(git -C "$dir" rev-parse HEAD 2>/dev/null)" = "$sha" ]; then
    git -C "$dir" reset --quiet --hard && git -C "$dir" clean --quiet -fdx
    return
  fi
  rm -rf "$dir"
  git init --quiet "$dir"
  git -C "$dir" fetch --quiet --depth 1 "$url" "$sha"
  git -C "$dir" checkout --quiet FETCH_HEAD
}

# check_pinned <repos file>: every repository entry carries a full commit sha. An
# entry without a version line would import the remote's default branch.
check_pinned() {
  local repos="$1" entries pinned
  entries="$(grep -cE '^  [^ #][^:]*:$' "$repos")"
  pinned="$(grep -cE '^ +version: [0-9a-f]{40}( |$)' "$repos")"
  [ "$entries" -eq "$pinned" ] || {
    echo "$repos: $entries repositories, $pinned pinned to a full commit sha" >&2
    return 1
  }
}

# pins_stamp <file>...: one hash over the given inputs, in order.
pins_stamp() {
  cat "$@" | shasum -a 256 | cut -d' ' -f1
}

# ws_inputs <repo>: what the ws build reads, one path per line: this checkout's
# tracked scripts/native files (build script, patch, shim), fm-sim.repos and
# pixi.lock. The fm-sim commit itself is hashed by ws_stamp.
ws_inputs() {
  local repo="$1"
  git -C "$repo" ls-files -z scripts/native | tr '\0' '\n' | sed "s|^|$repo/|"
  printf '%s\n' "$repo/fm-sim.repos" "$repo/pixi.lock"
}

# ws_stamp <repo> <sha>: hash of <sha> (the commit the ws sources come from) and the
# current contents of every ws input.
ws_stamp() {
  local repo="$1" sha="$2" path
  {
    echo "$sha"
    while IFS= read -r path; do cat "$path"; done < <(ws_inputs "$repo")
  } | shasum -a 256 | cut -d' ' -f1
}

# snapshot_ws <repo> <dest> <record>: extract <repo>'s HEAD into <dest> and write the
# hash of that commit's inputs to <record>. HEAD is read once and both use it, so a
# commit made later (while colcon builds <dest>) never lands in the record.
snapshot_ws() {
  local repo="$1" dest="$2" record="$3" head
  head="$(git -C "$repo" rev-parse HEAD)"
  ws_stamp "$repo" "$head" > "$record"
  rm -rf "$dest"
  mkdir -p "$dest"
  git -C "$repo" archive "$head" | tar -x -C "$dest"
}

# overlay_shas <dockerfile>: the anvil-embodied-ai and fm-teleop SHAs the
# controllers Dockerfile pins, on one line, each checked as a full commit sha.
overlay_shas() {
  local dockerfile="$1" anvil teleop
  anvil="$(sed -n 's/^ARG ANVIL_SHA=//p' "$dockerfile")"
  teleop="$(sed -n 's/^ARG FM_TELEOP_SHA=//p' "$dockerfile")"
  [[ "$anvil" =~ ^[0-9a-f]{40}$ && "$teleop" =~ ^[0-9a-f]{40}$ ]] || {
    echo "$dockerfile: no full ANVIL_SHA and FM_TELEOP_SHA" >&2
    return 1
  }
  echo "$anvil $teleop"
}
