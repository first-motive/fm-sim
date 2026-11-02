#!/usr/bin/env bash
# fm-render: d2-render sha256:6171d8b53166f6785ef549f885ad4159ca18270b76c451b86e07222d27b1042f — rendered by the First Motive render plane — edit the upstream source, not this file
# Render every d2 and tldraw diagram in the repo to an SVG sidecar with the
# First Motive font (Geist Mono). Sources live in docs/diagrams/ and next to the
# package they document (<package>/doc/diagrams/); both are found, so a repo
# that grows a package-level diagram needs no change here. The shared palettes
# (styles.d2, tldraw-palette.json) and the font ship in this directory.
# Self-contained: the font ships in fonts/, so anyone with the repo can
# re-render without installing fonts. A .d2 source needs d2 on PATH
# (https://d2lang.com); rendering a .tldr source needs Node (npx).
#
#   ./render.sh           write each SVG sidecar next to its source
#   ./render.sh --check   fail on any SVG that no longer matches its source
#
# --check is what CI runs. A committed .svg that no longer matches its .d2 is a
# stale picture, and a stale picture reads as current. Layout is stable within a
# d2 release and not across them, so the CI job pins the version and --check
# reports the one it used. The data-d2-version attribute is normalized before
# comparing: the release tarball writes "v0.7.1" where the brew build writes
# "0.7.1", so the raw byte would fail every check between install channels.
#
# A tldraw export runs a headless browser, so CI cannot re-render it cheaply.
# Instead the render writes a sha256 of its inputs (the .tldr source, the
# palette, the rebrand step, the font) into the SVG's first line, and --check
# recomputes it: a .tldr edited without a re-export fails as stale, and the
# check needs neither Node nor a browser.
# The render maps tldraw's colours to the First Motive palette through
# tldraw-palette.json (tldraw-rebrand.cjs) and fails on a colour outside it.
set -euo pipefail

MODE=render
case "${1:-}" in
  "") ;;
  --check) MODE=check ;;
  *)
    echo "usage: $(basename "$0") [--check]" >&2
    exit 2
    ;;
esac

HERE="$(cd "$(dirname "$0")" && pwd)"    # docs/diagrams
ROOT="$(cd "$HERE/../.." && pwd)"        # repo root
FONT="$HERE/fonts/GeistMono-VF.ttf"
TLDRAW_PALETTE="$HERE/tldraw-palette.json"

# styles.d2 is an import-only palette, not a diagram.
d2_sources() { find "$ROOT" -name '*.d2' ! -name 'styles.d2' -print0 | sort -z; }
tldraw_sources() { find "$ROOT" -name '*.tldr' -print0 | sort -z; }

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "error: $1 not on PATH — $2" >&2
    exit 1
  fi
}
if [ -n "$(d2_sources | head -c1)" ]; then
  need_cmd d2 "install from https://d2lang.com"
fi
if [ "$MODE" = render ] && [ -n "$(tldraw_sources | head -c1)" ]; then
  need_cmd node "install Node to export tldraw diagrams"
  need_cmd npx "install Node to export tldraw diagrams"
fi

render() {
  d2 --layout elk \
    --font-regular "$FONT" --font-bold "$FONT" --font-italic "$FONT" \
    "$1" "$2"
}

if [ "$MODE" = check ]; then
  SCRATCH="$(mktemp -d)"
  trap 'rm -rf "$SCRATCH"' EXIT
fi

# Compare SVGs with the d2 version attribute normalized (channel-dependent
# leading "v"). Directories of boards compare file-by-file the same way.
normalize_svg() { sed 's/data-d2-version="v\{0,1\}\([^"]*\)"/data-d2-version="\1"/' "$1"; }
svg_same() { cmp -s <(normalize_svg "$1") <(normalize_svg "$2"); }
svg_dir_same() {
  local a="$1" b="$2" rel
  ( cd "$a" && find . -type f | sort ) > "$SCRATCH/.list_a"
  ( cd "$b" && find . -type f | sort ) > "$SCRATCH/.list_b"
  cmp -s "$SCRATCH/.list_a" "$SCRATCH/.list_b" || return 1
  while IFS= read -r rel; do
    svg_same "$a/$rel" "$b/$rel" || return 1
  done < "$SCRATCH/.list_a"
}

# The stamp hashes everything the picture depends on: the source, the palette
# and pinned exporter, the rebrand step, and the font. A palette change then makes every tldraw SVG
# stale, as a re-render makes every d2 SVG differ. shasum ships with macOS,
# sha256sum with Linux.
tldraw_stamp() {
  local digest
  if command -v sha256sum >/dev/null 2>&1; then
    digest="$(cat "$1" "$TLDRAW_PALETTE" "$HERE/tldraw-rebrand.cjs" "$FONT" | sha256sum)"
  else
    digest="$(cat "$1" "$TLDRAW_PALETTE" "$HERE/tldraw-rebrand.cjs" "$FONT" | shasum -a 256)"
  fi
  echo "<!-- fm-tldraw sha256:${digest%% *} -->"
}

# Export one .tldr, then rebrand it (tldraw-rebrand.cjs) with the source hash
# on line one. The result is written beside the export and moved into place,
# so a committed symlink at the sidecar path is replaced rather than followed.
render_tldraw() {
  local raw exporter
  raw="$(mktemp -d)"
  # The pinned exporter comes from tldraw-palette.json; TLDRAW_EXPORT lets a
  # test stand in for the browser.
  exporter="${TLDRAW_EXPORT:-$(node -p 'require(process.argv[1]).exporter' "$TLDRAW_PALETTE")}"
  # shellcheck disable=SC2086 # the command is meant to split into words
  if $exporter "$1" --output "$raw" --name diagram >/dev/null &&
    node "$HERE/tldraw-rebrand.cjs" "$TLDRAW_PALETTE" "$FONT" "$raw/diagram.svg" "$(tldraw_stamp "$1")" >"$raw/branded.svg"; then
    mv -f "$raw/branded.svg" "$2"
    rm -rf "$raw"
  else
    rm -rf "$raw"
    return 1
  fi
}

status=0
while IFS= read -r -d '' f; do
  out="${f%.tldr}.svg"
  if [ "$MODE" != check ]; then
    render_tldraw "$f" "$out"
    echo "rendered ${out#"$ROOT"/}"
  elif [ ! -e "$out" ]; then
    echo "missing  ${out#"$ROOT"/} — this diagram has no committed render" >&2
    status=1
  elif [ "$(head -n1 "$out")" != "$(tldraw_stamp "$f")" ]; then
    echo "stale    ${out#"$ROOT"/} — re-render it and commit the result" >&2
    status=1
  fi
done < <(tldraw_sources)

while IFS= read -r -d '' f; do
  rel="${f#"$ROOT"/}"
  out="${f%.d2}.svg"
  if [ "$MODE" != check ]; then
    render "$f" "$out"
    echo "rendered ${out#"$ROOT"/}"
    continue
  fi

  # A multi-board diagram lands as a directory of boards beside the source
  # rather than a single sidecar, so compare whichever shape d2 just produced.
  fresh="$SCRATCH/${rel//\//_}"
  render "$f" "$fresh.svg" >/dev/null
  if [ -d "$fresh" ]; then
    out="${f%.d2}"
    same() { svg_dir_same "$fresh" "$out"; }
  else
    fresh="$fresh.svg"
    same() { svg_same "$fresh" "$out"; }
  fi

  if [ ! -e "$out" ]; then
    echo "missing  ${out#"$ROOT"/} — this diagram has no committed render" >&2
    status=1
  elif ! same; then
    echo "stale    ${out#"$ROOT"/} — re-render it and commit the result" >&2
    status=1
  fi
done < <(d2_sources)

if [ "$MODE" = check ]; then
  if [ "$status" -ne 0 ]; then
    echo "" >&2
    echo "run ${0#"$ROOT"/} and commit what it writes" >&2
    if [ -n "$(d2_sources | head -c1)" ]; then
      echo "rendered here with $(d2 --version) — a different d2 version draws differently" >&2
    fi
  else
    echo "diagram check: every committed SVG matches its source"
  fi
fi
exit "$status"
