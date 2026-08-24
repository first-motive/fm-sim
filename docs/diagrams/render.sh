#!/usr/bin/env bash
# fm-render: d2-render sha256:b7cce2f4d9b4155c4fa5ca1eeff249eb76b8c4bfde53a652e77e7d25ff9bd9c4 — rendered by the First Motive render plane — edit the upstream source, not this file
# Render every d2 diagram in the repo to an SVG sidecar with the First Motive
# font (Geist Mono). Sources live in docs/diagrams/ and next to the package they
# document (<package>/doc/diagrams/); both are found, so a repo that grows a
# package-level diagram needs no change here. The shared palette (styles.d2) and
# the font ship in this directory and are imported by relative path.
# Self-contained: the font ships in fonts/, so anyone with the repo can
# re-render without installing fonts. Needs d2 on PATH (https://d2lang.com).
#
#   ./render.sh           write each SVG sidecar next to its .d2 source
#   ./render.sh --check   re-render into a scratch dir and fail on any drift
#
# --check is what CI runs. A committed .svg that no longer matches its .d2 is a
# stale picture, and a stale picture reads as current. Layout and the version
# string d2 writes into the SVG are stable within a d2 release and not across
# them, so the CI job pins the version and --check reports the one it used.
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

if ! command -v d2 >/dev/null 2>&1; then
  echo "error: d2 not on PATH — install from https://d2lang.com" >&2
  exit 1
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

status=0
# styles.d2 is an import-only palette, not a diagram.
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
    compare=(diff -r --brief -- "$fresh" "$out")
  else
    fresh="$fresh.svg"
    compare=(cmp -s -- "$fresh" "$out")
  fi

  if [ ! -e "$out" ]; then
    echo "missing  ${out#"$ROOT"/} — this diagram has no committed render" >&2
    status=1
  elif ! "${compare[@]}" >/dev/null; then
    echo "stale    ${out#"$ROOT"/} — re-render it and commit the result" >&2
    status=1
  fi
done < <(find "$ROOT" -name '*.d2' ! -name 'styles.d2' -print0 | sort -z)

if [ "$MODE" = check ]; then
  if [ "$status" -ne 0 ]; then
    echo "" >&2
    echo "run ${0#"$ROOT"/} and commit what it writes" >&2
    echo "rendered here with $(d2 --version) — a different d2 version draws differently" >&2
  else
    echo "diagram check: every committed SVG matches its source"
  fi
fi
exit "$status"
