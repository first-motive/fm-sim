#!/usr/bin/env bash
# Render every d2 diagram to SVG with the First Motive font (Geist Mono).
# Self-contained: the font ships in fonts/, so anyone with the repo can
# re-render without installing fonts or any personal tooling. Needs d2 on PATH
# (https://d2lang.com). styles.d2 is an import-only palette, not a diagram.
set -euo pipefail
cd "$(dirname "$0")"

FONT="fonts/GeistMono-VF.ttf"

if ! command -v d2 >/dev/null 2>&1; then
  echo "error: d2 not on PATH — install from https://d2lang.com" >&2
  exit 1
fi

for f in *.d2; do
  [ "$f" = "styles.d2" ] && continue
  out="${f%.d2}.svg"
  d2 --layout elk \
    --font-regular "$FONT" --font-bold "$FONT" --font-italic "$FONT" \
    "$f" "$out"
  echo "rendered $out"
done
