#!/usr/bin/env bash
# fm-render: d2-render sha256:2acf6888887bc63280e7fca7aa208520b31040421a6afe2a56e4df81797e49fc — rendered by the First Motive render plane — edit the upstream source, not this file
# Render every d2 diagram in the repo to an SVG sidecar with the First Motive
# font (Geist Mono). Sources live in docs/diagrams/ and next to the package they
# document (<package>/doc/diagrams/); both are found, so a repo that grows a
# package-level diagram needs no change here. The shared palette (styles.d2) and
# the font ship in this directory and are imported by relative path.
# Self-contained: the font ships in fonts/, so anyone with the repo can
# re-render without installing fonts. Needs d2 on PATH (https://d2lang.com).
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"    # docs/diagrams
ROOT="$(cd "$HERE/../.." && pwd)"        # repo root
FONT="$HERE/fonts/GeistMono-VF.ttf"

if ! command -v d2 >/dev/null 2>&1; then
  echo "error: d2 not on PATH — install from https://d2lang.com" >&2
  exit 1
fi

# styles.d2 is an import-only palette, not a diagram.
find "$ROOT" -name '*.d2' ! -name 'styles.d2' -print0 | sort -z | while IFS= read -r -d '' f; do
  out="${f%.d2}.svg"
  d2 --layout elk \
    --font-regular "$FONT" --font-bold "$FONT" --font-italic "$FONT" \
    "$f" "$out"
  echo "rendered ${out#"$ROOT"/}"
done
