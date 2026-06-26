#!/usr/bin/env bash
# fm-sim host installer (macOS only). Brings a Mac to the point it can launch the
# baked sim image: sets up the container runtime (delegated to fm-docker), then
# pulls the published fm-sim image locally. Idempotent — safe to re-run.
#
# Install gets it running; clone is the dev path. This does not clone a workspace
# — for the edit-rebuild loop, clone fm-sim and use run.sh from the checkout.
#
# Linux is not handled here — it runs ROS2 Humble natively (see run.sh), with no
# container runtime to install.
#
# Curl-able (no clone needed):
#   curl -fsSL https://raw.githubusercontent.com/first-motive/fm-sim/main/install.sh | bash
#
# From a clone:
#   ./install.sh [--no-pull]
#
# --no-pull sets up the runtime only and skips the image pull.
set -euo pipefail

IMAGE="ghcr.io/first-motive/fm-sim:humble"
FM_SIM_RAW="https://raw.githubusercontent.com/first-motive/fm-sim/main"
# lib.sh is owned by fm-tools; the container runtime is delegated to fm-docker.
# Both are fetched from pinned release tags (the single reuse home).
FM_TOOLS_RAW="https://raw.githubusercontent.com/first-motive/fm-tools/v0.2.0"
FM_DOCKER_RAW="https://raw.githubusercontent.com/first-motive/fm-docker/v0.1.0"
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/fm-sim"

# Resolve the script's own dir; empty when piped via curl|bash. A clone has the
# repo files next to the script (REPO_DIR set); a piped run does not.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-}")" 2>/dev/null && pwd)" || SCRIPT_DIR=""
if [ -n "$SCRIPT_DIR" ] && [ -f "$SCRIPT_DIR/fm-sim.repos" ]; then
  REPO_DIR="$SCRIPT_DIR"
else
  REPO_DIR=""
fi

# Load the shared bootstrap library (fm-tools lib.sh) for fm_detect_os /
# fm_has_docker: reuse a cached fetch, else fetch from the pinned fm-tools tag
# and cache it. install.sh is itself curl|bash-able, so the library may not be
# on disk. The checks must run in this shell, so source rather than execute.
load_lib() {
  local cached="$CACHE_DIR/lib.sh"
  if [ ! -f "$cached" ]; then
    mkdir -p "$CACHE_DIR"
    local tmp="$cached.tmp.$$"
    curl -fsSL --proto '=https' --proto-redir '=https' "$FM_TOOLS_RAW/lib.sh" -o "$tmp" \
      || { rm -f "$tmp"; echo "error: failed to fetch lib.sh from fm-tools" >&2; exit 1; }
    [ -s "$tmp" ] || { rm -f "$tmp"; echo "error: empty lib.sh download" >&2; exit 1; }
    mv "$tmp" "$cached"
  fi
  # shellcheck source=/dev/null
  source "$cached"
}
load_lib

PULL=1
for arg in "$@"; do
  case "$arg" in
    --no-pull) PULL=0 ;;
    *) echo "error: unknown flag: $arg" >&2; exit 2 ;;
  esac
done

OS=$(fm_detect_os) || exit 1

# CI self-test hook: deps loaded and OS resolved — stop before any host changes.
# Lets the curl-path test exercise the piped fetch without installing anything.
if [ -n "${FM_SELFTEST:-}" ]; then
  echo "selftest ok: lib loaded, os=$OS"
  exit 0
fi

if [ "$OS" != "macos" ]; then
  echo "error: install.sh is macOS-only; Linux runs ROS2 Humble natively (see run.sh)." >&2
  exit 1
fi

# Delegate the container runtime (OrbStack install + start) to fm-docker's
# installer — no copy of that logic here. --no-pull stops it from pulling the
# fm-docker base image; this script pulls the fm-sim image below instead.
setup_runtime() {
  local imported="${REPO_DIR}/docker/install.sh"
  if [ -n "$REPO_DIR" ] && [ -f "$imported" ]; then
    bash "$imported" --no-pull
  else
    curl -fsSL --proto '=https' --proto-redir '=https' "$FM_DOCKER_RAW/install.sh" | bash -s -- --no-pull
  fi
}

pull_image() {
  if ! fm_has_docker; then
    echo "warn: docker unavailable — skipping image pull" >&2
    return 0
  fi
  echo "Pulling $IMAGE ..."
  docker pull "$IMAGE" || echo "warn: pull failed — pull later: docker pull $IMAGE" >&2
}

echo "fm-sim install (macOS) ..."
setup_runtime
if [ "$PULL" -eq 1 ]; then
  pull_image
fi
echo "Done. Launch the sim: curl -fsSL $FM_SIM_RAW/run.sh | bash"
