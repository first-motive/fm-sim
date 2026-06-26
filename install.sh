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
# Inspect before running (always offer this path):
#   curl -fsSL https://raw.githubusercontent.com/first-motive/fm-sim/main/install.sh -o install.sh
#   less install.sh && bash install.sh
#
# From a clone:
#   ./install.sh [install|uninstall] [--no-pull] [--dry-run] [-y] [-h]
#
# --no-pull sets up the runtime only and skips the image pull. uninstall removes
# the pulled fm-sim image (the container runtime is left in place). The body is
# wrapped in main() and called on the last line, so a truncated curl|bash leaves
# an incomplete function that never runs.
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
    chmod 700 "$CACHE_DIR"  # lib.sh is sourced from here; keep the cache user-only
    local tmp="$cached.tmp.$$"
    curl -fsSL --proto '=https' --proto-redir '=https' "$FM_TOOLS_RAW/lib.sh" -o "$tmp" \
      || { rm -f "$tmp"; echo "error: failed to fetch lib.sh from fm-tools" >&2; exit 1; }
    [ -s "$tmp" ] || { rm -f "$tmp"; echo "error: empty lib.sh download" >&2; exit 1; }
    mv "$tmp" "$cached"
  fi
  # shellcheck source=/dev/null
  source "$cached"
}

usage() {
  cat <<'EOF'
install.sh — set up the fm-sim host (macOS only)

Usage: ./install.sh [install|uninstall] [options]

  install      set up the container runtime, pull the fm-sim image (default)
  uninstall    remove the pulled fm-sim image (runtime is left in place)

Options:
  --no-pull    install path: set up the runtime only, skip the image pull
  --dry-run    print what would happen, change nothing
  -y, --yes    non-interactive; assume yes (CI mode)
  -h, --help   show this help

Env: FM_SELFTEST=1  load deps + resolve OS, then stop before any host change.
EOF
}

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

do_install() {
  local pull="$1" dry="$2"
  echo "fm-sim install (macOS) ..."
  if [ "$dry" = "1" ]; then
    echo "  would set up the container runtime via fm-docker install.sh"
    [ "$pull" = "1" ] && echo "  would pull $IMAGE"
    return 0
  fi
  setup_runtime
  if [ "$pull" = "1" ]; then
    pull_image
  fi
  echo "Done. Launch the sim: curl -fsSL $FM_SIM_RAW/run.sh | bash"
}

do_uninstall() {
  local dry="$1"
  echo "fm-sim uninstall (macOS) ..."
  if [ "$dry" = "1" ]; then
    echo "  would remove image $IMAGE (if present); container runtime left in place"
    return 0
  fi
  # Remove only what this installer owns: the pulled fm-sim image. The container
  # runtime is shared (fm-docker owns it), so we never tear it down here.
  if fm_has_docker && docker image inspect "$IMAGE" >/dev/null 2>&1; then
    docker rmi "$IMAGE" && echo "removed $IMAGE"
  else
    echo "image $IMAGE not present; nothing to remove"
  fi
  echo "Done."
}

main() {
  # Parse before loading lib so --help works offline, with no network fetch.
  local cmd="install" pull=1 dry=0 arg
  for arg in "$@"; do
    case "$arg" in
      install|uninstall) cmd="$arg" ;;
      --no-pull) pull=0 ;;
      --dry-run) dry=1 ;;
      -y|--yes) ;;  # accepted for CI parity; this installer prompts for nothing
      -h|--help) usage; return 0 ;;
      *) echo "error: unknown argument: $arg" >&2; usage; return 2 ;;
    esac
  done

  load_lib

  local os
  os=$(fm_detect_os) || return 1

  # CI self-test hook: deps loaded and OS resolved — stop before any host change.
  # Lets the curl-path test exercise the piped fetch without installing anything.
  if [ -n "${FM_SELFTEST:-}" ]; then
    echo "selftest ok: lib loaded, os=$os"
    return 0
  fi

  if [ "$os" != "macos" ]; then
    echo "error: install.sh is macOS-only; Linux runs ROS2 Humble natively (see run.sh)." >&2
    return 1
  fi

  case "$cmd" in
    install)   do_install "$pull" "$dry" ;;
    uninstall) do_uninstall "$dry" ;;
  esac
}

main "$@"
