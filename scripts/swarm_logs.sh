#!/usr/bin/env bash
# swarm_logs.sh — open the current swarm run's ROS 2 logs in lnav with the
# project's format definitions.
#
# Usage:
#   swarm_logs.sh                   # open ~/.ros/log/latest in lnav
#   swarm_logs.sh <run-dir>         # open a specific log directory
#   swarm_logs.sh -- <lnav-args>    # forward extra args to lnav
#
# Inside lnav, the most useful keys/queries:
#   :filter-in <regex>    keep only lines matching <regex>
#   :filter-out <regex>   drop lines matching <regex>
#   :hide-fields logger   hide the logger column
#   e / E                 jump to next/prev error
#   w / W                 jump to next/prev warning
#   ;SELECT log_level, logger, count(*) FROM ros2_node_log GROUP BY 1,2 ORDER BY 3 DESC
#                         tally messages by node+level (SQL over logs)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FORMATS_DIR="$SCRIPT_DIR/lnav"

# Ubuntu 22.04's apt lnav is 0.9.0 (2020) — broken input prompt rendering and
# crashes on backspace inside `:` / `/` / `;` modes. Use the upstream static
# musl binary instead. Drops into ~/.local/bin, no root, no apt repos.
LNAV_VERSION="0.14.0"
LOCAL_BIN="$HOME/.local/bin"
LNAV_BIN="$LOCAL_BIN/lnav"

lnav_too_old() {
  # Return 0 (true) if installed lnav is older than 0.12.
  local v
  v="$(lnav -V 2>/dev/null | awk '{print $2}')" || return 0
  [[ -z "$v" ]] && return 0
  ! printf '%s\n%s\n' "0.12.0" "$v" | sort -V -C
}

install_lnav() {
  local arch zip url tmpdir
  case "$(uname -m)" in
    x86_64)  arch="linux-musl-x86_64" ;;
    aarch64) arch="linux-musl-arm64" ;;
    *) echo "Unsupported arch $(uname -m)" >&2; return 1 ;;
  esac
  zip="lnav-${LNAV_VERSION}-${arch}.zip"
  url="https://github.com/tstack/lnav/releases/download/v${LNAV_VERSION}/${zip}"
  echo "Downloading lnav ${LNAV_VERSION} static binary..." >&2
  mkdir -p "$LOCAL_BIN"
  tmpdir="$(mktemp -d)"
  trap 'rm -rf "$tmpdir"' RETURN
  curl -fsSL "$url" -o "$tmpdir/$zip" || { echo "Download failed: $url" >&2; return 1; }
  # Use python3's zipfile module — `unzip` isn't always present in slim images.
  python3 -m zipfile -e "$tmpdir/$zip" "$tmpdir" || { echo "Extract failed" >&2; return 1; }
  install -m 0755 "$tmpdir/lnav-${LNAV_VERSION}/lnav" "$LNAV_BIN" || { echo "Install failed" >&2; return 1; }
  echo "Installed: $LNAV_BIN" >&2
}

if [[ -x "$LNAV_BIN" ]]; then
  export PATH="$LOCAL_BIN:$PATH"
elif command -v lnav >/dev/null 2>&1 && ! lnav_too_old; then
  : # apt lnav is recent enough
else
  if ! install_lnav; then
    echo "Failed to install lnav. Grab it manually from" >&2
    echo "  https://github.com/tstack/lnav/releases" >&2
    exit 1
  fi
  export PATH="$LOCAL_BIN:$PATH"
fi

LOG_ROOT="$HOME/.ros/log"

resolve_latest() {
  # Prefer the `latest` symlink if present; otherwise newest timestamped subdir.
  if [[ -e "$LOG_ROOT/latest" ]]; then
    echo "$LOG_ROOT/latest"
    return
  fi
  find "$LOG_ROOT" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr | head -1 | cut -d' ' -f2-
}

if [[ "${1-}" == "--" ]]; then
  shift
  RUN_DIR="$(resolve_latest)"
  EXTRA=("$@")
elif [[ $# -gt 0 && -d "$1" ]]; then
  RUN_DIR="$1"
  shift
  EXTRA=("$@")
else
  RUN_DIR="$(resolve_latest)"
  EXTRA=("$@")
fi

if [[ -z "$RUN_DIR" || ! -e "$RUN_DIR" ]]; then
  echo "No log directory found under $LOG_ROOT — start a launch first." >&2
  exit 1
fi
sync_lnav_assets() {
  # Install format + named filter scripts into ~/.lnav/formats/installed/.
  # `lnav -i` only honors a single file when the first arg is already installed,
  # so call it once per file. Each call is idempotent (writes a .bak).
  local installed="$HOME/.lnav/formats/installed"
  local f
  for f in "$FORMATS_DIR"/*.json "$FORMATS_DIR"/scripts/*.lnav; do
    [[ -e "$f" ]] || continue
    local dst="$installed/$(basename "$f")"
    if [[ ! -e "$dst" || "$f" -nt "$dst" ]]; then
      lnav -i "$f" >/dev/null 2>&1 || true
    fi
  done
}
sync_lnav_assets

echo "Opening logs from: $RUN_DIR" >&2
echo "Filter presets pre-loaded. Press Tab in lnav, then Space to toggle any filter." >&2

# Tail-follow by default — most runs are looked at live. Pass any flag in EXTRA
# (e.g. `-n` for headless) to disable follow.
exec lnav -t -I "$FORMATS_DIR" -c '|swarm-presets' "${EXTRA[@]}" "$RUN_DIR"
