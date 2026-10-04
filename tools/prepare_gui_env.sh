#!/usr/bin/env bash
# Prepare host-specific Compose variables; this script never invokes Docker.
set -euo pipefail

fail() { printf 'AutoNav setup: %s\n' "$*" >&2; exit 1; }
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"

[[ "$(uname -s)" == Linux ]] || fail 'GUI setup currently supports Linux desktops only.'
[[ "${DISPLAY:-}" =~ ^:([0-9]+)(\.[0-9]+)?$ ]] || fail 'DISPLAY must be a local X11/XWayland display (e.g. :0). Log in to a graphical desktop session.'
display_number="${BASH_REMATCH[1]}"
[[ -S "/tmp/.X11-unix/X${display_number}" ]] || fail "No X11/XWayland socket for DISPLAY=$DISPLAY. Check your desktop session and XWayland."
command -v xauth >/dev/null || fail 'Install xauth on the host: sudo apt-get install xauth'
[[ -n "${XDG_RUNTIME_DIR:-}" && -d "$XDG_RUNTIME_DIR" ]] || fail 'XDG_RUNTIME_DIR is missing; run as your logged-in desktop user, not under sudo.'
[[ "$(id -u)" != 0 ]] || fail 'Run this script without sudo, as your desktop user.'

cookies="$(xauth nlist "$DISPLAY" 2>/dev/null)" || fail 'Cannot read X11 authorization. Run xauth from your graphical desktop session.'
[[ -n "$cookies" ]] || fail "No X11 cookie for DISPLAY=$DISPLAY. Try 'xauth list' in your graphical desktop terminal."
xauth_file="$XDG_RUNTIME_DIR/autonav-docker.xauth"
umask 077
: > "$xauth_file"
printf '%s\n' "$cookies" | sed 's/^..../ffff/' | xauth -f "$xauth_file" nmerge - >/dev/null
[[ -s "$xauth_file" ]] || fail 'Failed to create the container X11 authorization file.'

render_node="${AUTONAV_RENDER_NODE:-}"
if [[ -z "$render_node" ]]; then
  for candidate in /dev/dri/renderD*; do
    if [[ -c "$candidate" ]]; then render_node="$candidate"; break; fi
  done
fi
[[ -c "$render_node" ]] || fail 'No DRM render node found. This setup requires an Intel/AMD Mesa GPU exposed at /dev/dri/renderD*.'

meshes=(src/maya_description/meshes/*.STL)
[[ -f "${meshes[0]}" ]] || fail 'No STL meshes found. Check the repository clone and run git lfs pull.'
for mesh in "${meshes[@]}"; do
  [[ -s "$mesh" ]] || fail "$mesh is empty. Run git lfs pull on the host."
  # LFS pointer files are tiny; do not scan large binary meshes as text.
  if [[ "$(stat -c %s "$mesh")" -lt 1024 ]] && grep -q '^version https://git-lfs.github.com/spec/v1$' "$mesh"; then
    fail "$mesh is a Git LFS pointer. Run git lfs pull on the host."
  fi
done

# Keep local configuration out of Git. Never place the X11 cookie itself here.
env_temp="$(mktemp "$repo_dir/.env.autonav.XXXXXX")"
trap 'rm -f "$env_temp"' EXIT
chmod 600 "$env_temp"
printf 'AUTONAV_UID=%s\nAUTONAV_GID=%s\nAUTONAV_RENDER_GID=%s\nAUTONAV_DISPLAY=%s\nAUTONAV_XAUTHORITY=%s\nAUTONAV_RENDER_NODE=%s\n' \
  "$(id -u)" "$(id -g)" "$(stat -c %g "$render_node")" "$DISPLAY" "$xauth_file" "$render_node" > "$env_temp"
mv -f "$env_temp" "$repo_dir/.env.autonav"

printf 'Host preflight passed. Prepared .env.autonav for DISPLAY=%s and %s.\n' "$DISPLAY" "$render_node"
printf 'Next: run the Docker Compose commands in Docs/docker-setup.md yourself.\n'
