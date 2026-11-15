#!/usr/bin/env bash
# Launcher for the tbots CLI (//cli:tbots), run with Bazel's hermetic Python.
set -euo pipefail
cd "$(dirname "$0")"

# --output_base: separate Bazel server, so the CLI doesn't clear the main build's cache
BAZEL=(bazel --quiet --output_base="${HOME}/.cache/bazel/tbots_cli")

# --symlink_prefix=/: don't overwrite the main bazel-bin/bazel-out symlinks in src/
# --ui_event_filters, --noshow_progress: hide Bazel's output
FLAGS=(--symlink_prefix=/ --ui_event_filters=-info,-stdout,-stderr --noshow_progress)

# First build can take some time, prints a message that it is running if it takes over 3s
(sleep 3; echo "tbots.py: building the tbots CLI, this can take a few minutes the first time..." >&2) &
notice_pid=$!
trap 'kill "$notice_pid" 2>/dev/null || true' EXIT
"${BAZEL[@]}" build "${FLAGS[@]}" //cli:tbots
kill "$notice_pid" 2>/dev/null || true
trap - EXIT

exec "${BAZEL[@]}" run "${FLAGS[@]}" //cli:tbots -- "$@"
