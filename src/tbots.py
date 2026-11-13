#!/usr/bin/env bash
# Launcher for the tbots CLI (//cli:tbots), run with Bazel's hermetic Python.
#
# The CLI runs in its own Bazel server (a separate --output_base) so that its
# build options don't discard the analysis cache of the main Bazel server,
# which the CLI's own bazel commands use.
set -euo pipefail

cd "$(dirname "$0")"

exec bazel --quiet --output_base="${HOME}/.cache/bazel/tbots_cli" run \
    --symlink_prefix=/ \
    --ui_event_filters=-info,-stdout,-stderr \
    --noshow_progress \
    //cli:tbots -- "$@"
