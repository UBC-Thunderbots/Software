#!/usr/bin/env bash

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# UBC Thunderbots Software Setup
#
# This script will install all required libraries and dependencies to build
# and run the Thunderbots codebase including the AI and unit tests.
#
# Our codebase currently supports macOS Arm, and Ubuntu 24.04 Intel and Arm.
#
# The setup is split into numbered modules in modules/, which are run in order.
# Everything platform specific lives behind two extension points:
#
#   lib/pkg/<family>.sh          maps logical dependencies to real packages
#   lib/providers/<family>.sh    overrides portable install routines
#
# Supporting another operating system means adding one file to each of those
# directories and one branch in lib/os.sh; the modules stay unchanged.
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Exit immediately after failed command
set -euo pipefail

# Save parent directory of this setup script
SETUP_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -P "$SETUP_DIR/.." && pwd)"
cd "$SETUP_DIR" || exit

source "$SETUP_DIR/lib/utils.sh"
source "$SETUP_DIR/lib/os.sh"
source "$SETUP_DIR/lib/pkg.sh"
source "$SETUP_DIR/lib/python.sh"
source "$SETUP_DIR/lib/udev.sh"
source "$SETUP_DIR/lib/providers.sh"

# Sources rather than executes, so that modules inherit the helpers above.
run_module() {
  local script="$1"
  log_info "========== $(basename "$script") =========="
  source "$script"
}

check_os_and_arch
load_pkg_backend
load_providers
init_download_cache

for module in "$SETUP_DIR"/modules/*.sh; do
  run_module "$module"
done
