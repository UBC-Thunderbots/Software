#!/usr/bin/env bash

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# UBC Thunderbots Software Setup
#
# This script will install all required libraries and dependencies to build
# and run the Thunderbots codebase including the AI and unit tests.
#
# Our codebase currently supports macOS Arm, and Ubuntu 24.04 Intel and Arm.
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Exit immediately after failed command
set -euo pipefail

# # Ensure root/sudo privileges
# if [ "$EUID" -ne 0 ]; then
#   echo "Error: This setup script must be run with sudo or as root."
#   exit 1
# fi

# Save parent directory of this setup script
SETUP_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SETUP_DIR" || exit

source "$SETUP_DIR/lib/utils.sh"
source "$SETUP_DIR/lib/os.sh"

check_os_and_arch

run_module() {
  local script="$1"
  log_info "Running module: $(basename "$script")"
  bash "$script"
}

run_module "$SETUP_DIR/modules/1-system-packages.sh"
run_module "$SETUP_DIR/modules/2-tbotspython-venv.sh"
run_module "$SETUP_DIR/modules/3-ssl-software.sh"
run_module "$SETUP_DIR/modules/4-bazel.sh"
run_module "$SETUP_DIR/modules/5-cross-compilers.sh"
run_module "$SETUP_DIR/modules/6-platformio.sh"
run_module "$SETUP_DIR/modules/7-misc.sh"
