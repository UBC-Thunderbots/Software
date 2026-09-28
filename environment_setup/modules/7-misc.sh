#!/usr/bin/env bash

# Remaining tooling that the robot deployment scripts rely on.

log_info "Set up ansible-lint"
# Deliberately not run as root: ansible-galaxy installs into the invoking
# user's home directory, which is where the deployment tooling looks.
"$(venv_bin ansible-galaxy)" collection install ansible.posix
log_info "Finished setting up ansible-lint"

print_setup_complete_notice
