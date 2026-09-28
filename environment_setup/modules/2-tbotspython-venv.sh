#!/usr/bin/env bash

# The virtual environment at /opt/tbotspython that Bazel and our runtime
# scripts expect, plus the directory Bazel uses for external runtimes.

log_info "Setting Up Virtual Python Environment"
create_tbotspython_venv
log_info "Done Setting Up Virtual Python Environment"

prepare_external_runtimes
