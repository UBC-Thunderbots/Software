#!/usr/bin/env bash

# Bazel, and the clang-format symlink that scripts/lint_and_format.sh expects
# to find inside the virtual environment.

pkg_install bazel
log_info "Installing Bazel"
install_bazel
log_info "Done Installing Bazel"

log_info "Installing clang-format"
install_clang_format
log_info "Done installing clang-format"
