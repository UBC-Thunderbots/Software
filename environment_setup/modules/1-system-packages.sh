#!/usr/bin/env bash

# Platform validation, package sources and the system level dependencies.
#
# This module only names logical dependencies. Which packages those turn into is
# decided by the backend in lib/pkg/ for the detected platform, so this file
# never has to mention Ubuntu, macOS or Arch.
#
# pkg_require aborts when the platform has no equivalent, so it is only used for
# dependencies the codebase cannot be built, linted or tested without.

require_supported_platform
setup_git_hooks

pkg_prepare

pkg_require cmake clang-format codespell sshpass
pkg_require python python-dev python-venv

# Provided by the operating system on some platforms, or only needed for parts
# of the toolchain such as profiling and cross compilation.
pkg_install git curl unzip openssl sqlite3-dev ffi-dev ssl-dev eigen valgrind
pkg_install python-pip python-yaml
pkg_install gcc g++ kcachegrind libstdcxx-dbg jdk xvfb
pkg_install udev-dev usb-dev xcb-cursor pyqt
pkg_install bazel node go cross-toolchain

install_brltty_udev_rule

# WSL has no OpenGL driver of its own, so the thunderscope GUI needs this.
if is_wsl; then
  log_info "Detected WSL, installing its OpenGL runtime"
  pkg_install libopengl0
fi
