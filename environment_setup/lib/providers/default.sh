# Portable install routines.
#
# These are the implementations that work on every supported platform. Anything
# that genuinely differs per operating system is overridden by
# lib/providers/<family>.sh, which is loaded after this file.
#
# The layout under /opt/tbotspython is referenced by src/MODULE.bazel and
# src/toolchains/cc/BUILD, so the paths below must not change.

PYTHON_VERSION="3.12"
VENV_DIR="/opt/tbotspython"
DOWNLOAD_CACHE="/tmp/tbots_download_cache"

# Interpreter used to build the virtual environment and to run the toolchain
# steps. Overridden where the system interpreter is not on PATH.
python_interpreter() {
  echo "python${PYTHON_VERSION}"
}

# Absolute path to the same interpreter, for the places that need one.
python_interpreter_path() {
  local interpreter
  interpreter="$(command -v "$(python_interpreter)" || true)"
  echo "${interpreter:-$(python_interpreter)}"
}

# Extra arguments passed to "python -m venv".
venv_args() {
  :
}

venv_bin() {
  echo "$VENV_DIR/bin/$1"
}

# Header directory of the interpreter that the virtual environment is built
# from, which is what the Bazel Python toolchain expects.
python_include_dir() {
  "$(python_interpreter)" -c 'import sysconfig; print(sysconfig.get_paths()["include"])'
}

init_download_cache() {
  # Files in the cache are unpacked as the invoking user, so the directory has
  # to be owned by that user even when a previous run created it as root.
  as_root rm -rf "$DOWNLOAD_CACHE"
  mkdir -p "$DOWNLOAD_CACHE"
}

install_bazel() {
  if command -v bazel >/dev/null 2>&1; then
    log_skip "bazel is already available"
    return 0
  fi

  local version="v1.26.0"
  local archive="$DOWNLOAD_CACHE/bazel"

  fetch "https://github.com/bazelbuild/bazelisk/releases/download/${version}/bazelisk-${BAZEL_OS}-${BAZEL_ARCH}" "$archive"
  as_root install -m 0755 "$archive" /usr/bin/bazel
  rm -f "$archive"
}

# lint_and_format.sh expects clang-format inside the virtual environment. The
# versioned Debian name and the Homebrew name both resolve through PATH.
install_clang_format() {
  local binary

  for binary in clang-format-14 clang-format; do
    if command -v "$binary" >/dev/null 2>&1; then
      force_symlink "$(command -v "$binary")" "$(venv_bin clang-format)"
      return 0
    fi
  done

  fail "No clang-format binary found. Install the 'clang-format' dependency first."
}

# Cross compiler used to build our own robot software. Linux uses the pinned
# toolchain release; platforms that ship one through their package manager
# override this.
install_cross_compiler() {
  if ! is_linux; then
    log_skip "cross compiler is provided by the $OS_FAMILY package manager"
    return 0
  fi

  local commit="4d595097dd14104dbd6ca1f42ad8c8fdcb68db6b"
  local package="aarch64-tbots-linux-gnu-for-${TBOTS_TOOLCHAIN_ARCH}"
  local archive="$DOWNLOAD_CACHE/${package}.tar.xz"

  fetch "https://raw.githubusercontent.com/UBC-Thunderbots/Software-External-Dependencies/${commit}/toolchain/${package}.tar.xz" "$archive"
  tar -xf "$archive" -C "$DOWNLOAD_CACHE"
  as_root rm -rf "$VENV_DIR/aarch64-tbots-linux-gnu"
  as_root mv "$DOWNLOAD_CACHE/aarch64-tbots-linux-gnu" "$VENV_DIR"
  rm -f "$archive"
}

# Cross compilation headers for the aarch64 robot software. Only meaningful on
# Linux hosts, which have the aarch64 sysroot the configure step needs.
install_python_cross_compile_headers() {
  if ! is_linux; then
    log_skip "cross compile headers are not used on $OS"
    return 0
  fi

  local source_dir="Python-3.12.0"
  local archive="$DOWNLOAD_CACHE/python-${PYTHON_VERSION}.tar.xz"

  fetch "https://www.python.org/ftp/python/${PYTHON_VERSION}/${source_dir}.tar.xz" "$archive"
  tar -xf "$archive" -C "$DOWNLOAD_CACHE"
  (
    cd "$DOWNLOAD_CACHE/$source_dir"

    # The configuration is taken from the examples provided in
    # https://docs.python.org/3.12/using/configure.html
    echo ac_cv_buggy_getaddrinfo=no >config.site-aarch64
    echo ac_cv_file__dev_ptmx=yes >>config.site-aarch64
    echo ac_cv_file__dev_ptc=no >>config.site-aarch64

    CONFIG_SITE=config.site-aarch64 ./configure \
      --build="$HOST_TRIPLE" \
      --host="$TARGET_TRIPLE" \
      "-with-build-python=$(python_interpreter_path)" \
      --enable-optimizations \
      --prefix="$VENV_DIR/cross_compile_headers" >/dev/null
    make inclinstall -j"$(getconf _NPROCESSORS_ONLN)" >/dev/null
  )
  rm -rf "$DOWNLOAD_CACHE/$source_dir" "$archive"
}

install_python_toolchain_headers() {
  local include
  include="$(python_include_dir)"

  as_root mkdir -p "$VENV_DIR/py_headers/include"
  force_symlink "$include" "$VENV_DIR/py_headers/include/python${PYTHON_VERSION}"
}

# Cross compiler for the motor board firmware.
install_stm32_cross_compiler() {
  local version="14.3.rel1"
  local toolchain="arm-gnu-toolchain-${version}-${STM32_ARCH}-arm-none-eabi"
  local archive="$DOWNLOAD_CACHE/arm-gnu-toolchain.tar.xz"

  fetch "https://developer.arm.com/-/media/Files/downloads/gnu/${version}/binrel/${toolchain}.tar.xz" "$archive"
  tar -xf "$archive" -C "$DOWNLOAD_CACHE"
  as_root rm -rf "$VENV_DIR/arm-none-eabi-gcc"
  as_root mv "$DOWNLOAD_CACHE/$toolchain" "$VENV_DIR/arm-none-eabi-gcc"
  rm -f "$archive"
}

install_java() {
  log_skip "TIGERS AutoRef uses a prebuilt release, no JDK download needed"
}

install_game_controller() {
  fail "No game controller implementation for '$OS_FAMILY'. Add lib/providers/$OS_FAMILY.sh."
}

install_autoref() {
  fail "No AutoRef implementation for '$OS_FAMILY'. Add lib/providers/$OS_FAMILY.sh."
}

install_platformio() {
  fail "No PlatformIO implementation for '$OS_FAMILY'. Add lib/providers/$OS_FAMILY.sh."
}

# Point the AutoRef install at this repository's field geometry.
configure_autoref() {
  as_root chmod +x "$REPO_ROOT/src/software/autoref/run_autoref.sh"
  as_root install -m 0644 "$REPO_ROOT/src/software/autoref/DIV_B.txt" \
    "$VENV_DIR/autoReferee/config/geometry/DIV_B.txt"
}

# Wire up the repository's commit hooks, which the old setup scripts enabled
# implicitly by running git config from inside the checkout.
setup_git_hooks() {
  if ! (cd "$REPO_ROOT" && git rev-parse --git-dir >/dev/null 2>&1); then
    log_skip "$REPO_ROOT is not a git checkout, leaving git hooks alone"
    return 0
  fi

  (cd "$REPO_ROOT" && git config core.hooksPath "$REPO_ROOT/scripts/githooks")
}

install_udev_rules() {
  log_skip "udev rules are a Linux concept"
}

install_brltty_udev_rule() {
  log_skip "the brltty device is a Linux concept"
}

grant_serial_port_access() {
  log_skip "serial port groups are a Linux concept"
}

print_setup_complete_notice() {
  log_info "Software setup complete. Some changes require a new terminal session to take effect."
}
