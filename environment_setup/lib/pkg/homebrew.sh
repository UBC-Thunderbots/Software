# macOS package backend, backed by Homebrew.

pkg_prepare_repositories() {
  brew update
}

pkg_install_names() {
  local name

  for name in "$@"; do
    if brew list "$name" &>/dev/null; then
      log_info "$name already installed, skipping..."
    else
      log_info "Installing $name..."
      brew install "$name"
    fi
  done
}

# Maps a logical dependency to the Homebrew formulae that provide it. Packages
# that macOS already ships, such as curl, git and unzip, are intentionally left
# unmapped so that Homebrew does not shadow the system versions.
#
# The cmake and clang-format entries are deliberately unversioned: Homebrew
# retires versioned formulae once they fall out of support, and cmake@4 and
# clang-format@20 have both already been removed. The unversioned formulae track
# the current releases, and the compiler is still modern enough for the
# repository's .clang-format.
pkg_resolve() {
  case "$1" in
    cmake)                echo "cmake" ;;
    clang-format)         echo "clang-format" ;;
    codespell)            echo "codespell" ;;
    sshpass)              echo "sshpass" ;;
    jdk)                  echo "openjdk@21" ;;
    bazel)                echo "bazelisk" ;;
    python)               echo "python@3.12" ;;
    python-dev)           echo "python@3.12" ;;
    python-venv)          echo "python@3.12" ;;
    pyqt)                 echo "pyqt@6 qt@6" ;;
    node)                 echo "node@20" ;;
    go)                   echo "go@1.24" ;;
    cross-toolchain)      echo "messense/macos-cross-toolchains/aarch64-unknown-linux-gnu" ;;
    *)                    return 1 ;;
  esac
}
