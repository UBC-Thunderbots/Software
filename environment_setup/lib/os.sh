# Platform detection.
#
# Populates the following variables and is the only place that needs to change
# when support for a new operating system or architecture is added:
#
#   OS           uname -s output (Darwin, Linux)
#   OS_FAMILY    package manager family, e.g. debian, homebrew, archlinux
#   OS_ID        /etc/os-release ID, e.g. ubuntu
#   OS_VERSION   /etc/os-release VERSION_ID, e.g. 24.04
#   ARCH         uname -m output (x86_64, arm64, aarch64)
#   ARCH_NORM    ARCH normalized to x86_64 or arm64
#
# It also derives the architecture dependent names used to pick release assets,
# so that modules never have to compare architectures themselves.

# Normalizes the architecture and maps every supported operating system onto a
# package manager family.
check_os_and_arch() {
  OS="$(uname -s)"   # Darwin or Linux
  ARCH="$(uname -m)" # x86_64, arm64, or aarch64

  # Only Linux distributions carry a release file, so these stay empty elsewhere.
  OS_ID=""
  OS_VERSION=""
  ID_LIKE=""

  case "$OS" in
    Darwin) OS_FAMILY="homebrew" ;;
    Linux)  detect_linux_os_family ;;
    *)
      fail "Error: Unsupported operating system '$OS'."
      ;;
  esac

  # Normalize Architecture representation across macOS and Linux
  case "$ARCH" in
    x86_64|amd64) ARCH_NORM="x86_64" ;;
    arm64|aarch64) ARCH_NORM="arm64" ;;
    *)
      fail "Error: Unsupported architecture '$ARCH'."
      ;;
  esac

  derive_architecture_names
}

# Reads /etc/os-release and decides which package manager family owns this
# distribution. New Linux distributions are supported by adding a branch here.
detect_linux_os_family() {
  if [ -r /etc/os-release ]; then
    # shellcheck source=/dev/null
    . /etc/os-release
  fi

  OS_ID="${ID:-$OS_ID}"
  OS_VERSION="${VERSION_ID:-$OS_VERSION}"
  ID_LIKE="${ID_LIKE:-$ID_LIKE}"

  case " $OS_ID $ID_LIKE " in
    *" ubuntu "*|*" debian "*) OS_FAMILY="debian" ;;
    *" arch "*)                 OS_FAMILY="archlinux" ;;
    *)
      fail "Error: Unsupported Linux distribution '$OS_ID' (ID_LIKE='$ID_LIKE')."
      ;;
  esac
}

# Derives the architecture dependent asset names and GNU triples. Only this
# function has to grow when a new architecture becomes supported.
derive_architecture_names() {
  case "$ARCH_NORM" in
    x86_64)
      HOST_TRIPLE="x86_64-pc-linux-gnu"
      GO_ARCH="amd64"
      BAZEL_ARCH="amd64"
      JAVA_ARCH="x64"
      STM32_ARCH="x86_64"
      TBOTS_TOOLCHAIN_ARCH="x86"
      ;;
    arm64)
      HOST_TRIPLE="aarch64-pc-linux-gnu"
      GO_ARCH="arm64"
      BAZEL_ARCH="arm64"
      JAVA_ARCH="aarch64"
      STM32_ARCH="aarch64"
      TBOTS_TOOLCHAIN_ARCH="aarch64"
      ;;
  esac

  # The robot software and the motor board firmware both target aarch64 Linux.
  TARGET_TRIPLE="aarch64-unknown-linux-gnu"
  BAZEL_OS="linux"

  if [ "$OS_FAMILY" = "homebrew" ]; then
    STM32_ARCH="darwin-arm64"
    BAZEL_OS="darwin"
  fi
}

is_linux() {
  [ "$OS" = "Linux" ]
}

is_darwin() {
  [ "$OS" = "Darwin" ]
}

is_x86() {
  [ "$ARCH_NORM" = "x86_64" ]
}

is_arm64() {
  [ "$ARCH_NORM" = "arm64" ]
}

# Detects WSL, which needs a couple of extra OpenGL packages to run our GUI
# applications. See https://github.com/microsoft/WSL/issues/4071
is_wsl() {
  is_linux && grep -qi microsoft /proc/version 2>/dev/null
}

os_version_is() {
  [ "$OS_VERSION" = "$1" ]
}

# Rejects distributions that this setup has not been validated against.
require_os_version() {
  local supported="$1"

  if [ -n "$OS_VERSION" ] && ! os_version_is "$supported"; then
    fail "Error: $OS_ID $OS_VERSION is not supported, please upgrade to $OS_ID $supported."
  fi
}

# Validates the platform before any package is installed.
require_supported_platform() {
  log_info "Detected platform: $OS / $OS_ID $OS_VERSION ($ARCH_NORM)"

  case "$OS_FAMILY" in
    debian)
      require_os_version "24.04"
      ;;
    homebrew)
      require_cmd brew
      ;;
  esac
}
