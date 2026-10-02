# Colors
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
  echo -e "${BLUE}[INFO]${NC} $1"
}

log_warn() {
  echo -e "${YELLOW}[WARN]${NC} $1" >&2
}

log_error() {
  echo -e "${RED}[ERROR]${NC} $1" >&2
}

log_skip() {
  log_info "Skipping: $1"
}

fail() {
  log_error "$1"
  exit 1
}

# Run a command with root privileges. Uses sudo only when the setup is not
# already running as root so that the same script works in CI and containers.
as_root() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    sudo "$@"
  fi
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    fail "Required command '$1' was not found in PATH."
  fi
}

# Download $1 into the file $2. curl is preferred because it is present on every
# supported platform, but wget is accepted as a fallback.
fetch() {
  local url="$1"
  local dest="$2"

  if command -v curl >/dev/null 2>&1; then
    curl -fsSL "$url" -o "$dest"
  elif command -v wget >/dev/null 2>&1; then
    wget -q "$url" -O "$dest"
  else
    fail "Neither curl nor wget is available to download $url."
  fi
}

# Replace the symlink at $1 so that re-running the setup is idempotent.
force_symlink() {
  local target="$1"
  local link="$2"

  as_root rm -rf "$link"
  as_root ln -s "$target" "$link"
}
