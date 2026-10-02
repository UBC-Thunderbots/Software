# Package manager abstraction.
#
# Modules never name a concrete package. They ask for a logical dependency such
# as "clang-format" and the backend for the current platform decides what that
# actually maps to. Adding support for a new operating system therefore means
# writing one file in lib/pkg/ and nothing else.
#
# A backend must define:
#   pkg_prepare_repositories  add package sources and refresh the package index
#   pkg_install_names         install the given concrete package names
#   pkg_resolve               map a logical dependency to concrete package names,
#                             returning non-zero when the platform has no equivalent
#
# Backends may additionally define venv_args and python_interpreter in
# lib/providers/, which control how the virtual environment is created.

load_pkg_backend() {
  local backend="$SETUP_DIR/lib/pkg/$OS_FAMILY.sh"

  if [ ! -r "$backend" ]; then
    fail "No package backend for platform family '$OS_FAMILY' (expected $backend)."
  fi

  # shellcheck source=/dev/null
  . "$backend"
  log_info "Using package backend: $OS_FAMILY"
}

# Delegates to the backend, guarding against an incomplete backend.
pkg_delegate() {
  local function="$1"
  shift

  if ! declare -F "$function" >/dev/null; then
    fail "The '$OS_FAMILY' package backend does not implement $function()."
  fi

  "$function" "$@"
}

# Adds the package sources the platform needs and refreshes the package index.
pkg_prepare() {
  pkg_delegate pkg_prepare_repositories
}

# Installs logical dependencies, reporting the ones this platform has no
# equivalent for. Use this for dependencies that are useful but not essential.
pkg_install() {
  local logical

  for logical in "$@"; do
    if ! pkg_install_logical "$logical"; then
      log_skip "'$logical' has no $OS_FAMILY equivalent"
    fi
  done
}

# Installs logical dependencies and aborts when the platform cannot provide one.
# Use this for dependencies the codebase cannot be built or tested without.
pkg_require() {
  local logical

  for logical in "$@"; do
    if ! pkg_install_logical "$logical"; then
      fail "Required dependency '$logical' has no $OS_FAMILY equivalent."
    fi
  done
}

pkg_install_logical() {
  local logical="$1"
  local names
  local -a name_list=()

  names="$(pkg_delegate pkg_resolve "$logical")" || return 1
  [ -n "$names" ] || return 1

  read -r -a name_list <<<"$names"
  log_info "Installing $logical ($names)"
  pkg_delegate pkg_install_names "${name_list[@]}"
}
