# Provider layer.
#
# A provider implements the platform specific parts of installing the software
# we depend on. lib/providers/default.sh is loaded first and holds the
# implementations that are portable across every supported platform; a provider
# named after the platform family is then loaded on top of it and overrides
# anything it needs to.
#
# Adding a new operating system means optionally adding
# lib/providers/<family>.sh. Any function left undefined falls back to the
# portable implementation.

load_providers() {
  # shellcheck source=/dev/null
  . "$SETUP_DIR/lib/providers/default.sh"

  local provider="$SETUP_DIR/lib/providers/$OS_FAMILY.sh"
  if [ -r "$provider" ]; then
    # shellcheck source=/dev/null
    . "$provider"
    log_info "Using platform provider: $OS_FAMILY"
  fi
}
