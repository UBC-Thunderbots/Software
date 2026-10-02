# udev helpers.
#
# udev only exists on Linux, so every function here is a no-op elsewhere. That
# lets the modules call them unconditionally.

refresh_udev() {
  if ! is_linux; then
    return 0
  fi

  if command -v systemctl >/dev/null 2>&1; then
    # udev is frequently unavailable inside containers, which is not fatal.
    if ! as_root systemctl restart udev 2>/dev/null; then
      log_warn "Could not restart udev; the rules will apply on the next reboot."
    fi
  elif command -v service >/dev/null 2>&1; then
    if ! as_root service udev restart 2>/dev/null; then
      log_warn "Could not restart udev; the rules will apply on the next reboot."
    fi
  fi
}
