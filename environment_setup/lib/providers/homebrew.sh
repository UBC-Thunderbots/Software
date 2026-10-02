# macOS platform provider.

# AutoRef is published as a release archive for macOS, so it needs neither a JDK
# nor a build step. The portable install_java handles that.
install_autoref() {
  local version="1.5.5"
  local archive="$DOWNLOAD_CACHE/autoReferee.zip"

  fetch "https://github.com/TIGERs-Mannheim/AutoReferee/releases/download/${version}/autoReferee.zip" "$archive"
  unzip -q -o -d "$DOWNLOAD_CACHE" "$archive"

  as_root rm -rf "$VENV_DIR/autoReferee"
  as_root mv "$DOWNLOAD_CACHE/autoReferee" "$VENV_DIR/"
  rm -rf "$archive"
}

# No release binary is published for macOS, so the game controller is built from
# source with the Go toolchain from the package manager.
install_game_controller() {
  local version="v3.17.0"
  local archive="$DOWNLOAD_CACHE/ssl-game-controller.zip"
  local source_dir="$DOWNLOAD_CACHE/ssl-game-controller-3.17.0"

  fetch "https://github.com/RoboCup-SSL/ssl-game-controller/archive/refs/tags/${version}.zip" "$archive"
  unzip -q -o -d "$DOWNLOAD_CACHE" "$archive"

  (
    cd "$source_dir"
    make install
    go build -o main ./cmd/ssl-game-controller
  )

  as_root install -m 0755 "$source_dir/main" "$VENV_DIR/gamecontroller"
  rm -rf "$source_dir" "$archive"
}

# platformio is installed into the virtual environment through
# environment_setup/requirements.txt, so only the symlink Bazel looks for is
# missing here.
install_platformio() {
  as_root mkdir -p /usr/local/bin
  force_symlink "$(venv_bin platformio)" /usr/local/bin/platformio
}

print_setup_complete_notice() {
  log_info "Software Setup Complete"
  log_info "Note: Some changes require a new terminal session to take effect"
}
