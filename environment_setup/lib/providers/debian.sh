# Debian / Ubuntu platform provider.

# The system packages are installed system wide, so the virtual environment has
# to see them. src/software/thunderscope imports PyQt6 through them.
venv_args() {
  echo "--system-site-packages"
}

python_interpreter() {
  echo "/usr/bin/python${PYTHON_VERSION}"
}

# TIGERS AutoRef is not published as a Linux binary, so it is built from source.
# The build needs a JDK, which the Oracle release is downloaded for.
install_java() {
  local archive="$DOWNLOAD_CACHE/jdk-21.tar.gz"

  fetch "https://download.oracle.com/java/21/latest/jdk-21_linux-${JAVA_ARCH}_bin.tar.gz" "$archive"
  tar -xzf "$archive" -C "$VENV_DIR"
  as_root rm -rf "$VENV_DIR/bin/jdk"
  as_root mv "$VENV_DIR"/jdk-21* "$VENV_DIR/bin/jdk"
  rm -f "$archive"
}

install_autoref() {
  local commit="b30660b78728c3ce159de8ae096181a1ec52e9ba"
  local archive="$DOWNLOAD_CACHE/AutoReferee.zip"
  local source_dir="$DOWNLOAD_CACHE/AutoReferee-${commit}"

  fetch "https://github.com/TIGERs-Mannheim/AutoReferee/archive/${commit}.zip" "$archive"
  unzip -q -o -d "$DOWNLOAD_CACHE" "$archive"
  chmod +x "$source_dir/gradlew"

  "$source_dir/gradlew" installDist -p "$source_dir" -Dorg.gradle.java.home="$VENV_DIR/bin/jdk"
  as_root rm -rf "$VENV_DIR/autoReferee"
  as_root mv "$source_dir/build/install/autoReferee" "$VENV_DIR/"
  rm -rf "$source_dir" "$archive"
}

# A release binary is published for Linux, so no build toolchain is needed.
install_game_controller() {
  local version="v3.16.1"
  local binary="$DOWNLOAD_CACHE/gamecontroller"

  fetch "https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/${version}/ssl-game-controller_${version}_linux_${GO_ARCH}" "$binary"
  as_root install -m 0755 "$binary" "$VENV_DIR/gamecontroller"
  rm -f "$binary"
}

install_platformio() {
  local installer="$DOWNLOAD_CACHE/get-platformio.py"

  fetch "https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py" "$installer"
  # Not run as root: the installer writes to the invoking user's home directory
  # and that is the copy Bazel needs to find.
  "$(python_interpreter)" "$installer"

  as_root mkdir -p /usr/local/bin
  # Link into /usr/local/bin so that bazel can find it
  force_symlink "$HOME/.platformio/penv/bin/platformio" /usr/local/bin/platformio
  rm -f "$installer"
}

install_udev_rules() {
  local rules="$DOWNLOAD_CACHE/99-platformio-udev.rules"

  fetch "https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules" "$rules"
  as_root install -m 0644 "$rules" /etc/udev/rules.d/99-platformio-udev.rules
  rm -f "$rules"

  refresh_udev
}

# This is required because a Braille TTY device that Linux provides a driver
# for conflicts with the ESP32. The rule is activated when module 6 restarts udev.
install_brltty_udev_rule() {
  local rule="$DOWNLOAD_CACHE/85-brltty.rules"

  fetch "https://raw.githubusercontent.com/UBC-Thunderbots/Software-External-Dependencies/main/85-brltty.rules" "$rule"
  as_root install -m 0644 "$rule" /usr/lib/udev/rules.d/85-brltty.rules
  rm -f "$rule"
}

grant_serial_port_access() {
  as_root usermod -a -G dialout "$(id -un)"
}

print_setup_complete_notice() {
  log_info "Done Software Setup, please reboot for changes to take place"
}
