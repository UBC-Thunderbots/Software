# Arch Linux platform provider.
#
# Only the pieces that differ from the portable implementations live here. The
# udev rules and the serial port group follow the same steps as Debian, so they
# are pulled in from the Debian provider rather than duplicated.

# shellcheck source=providers/debian.sh
. "$SETUP_DIR/lib/providers/debian.sh"

python_interpreter() {
  echo "/usr/bin/python${PYTHON_VERSION}"
}
