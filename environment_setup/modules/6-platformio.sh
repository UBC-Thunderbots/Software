#!/usr/bin/env bash

# PlatformIO, which compiles the ESP32 code on the robots.
# https://docs.platformio.org/en/latest/core/installation.html
#
# Note: rebooting is required for the udev rules and the serial port group to
# take effect.

log_info "Setting Up PlatformIO"
install_udev_rules
grant_serial_port_access
install_platformio
log_info "Done PlatformIO Setup"
