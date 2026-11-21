#!/usr/bin/env bash

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# UBC Thunderbots Software Minimal Setup Script
#
# Installs everything into /opt/tbots/, to undo this install script, just run:
#    sudo rm -rf /opt/tbots/
#
# Requirements: wget, unzip
#
# NOTE: Only officially supports macOS (ARM64) and Ubuntu 24.04 (ARM64/AMD64)
#       Any other OS, OS Version, Architecture may work, but not tested in CI.
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Exit immediately after failed command
set -euo pipefail

OS=$(uname -s | tr '[:upper:]' '[:lower:]')
ARCH=$(uname -m | tr '[:upper:]' '[:lower:]')

case "$ARCH" in
     x86_64|amd64)
          ARCH="amd64"
          ;;
     aarch64|arm64)
          ARCH="arm64"
          ;;
     *)
          echo "Unsupported architecture: $ARCH" >&2
          exit 1
          ;;
esac

# Create our tbots directory
mkdir -p /opt/tbots/bin
chown -R $SUDO_USER /opt/tbots

echo 'export PATH="/opt/tbots/bin:$PATH"' > /opt/tbots/bin/activate

# Install bazelisk
wget -O /opt/tbots/bin/bazel "https://github.com/bazelbuild/bazelisk/releases/download/v1.29.0/bazelisk-${OS}-${ARCH}"
chmod +x /opt/tbots/bin/bazel

echo "Setup successful!"
echo "source /opt/tbots/bin/activate"
