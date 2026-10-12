#!/usr/bin/env bash

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# UBC Thunderbots Software Minimal Setup Script
#
# Installs everything into /opt/tbots/, to undo this install script, just run:
#    sudo rm -rf /opt/tbots/
#
# Requirements: wget
#
# NOTE: Currently WIP, only partial support for macOS (ARM64) and Ubuntu (AMD64)
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Exit immediately after failed command
set -euo pipefail

OS="$(uname -s)"

# Create our tbots directory
mkdir -p /opt/tbots/bin
chown -R $USER /opt/tbots

echo 'export PATH="/opt/tbots/bin:$PATH"' > /opt/tbots/bin/activate

# Install bazelisk
if [ "$OS" = "Darwin" ]; then
     wget -O /opt/tbots/bin/bazel https://github.com/bazelbuild/bazelisk/releases/download/v1.29.0/bazelisk-darwin-arm64
elif [ "$OS" = "Linux" ]; then
     wget -O /opt/tbots/bin/bazel https://github.com/bazelbuild/bazelisk/releases/download/v1.29.0/bazelisk-linux-amd64
fi
chmod +x /opt/tbots/bin/bazel

# Install SSL gamecontroller
if [ "$OS" = "Darwin" ]; then
     wget -O /opt/tbots/bin/gamecontroller \
          https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.23.0/ssl-game-controller_v3.23.0_darwin_arm64
elif [ "$OS" = "Linux" ]; then
     wget -O /opt/tbots/bin/gamecontroller \
          https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.23.0/ssl-game-controller_v3.23.0_linux_amd64
fi
chmod +x /opt/tbots/bin/gamecontroller

echo "Setup successful!"
echo "source /opt/tbots/bin/activate"
