#!/usr/bin/env bash

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# UBC Thunderbots Software Minimal Setup Script
#
# Installs everything into /opt/tbots/, to undo this install script, just run:
#    sudo rm -rf /opt/tbots/
#
# Requirements: wget, unzip
#
# NOTE: Currently WIP, only partial support for macOS (ARM64) and Ubuntu (AMD64)
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

# Install SSL gamecontroller
wget -O /opt/tbots/bin/gamecontroller "https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.23.0/ssl-game-controller_v3.23.0_${OS}_${ARCH}"
chmod +x /opt/tbots/bin/gamecontroller

# Install TIGERS AutoReferee
wget -O /opt/tbots/autoReferee.zip https://github.com/TIGERs-Mannheim/AutoReferee/releases/download/1.7.1/autoReferee.zip
unzip /opt/tbots/autoReferee.zip -d /opt/tbots/
rm /opt/tbots/autoReferee.zip

echo "Setup successful!"
echo "source /opt/tbots/bin/activate"
