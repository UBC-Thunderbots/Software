#!/usr/bin/env bash
# NOTE: currently script only for macOS

# requirements: homebrew

# System packages
brew update
system_packages=(
    bazelisk    # bazel is our build system
)

for pkg in "${host_software_packages[@]}"; do
	  brew install "$pkg"
done

# Sandbox
sudo mkdir /opt/tbots
sudo chown -R $USER /opt/tbots
mkdir /opt/tbots/bin

# SSL gamecontroller
curl -L https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.23.0/ssl-game-controller_v3.23.0_darwin_arm64 \
     -o /opt/tbots/bin/gamecontroller
chmod +x /opt/tbots/bin/gamecontroller
