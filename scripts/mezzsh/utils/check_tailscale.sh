#!/bin/bash

# Checks if Tailscale is installed, the daemon is up and running, and we are logged in (connected) to the VPN

# Check if tailscale is installed
if ! command -v tailscale &> /dev/null; then
    echo "Tailscale not found. Installing..."
    curl -fsSL https://tailscale.com/install.sh | sh
fi

# Check if the Tailscale daemon (tailscaled) is even running
if ! systemctl is-active --quiet tailscaled; then
    echo "tailscaled is not running. Starting service..."
    sudo systemctl start tailscaled
fi

# Check if the node is authenticated and connected to the tailnet
# 'tailscale status' returns 0 if connected, non-zero otherwise
if ! tailscale status >/dev/null 2>&1; then
    echo "Tailscale is down or unauthenticated. Running 'up'..."

    read -p "Enter the auth key. Please contact a software lead if you don't have one: " AUTH_KEY

    sudo tailscale up --auth-key=$AUTH_KEY
fi