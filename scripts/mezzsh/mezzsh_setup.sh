#!/bin/bash

SERVER_SCRIPT="/home/thunderbots/Software/scripts/mezzsh/mezzsh_server.sh"
TIMEOUT_SECONDS=3600  # 1 hour
SSHD_CONFIG="/etc/ssh/sshd_config"
TARGET_USER="thunderbots"

# Check for root privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)"
   exit 1
fi

echo "--- Initializing On-Site PC SSH Security Setup ---"

echo "[1/5] Installing Dependencies"
sudo apt update
sudo apt install openssh-server
sudo apt install zenity

# Install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --hostname=$TARGET_USER --operator=$USER

echo "[2/5] Configuring SSH service to start on boot..."
systemctl enable --now ssh
systemctl start ssh

echo "[3/5] Modifying sshd_config for custom Environment variables, and to perform safety checks on new connections......"

# Backup original config
cp $SSHD_CONFIG "${SSHD_CONFIG}.bak"

# This step uses a match block to modify these ssh settings for only 1 user
# Clean up any previous global ForceCommand we might have added
sed -i '/Match User $TARGET_USER/,/AcceptEnv SSH_CHECK_MODE FORCE_CONNECT/d' $SSHD_CONFIG

# Append the Match block to the end of the file
cat <<EOF >> $SSHD_CONFIG
Match User $TARGET_USER
    ForceCommand $SERVER_SCRIPT
    AcceptEnv SSH_CHECK_MODE FORCE_CONNECT
EOF

echo "[4/5] Setting 1-hour shell timeout..."

USER_HOME=$(eval echo "~$TARGET_USER")
TIMEOUT_BLOCK='if [ -n "$SSH_TTY" ]; then export TMOUT='$TIMEOUT_SECONDS' && readonly TMOUT; fi'

if [ -d "$USER_HOME" ]; then
    # Remove old TMOUT lines if they exist and append new one
    sed -i '/TMOUT/d' "$USER_HOME/.bashrc"
    echo $TIMEOUT_BLOCK >> "$USER_HOME/.bashrc"
    chown $TARGET_USER:$TARGET_USER "$USER_HOME/.bashrc"
    echo "Success: SSH timeout set for $TARGET_USER."
else
    echo "ERROR: Home directory for $TARGET_USER not found. Timeout not set."
fi

# 4. Finalizing and Restarting Service
echo "[5/5] Restarting SSH service to apply changes..."
systemctl restart ssh
