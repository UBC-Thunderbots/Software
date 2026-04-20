#!/bin/bash

USER_HOME=$(tmp=$(getent passwd "$SUDO_USER" | cut -d: -f6); echo "${tmp:-/home/thunderbots}")

# Configuration
AUTH_KEYS="$USER_HOME/.ssh/authorized_keys"
SERVER_SCRIPT="/home/thunderbots/Software/scripts/mezzsh/mezzsh_server.sh"

if [ "$#" -ne 1 ]; then
    echo "Usage: sudo ./mezzsh_keystore.sh \"PUBLIC_KEY_STRING\""
    exit 1
fi

PUB_KEY="$1"

echo "--- Registering New Remote User ---"

# Add to authorized_keys with a command restriction
# We prepend the SERVER_SCRIPT command to the key
ENTRY="$PUB_KEY
command=\"$SERVER_SCRIPT\"
environment=\"SSH_CHECK_MODE=1 FORCE_CONNECT=1\""

if grep -q "$PUB_KEY" "$AUTH_KEYS"; then
    echo "Error: This public key is already registered."
else
    echo "$ENTRY" >> "$AUTH_KEYS"
    chmod 600 "$AUTH_KEYS"
    echo "Key added to authorized_keys."
fi

echo "Registration complete."
