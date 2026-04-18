#!/bin/bash

# Configuration
AUTH_KEYS="$HOME/.ssh/authorized_keys"
SERVER_SCRIPT="./mezzsh_server.sh"

if [ "$#" -ne 2 ]; then
    echo "Usage: sudo ./register_user.sh \"PUBLIC_KEY_STRING\""
    exit 1
fi

PUB_KEY="$1"

echo "--- Registering New Remote User ---"

# Add to authorized_keys with a command restriction
# We prepend the SERVER_SCRIPT command to the key
ENTRY="command=\"$SERVER_SCRIPT\",environment=\"SSH_CHECK_MODE=1 FORCE_CONNECT=1\" $PUB_KEY"

if grep -q "$PUB_KEY" "$AUTH_KEYS"; then
    echo "Error: This public key is already registered."
else
    echo "$ENTRY" >> "$AUTH_KEYS"
    chmod 600 "$AUTH_KEYS"
    echo "Key added to authorized_keys."
fi

echo "Registration complete."
