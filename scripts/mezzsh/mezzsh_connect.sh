#!/bin/bash

# Connects to the Mezz PC
# Receives safety check results from the server, and warns
# the user if other users are currently connected remotely or using in person
# Allows user to force a connection

TAILSCALE_HOSTNAME="thunderbots"

bash ./utils/check_tailscale.sh

# 2. Get the Tailscale IP of the Main PC
TARGET_IP=$(tailscale ip -4 $TAILSCALE_HOSTNAME)

if [ -z "$TARGET_IP" ]; then
    echo "Could not find Main PC on Tailscale. Are you logged in?"
    exit 1
fi

# flag used to force a connection despite warnings
FORCE_FLAG="NORMAL"
SSH_TARGET="thunderbots@$TARGET_IP"

# first, check server status (if other users are using the PC)
RESPONSE=$(ssh -o SendEnv=SSH_CHECK_MODE -t $SSH_TARGET "check_status" 2>&1)

RED='\e[1;31m'
NC='\e[0m'

# someone is using IRL
if [[ "$RESPONSE" == *"STATUS_BUSY_LOCAL"* ]]; then
    echo -e "⚠️  {RED}WARNING{NC}: Someone is physically logged into the PC onsite."
    read -p "Do you want to force the connection? (y/n): " choice

    # exits if user does not want to force
    [[ "$choice" == [yY] ]] && FORCE_FLAG="FORCE" || exit 1

# someone is connected remotely
elif [[ "$RESPONSE" == *"STATUS_BUSY_REMOTE"* ]]; then
    echo -e "⚠️  {RED}WARNING{NC}: Other SSH users are connected:"
    echo "$RESPONSE" | grep "Connected:"
    read -p "Do you want to force the connection? (y/n): " choice

    # exits if user does not want to force
    [[ "$choice" == [yY] ]] && FORCE_FLAG="FORCE" || exit 1
fi

# if the user wanted to force the connection, does it here
if [ "$FORCE_FLAG" == "FORCE" ]; then
    echo "Force connecting..."
    ssh -o SendEnv=FORCE_CONNECT=1 -t $SSH_TARGET
else
    ssh -t $SSH_TARGET
fi
