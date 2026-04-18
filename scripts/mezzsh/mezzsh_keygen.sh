#!/bin/bash

PC_NAME="thunderbots"
ALIAS="mezzsh"

GREEN='\e[1;32m'
NC='\e[0m'

USER_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)

# Check for root privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)"
   exit 1
fi

bash ./utils/check_tailscale.sh

# Get the Tailscale IP of the Main PC
TARGET_IP=$(tailscale ip -4 $TAILSCALE_HOSTNAME)

if [ -z "$TARGET_IP" ]; then
    echo "Could not find Main PC on Tailscale. Are you logged in?"
    exit 1
fi

read -p "Enter a username for yourself. Please make it somewhat recognizable, preferably just your first name: " USERNAME

KEY_NAME="id_rsa_$ALIAS"
KEY_PATH="$USER_HOME/.ssh/$KEY_NAME"

echo -e "\n--- Generating SSH Key Pair ---"
ssh-keygen -t rsa -b 4096 -f "$KEY_PATH" -C "$USERNAME" -N ""

echo -e "\n--- Configuring SSH Alias ---"
cat <<EOF >> "$USER_HOME/.ssh/config"

Host $ALIAS
    HostName $PC_NAME
    User $PC_NAME
    IdentityFile $KEY_PATH
    IdentitiesOnly yes
    SendEnv SSH_CHECK_MODE FORCE_CONNECT
EOF

echo -e "\n--- PUBLIC KEY ---"
cat "${KEY_PATH}.pub"
echo "--------------------------------------------"

echo -e "{GREEN}Success!{NC} Please provide the whole public key above (at "$KEY_PATH.pub") to a software lead to finish setup\n"
