#!/bin/bash

# Generates a private / public key pair for connecting via ssh
# Adds the Mezz PC's IP to the ssh config file, and sets the new key as the identity to use
# And sets the key file's permissions correctly
# Each key also has a username attached to it for easier identification

PC_NAME="thunderbots"
ALIAS="mezzsh"

GREEN='\e[1;32m'
NC='\e[0m'

# get the actual current user despite running in sudo
USER_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)

# Check for root privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)"
   exit 1
fi

bash ./utils/check_tailscale.sh

# Get the Tailscale IP of the Main PC
TARGET_IP=$(tailscale ip -4 $PC_NAME)

if [ -z "$TARGET_IP" ]; then
    echo "Could not find Main PC on Tailscale. Are you logged in?"
    exit 1
fi

read -p "Enter a username for yourself. Please make it somewhat recognizable, preferably just your first name: " USERNAME

KEY_NAME="id_rsa_$ALIAS"
KEY_PATH="/.ssh/$KEY_NAME"
USER_KEY_PATH="$USER_HOME/.ssh/$KEY_NAME"

echo -e "\n--- Generating SSH Key Pair ---"

# username is appended as a comment to the key
ssh-keygen -t rsa -b 4096 -f "$USER_KEY_PATH" -C "$USERNAME" -N ""

# ssh is particular about permissions on the key file
# specifically, the private key file should be owned and should only be readable by the current user
sudo chown $SUDO_USER:$SUDO_USER $USER_KEY_PATH
sudo chmod 400 $USER_KEY_PATH
sudo chmod 400 "$USER_KEY_PATH.pub"

echo -e "\n--- Configuring SSH Alias ---"
cat <<EOF >> "$USER_HOME/.ssh/config"

Host $TARGET_IP
    User $PC_NAME
    IdentityFile ~$KEY_PATH
    IdentitiesOnly yes
    SendEnv SSH_CHECK_MODE FORCE_CONNECT
EOF

echo -e "\n--- PUBLIC KEY ---"
cat "${KEY_PATH}.pub"
echo "--------------------------------------------"

echo -e "${GREEN}Success!${NC} Please provide the whole public key above (at "$KEY_PATH.pub") to a software lead to finish setup\n"
