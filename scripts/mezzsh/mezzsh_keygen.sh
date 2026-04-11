#!/bin/bash

PC_ALIAS="mezzsh"

GREEN='\e[1;32m'
NC='\e[0m'

# Check for root privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)"
   exit 1
fi

read -p "Enter the PC's IP address: " PC_IP
read -p "Enter a username for yourself. Please make it somewhat recognizable, preferably just your first name: " USERNAME

KEY_NAME="id_rsa_$PC_ALIAS"
KEY_PATH="$HOME/.ssh/$KEY_NAME"

echo -e "\n--- Generating SSH Key Pair ---"
ssh-keygen -t rsa -b 4096 -f "$KEY_PATH" -C "$USERNAME" -N ""

echo -e "\n--- Configuring SSH Alias ---"
cat <<EOF >> "$HOME/.ssh/config"

Host $PC_ALIAS
    HostName $PC_IP
    User $USERNAME
    IdentityFile $KEY_PATH
    SetEnv USERNAME=$USERNAME
    SendEnv USERNAME SSH_CHECK_MODE FORCE_CONNECT
EOF

echo "Adding $PC_IP $PC_ALIAS to /etc/hosts..."
echo "$PC_IP $PC_ALIAS" | sudo tee -a /etc/hosts

echo -e "\n--- PUBLIC KEY ---"
cat "${KEY_PATH}.pub"
echo "--------------------------------------------"

echo -e "{GREEN}Success!{NC} Please provide the whole public key above (at "$KEY_PATH.pub") to a software lead to finish setup\n"