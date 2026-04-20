#!/bin/bash

# Gets the names of all currently connected SSH users
# Each user has their name attached to their SSH public key in authorized_keys
# So we find the key each person used to connect, and then the corresponding name

# Get fingerprints and comments from authorized_keys
KEYS_FILE="/home/thunderbots/.ssh/authorized_keys"
declare -A key_comments

while read -r line; do
    # Skip the line if it doesn't start with "ssh"
    [[ ! $line =~ ^ssh ]] && continue

    # Get fingerprint for each key in authorized_keys
    fp=$(echo "$line" | ssh-keygen -l -f - | awk '{print $2}')
    comment=$(echo "$line" | awk '{print $NF}')
    key_comments["$fp"]="$comment"
done < "$KEYS_FILE"

# Find currently active SSH PIDs and match them to log entries
pgrep -u "root" sshd | tail -n +2 | while read -r pid; do
    # Look for the 'Accepted publickey' log entry for this specific PID
    fp_match=$(grep "sshd\[$pid\]" /var/log/auth.log | grep "Accepted publickey" | grep -oE "SHA256:[^ ]+")

    if [ -n "$fp_match" ]; then
        comment=${key_comments[$fp_match]}
        echo "${comment}"
    fi
done
