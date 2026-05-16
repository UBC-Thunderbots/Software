#!/bin/bash

# Gets the names of all currently connected SSH users
# Each user has their name attached to their SSH public key in authorized_keys
# So we find the key each current connection used to connect, and then the corresponding name

# Get keys and comments from authorized_keys
KEYS_FILE="/home/thunderbots/.ssh/authorized_keys"
declare -A key_to_names

while read -r line; do
    # Skip the line if it doesn't start with "ssh"
    # to skip over the extra flags attached to each key on separate lines
    [[ ! $line =~ ^ssh ]] && continue

    # the line consists of <ssh key> <username>

    # Get fingerprint for the ssh key
    fingerprint=$(echo "$line" | ssh-keygen -l -f - | awk '{print $2}')

    # get the username part of the line
    username=$(echo "$line" | awk '{print $NF}')

    # map fingerprint to username
    key_to_names["$fingerprint"]="$username"
done < "$KEYS_FILE"

# Find currently active SSH PIDs and match them to log entries
pgrep -u "root" sshd | tail -n +2 | while read -r pid; do
    # Look for the 'Accepted publickey' log entry for this specific PID
    # there should be a log entry with the text "sshd[<pid>]
    # log entry contains the fingerprint within the text "SHA256:<fingerprint>
    fingerprint_match=$(grep "sshd\[$pid\]" /var/log/auth.log | grep "Accepted publickey" | grep -oE "SHA256:[^ ]+")

    if [ -n "$fingerprint_match" ]; then
        username=${key_to_names[$fingerprint_match]}
        echo "${username}"
    fi
done
