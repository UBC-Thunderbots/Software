#!/bin/bash

# The `who` command returns all logged in users (IRL and remote)
# Finds IRL users by looking for the physical monitor
LOCAL_USER=$(who | grep -E '(:0|tty2)')

# Finds remote users, who are marked with `pts`
# for each one, finds their username, to make identifying them easier
# excludes the current user, otherwise server will always seem busy
#REMOTE_USERS_LIST=$(who | grep pts | grep -v "$(basename $(tty))" | awk '{print $1}')

echo $SSH_CHECK_MODE
echo "????"

# if the client requested a check, return the user info from above
if [ "$SSH_CHECK_MODE" == "1" ]; then
    if [ ! -z "$LOCAL_USER" ]; then
        echo "STATUS_BUSY_LOCAL"
        exit 0
    elif [ ! -z "$REMOTE_USERS_LIST" ]; then
        echo "STATUS_BUSY_REMOTE"
        echo "Connected: $REMOTE_USERS_LIST"
        exit 0
    fi
    exit 0
fi

# If the user actually wanted to connect, make sure:
# 1. no other users (IRL or remote) are using the PC 
# OR
# 2. the force flag is provided
if ([ ! -z "$LOCAL_USER" ] || [ ! -z "$REMOTE_USERS" ]) && [ "$FORCE_CONNECT" != "1" ]; then
    echo "The Mezz Computer is in use! Please use the connect script to see who is currently using it."
    
    # we technically hae already have an active connection at this point
    # just no shell is provided
    # close the connection after 1 min
    sleep 60
    echo "Connection timed out."
    exit 1
fi

# Trigger the visual warning dialog if someone is using the PC IRL
./mezzsh_warn.sh &

# if we get here, start a normal shell
exec $SHELL
