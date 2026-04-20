#!/bin/bash

# find the username of the person logged in physically
# we look for someone attached to the main display (:0)
LOCAL_USER=$(who | grep -m 1 "(tty2)" | awk '{print $1}')

# if no one is logged in locally, just exit
if [ -z "$LOCAL_USER" ]; then
    exit 0
fi

# get the User ID of the local user
LOCAL_UID=$(id -u "$LOCAL_USER")

# Trigger the dialog
# these environment variables must be set to trigger the dialog
# specifically on our wayland setup
sudo -u "$LOCAL_USER" DISPLAY=tty2 \
    DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$LOCAL_UID/bus \
    XDG_RUNTIME_DIR="/run/user/$LOCAL_UID" \
    WAYLAND_DISPLAY="wayland-0" \
    GDK_BACKEND="wayland" \
    zenity --warning --title="Remote Connection" \
    --text="A new has just connected remotely (via SSH)." --timeout=10 &
