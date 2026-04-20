#!/bin/bash

# find the username of the person logged in physically
# we look for someone attached to the main display (:0)
LOCAL_USER=$(who | grep -m 1 "(tty2)" | awk '{print $1}')

# if no one is logged in locally, just exit
if [ -z "$LOCAL_USER" ]; then
    exit 0
fi

# get the User ID of the local user to access their "session bus"
LOCAL_UID=$(id -u "$LOCAL_USER")

# Trigger the dialog
# We must set DISPLAY and DBUS_SESSION_BUS_ADDRESS so the script knows 
# WHICH screen to pop up on.
sudo -u "$LOCAL_USER" DISPLAY=tty2 \
    DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$LOCAL_UID/bus \
    XDG_RUNTIME_DIR="/run/user/$LOCAL_UID" \
    WAYLAND_DISPLAY="wayland-0" \
    GDK_BACKEND="wayland" \
    zenity --warning --title="Remote Connection" \
    --text="A new has just connected remotely (via SSH)." --timeout=10 &
