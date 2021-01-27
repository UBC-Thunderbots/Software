#!/bin/bash

# Explicitly allow AI vs AI ports
# Ports found in `constants.h`
sudo ufw allow 42069:42073/tcp
# Enable UFW rules
sudo ufw enable
sudo ufw status
