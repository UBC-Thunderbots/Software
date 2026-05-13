#!/bin/bash

# Script to check if there's a person logged in physically
# at the Mezz PC
# The `who` command returns all logged in users (IRL and remote)
# Finds IRL users by looking for the physical monitor (tty2)
who | grep -m 1 "(tty2)" | awk '{print $1}'