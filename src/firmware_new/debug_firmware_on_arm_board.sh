#!/bin/bash

# Start gdb to connect to the board via openocd
# TODO: the directory here needs to be set relatively
arm-none-eabi-gdb -ex "directory /home/gareth/programming/thunderbots/Software/src/" "$@"
