#!/usr/bin/env bash

# The SSL game controller and the TIGERS AutoRef, which thunderscope talks to.

log_info "Fetching game controller"
install_game_controller

log_info "Setting Up TIGERS AutoRef"
install_java
install_autoref
configure_autoref
log_info "Finished setting up AutoRef"
