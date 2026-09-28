#!/usr/bin/env bash

# The cross compilers and header trees that the robot software and the motor
# board firmware are built with.

log_info "Setting up cross compiler for robot software"
install_cross_compiler
log_info "Done setting up cross compiler for robot software"

log_info "Setting Up Python Development Headers"
install_python_cross_compile_headers
install_python_toolchain_headers
log_info "Done Setting Up Python Development Headers"

log_info "Setting up STM32 cross-compiler"
install_stm32_cross_compiler
log_info "Done setting up STM32 cross-compiler"
