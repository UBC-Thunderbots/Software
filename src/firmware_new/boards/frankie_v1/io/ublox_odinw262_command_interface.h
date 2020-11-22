#pragma once

#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/usart.h"

typedef struct UbloxOdinW262CommandInterface UbloxOdinW262CommandInterface_t;

/**
 * Creates a UbloxOdinW262CommandInterface_t to communicate with the AT interface of the
 * Odin W262 This allows us to send AT Commands according to manual:
 *
 * https://www.u-blox.com/sites/default/files/u-connect-ATCommands-Manual_%28UBX-14044127%29_C1-Public.pdf
 *
 * @param uart_handle The uart handle to use to communicate with the ublox odin w262 chip
 * @param ublox_reset The pin to use to reset the ublox
 *
 * @returns A Ublox OdinW262 command interface
 */
UbloxOdinW262CommandInterface_t* io_ublox_odinw262_interface_create(
    UART_HandleTypeDef* uart_handle, GpioPin_t* ublox_reset);

/**
 * Destroys the provided UbloxOdinW262CommandInterface_t
 *
 * @param command_interface The UbloxOdinW262CommandInterface_t interface to destroy
 */
void io_ublox_odinw262_interface_destroy(
    UbloxOdinW262CommandInterface_t* command_interface);

/*
 * Reset the UBlox ODIN W262
 *
 * @param command_interface
 */
void io_ublox_odinw262_reset(UbloxOdinW262CommandInterface_t* command_interface);
