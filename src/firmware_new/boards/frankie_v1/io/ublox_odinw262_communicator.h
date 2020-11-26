#pragma once

#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/usart.h"

typedef struct UbloxOdinW262Communicator UbloxOdinW262Communicator_t;

/**
 * Creates a UbloxOdinW262Communicator_t to communicate with the AT interface of the
 * Odin W262. This allows us to send AT Commands according to the manual:
 *
 * https://www.u-blox.com/sites/default/files/u-connect-ATCommands-Manual_%28UBX-14044127%29_C1-Public.pdf
 *
 * @param uart_handle The uart handle to use to communicate with the ublox odin w262 chip
 * @param ublox_reset The pin to use to reset the ublox
 *
 * @returns A Ublox OdinW262 command interface
 */
UbloxOdinW262Communicator_t* io_ublox_odinw262_communicator_create(
    UART_HandleTypeDef* uart_handle, GpioPin_t* ublox_reset);

/**
 * Destroys the provided UbloxOdinW262Communicator_t
 *
 * @param communicator The UbloxOdinW262Communicator_t communicator to destroy
 */
void io_ublox_odinw262_communicator_destroy(
    UbloxOdinW262Communicator_t* communicator);

/*
 * Reset the UBlox ODIN W262
 *
 * NOTE: This resets the chip, including the interface
 *
 * @param communicator
 */
void io_ublox_odinw262_reset(UbloxOdinW262Communicator_t* communicator);

/**
 * Uart Idle Line Interrupt Service Routine
 *
 * This will be called in an ISR context when the UART line is idle, this is useful when
 * We update the counter, post the semaphore, and return.
 *
 * @param uart_handle The uart_handle
 */
void io_ublox_odinw262_communicator_connectToWifi(UbloxOdinW262Communicator_t* communicator, const char* wifi_ssid, const char* wifi_password);
void io_ublox_odinw262_communicator_handleIdleLineInterrupt(UART_HandleTypeDef* uart_handle);

void io_ublox_odinw262_communicator_sendATCommand(UbloxOdinW262Communicator_t* communicator, char* at_command);
