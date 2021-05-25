#pragma once

#include <stdbool.h>

#include "firmware/boards/robot_stm32h7/io/gpio_pin.h"
#include "firmware/boards/robot_stm32h7/usart.h"

/**
 * Initializes the Ublox ODIN-W262 Communicator. This allows us to send AT Commands
 * according to the manual:
 *
 * https://www.u-blox.com/sites/default/files/u-connect-ATCommands-Manual_%28UBX-14044127%29_C1-Public.pdf
 *
 * @param uart_handle The uart handle to use to communicate with the ublox odin w262 chip
 * @param ublox_reset The pin to use to reset the ublox
 * @param ublox_response_timeout How long to wait for the ublox to respond (10 seconds is
 * a good number)
 */
void io_ublox_odinw262_communicator_init(UART_HandleTypeDef* uart_handle,
                                         GpioPin_t* ublox_reset,
                                         uint32_t ublox_response_timeout);

/**
 * Connects the u-blox ODIN-W262 to WiFi
 */
void io_ublox_odinw262_communicator_connectToWiFi(void);

/*
 * Reset the u-blox ODIN-W262
 *
 * NOTE: This resets the chip, including the interface
 */
void io_ublox_odinw262_reset(void);

/*
 * Wait for the u-blox ODIN-W262 to boot up
 */
void io_ublox_odinw262_communicator_waitForBoot(void);

/**
 * UART Idle Line Interrupt Service Routine
 *
 * This will be called in an ISR context when the UART line is idle
 */
void io_ublox_odinw262_communicator_handleIdleLine(void);

/**
 * Send an AT Command over UART to the u-blox.
 *
 * NOTE: the ptr returned should NOT be freed
 *
 * @param at_command The AT command to send to the ublox
 * @returns The response from the u-blox if the response was OK, NULL if ERROR
 */
char* io_ublox_odinw262_communicator_sendATCommand(const char* at_command);
