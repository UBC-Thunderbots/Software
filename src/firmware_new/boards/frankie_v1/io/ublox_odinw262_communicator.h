#pragma once

#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/usart.h"
#include <stdbool.h>

/**
 *
 */
void io_ublox_odinw262_communicator_task(void* arg);

/**
 * Initializes the
 * Odin W262. This allows us to send AT Commands according to the manual:
 *
 * https://www.u-blox.com/sites/default/files/u-connect-ATCommands-Manual_%28UBX-14044127%29_C1-Public.pdf
 *
 * @param uart_handle The uart handle to use to communicate with the ublox odin w262 chip
 * @param ublox_reset The pin to use to reset the ublox
 *
 * @returns A Ublox OdinW262 command interface
 */
void io_ublox_odinw262_communicator_init(UART_HandleTypeDef* uart_handle,
                                         GpioPin_t* ublox_reset);

/*
 * Reset the UBlox ODIN W262
 *
 * NOTE: This resets the chip, including the interface
 */
void io_ublox_odinw262_reset(void);

/**
 * UART Idle Line Interrupt Service Routine
 *
 * This will be called in an ISR context when the UART line is idle, this is useful when
 * We update the counter, post the semaphore, and return.
 */
void io_ublox_odinw262_communicator_handleIdleLine(bool is_in_interrupt);

char* io_ublox_odinw262_communicator_sendATCommand(const char* at_command);
