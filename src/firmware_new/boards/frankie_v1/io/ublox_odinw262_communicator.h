#pragma once

#include <stdbool.h>

#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/usart.h"

typedef enum
{
    UBLOX_RESPONSE_UNDETERMINED = 0,  // The state is unkown, and yet to be determined
    UBLOX_RESPONSE_INCOMPLETE   = 1,  // An incomlete transmission was detected
    UBLOX_RESPONSE_OK           = 2,  // OK\r\n was returned at the end of the response
    UBLOX_RESPONSE_ERROR        = 3,  // ERROR\r\n was returned at the end of the response
} UbloxResponseStatus_t;

#define UBLOX_OK_RESPONSE_STRING "OK\r\n"
#define UBLOX_ERROR_RESPONSE_STRING "ERROR\r\n"
#define UBLOX_OK_RESPONSE_LENGTH_BYTES 4
#define UBLOX_ERROR_RESPONSE_LENGTH_BYTES 7

void io_ublox_odinw262_communicator_task(void* arg);

/**
 * Initializes the Ublox Odin W262 Communicator. This allows us to send AT Commands
 * according to the manual:
 *
 * https://www.u-blox.com/sites/default/files/u-connect-ATCommands-Manual_%28UBX-14044127%29_C1-Public.pdf
 *
 * @param uart_handle The uart handle to use to communicate with the ublox odin w262 chip
 * @param ublox_reset The pin to use to reset the ublox
 * @param ublox_response_timeout How long to wait for the ublox to respond (10 seconds is
 * a good number)
 *
 * @returns A Ublox OdinW262 command interface
 */
void io_ublox_odinw262_communicator_init(UART_HandleTypeDef* uart_handle,
                                         GpioPin_t* ublox_reset,
                                         uint32_t ublox_response_timeout);
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
 *
 * @param is_in_interrupt True if this function is being run in an interrupt
 */
void io_ublox_odinw262_communicator_handleIdleLine(bool is_in_interrupt);

char* io_ublox_odinw262_communicator_sendATCommand(const char* at_command);

void io_ublox_odinw262_communicator_extractResponseFromCircularBuffer(
    size_t last_parsed_byte_position, size_t current_byte_position, size_t buffer_length,
    uint8_t* circular_buffer, uint8_t* linear_buffer);

UbloxResponseStatus_t
io_ublox_odinw262_communicator_getUbloxResponseStatusFromCircularBuffer(
    size_t current_byte_position, size_t buffer_length, uint8_t* circular_buffer);
