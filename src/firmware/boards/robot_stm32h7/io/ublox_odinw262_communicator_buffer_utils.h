#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "firmware/boards/robot_stm32h7/io/ublox_odinw262_communicator_buffer_utils.h"

typedef enum
{
    UBLOX_RESPONSE_UNDETERMINED = 0,  // The state is unknown, and yet to be determined
    UBLOX_RESPONSE_INCOMPLETE   = 1,  // An incomlete transmission was detected
    UBLOX_RESPONSE_OK           = 2,  // OK\r\n was detected at the end of the response
    UBLOX_RESPONSE_ERROR        = 3,  // ERROR\r\n was detected at the end of the response
} UbloxResponseStatus_t;

#define UBLOX_OK_RESPONSE_STRING "OK\r\n"
#define UBLOX_ERROR_RESPONSE_STRING "ERROR\r\n"
#define UBLOX_OK_RESPONSE_LENGTH_BYTES 4
#define UBLOX_ERROR_RESPONSE_LENGTH_BYTES 7

/**
 * Extracts the bytes between last_parsed_byte_position and current_byte_position
 * for the circular_buffer, accounting for "wrapped around" data, and puts it in the
 * linear buffer.
 *
 * NOTE: This linear_buffer must be the same size as the circular_buffer.
 * The linear_buffer will be cleared and a '\0' character will be appended at the
 * end of the unwrapped data, turning the char array into a "string".
 *
 * @param last_parsed_byte_position The position of the byte that was last parsed
 * @param current_byte_position The position of the last byte that was updated in the
 * buffer
 * @param buffer_length The length of the circular and linear buffer
 * @param circular_buffer The circular buffer to extract from
 * @param linear_buffer The buffer to copy data from circular_buffer into
 *
 */
void io_ublox_odinw262_communicator_extractResponseFromCircularBuffer(
    size_t last_parsed_byte_position, size_t current_byte_position, size_t buffer_length,
    uint8_t* circular_buffer, char* linear_buffer);

/**
 * Peeks at the end of circular_buffer looking back from the current_byte_position,
 * and looks for UBLOX_OK_RESPONSE_STRING or a UBLOX_OK_RESPONSE_LENGTH_BYTES, and
 * returns UBLOX_RESPONSE_OK or UBLOX_RESPONSE_ERROR respectively.
 *
 * This function returns UBLOX_RESPONSE_INCOMPLETE if UBLOX_OK_RESPONSE_STRING or
 * UBLOX_ERROR_RESPONSE_STRING was not found.
 *
 * @param current_byte_position The position of the last byte that was updated in the
 * buffer
 * @param buffer_length The length of the circular buffer
 * @param circular_buffer The circular buffer to look at
 */
UbloxResponseStatus_t
io_ublox_odinw262_communicator_getUbloxResponseStatusFromCircularBuffer(
    size_t current_byte_position, size_t buffer_length, uint8_t* circular_buffer);
