#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "firmware_new/boards/frankie_v1/io/ublox_odinw262_communicator_buffer_handlers.h"


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

void io_ublox_odinw262_communicator_extractResponseFromCircularBuffer(
    size_t last_parsed_byte_position, size_t current_byte_position, size_t buffer_length,
    uint8_t* circular_buffer, uint8_t* linear_buffer);

UbloxResponseStatus_t
io_ublox_odinw262_communicator_getUbloxResponseStatusFromCircularBuffer(
    size_t current_byte_position, size_t buffer_length, uint8_t* circular_buffer);
