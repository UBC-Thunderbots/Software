/** This file contains constants and messages related to MRF communication */
#pragma once

#include <stdint.h>

#include <array>
#include <climits>

namespace MRF
{
    /**
     * The vendor ID used by the dongle.
     */
    constexpr uint16_t VENDOR_ID = 0x0483;

    /**
     * The product ID used by the dongle.
     */
    constexpr uint16_t PRODUCT_ID = 0x497C;

    /**
     * The interface subclass number used by the radio interface.
     */
    constexpr uint8_t SUBCLASS = 0x01;

    /**
     * The interface protocol number used by the dongle in radio off
     * mode.
     */
    constexpr uint8_t PROTOCOL_OFF = 0x01;

    /**
     * The interface protocol number used by the dongle in normal mode.
     */
    constexpr uint8_t PROTOCOL_NORMAL = 0x45;

    /**
     * The interface protocol number used by the dongle in promiscuous
     * mode.
     */
    constexpr uint8_t PROTOCOL_PROMISCUOUS = 0x82;

    /**
     * The vendor-specific control requests understood by the radio
     * interface.
     */
    enum
    {
        CONTROL_REQUEST_GET_CHANNEL           = 0x00,
        CONTROL_REQUEST_SET_CHANNEL           = 0x01,
        CONTROL_REQUEST_GET_SYMBOL_RATE       = 0x02,
        CONTROL_REQUEST_SET_SYMBOL_RATE       = 0x03,
        CONTROL_REQUEST_GET_PAN_ID            = 0x04,
        CONTROL_REQUEST_SET_PAN_ID            = 0x05,
        CONTROL_REQUEST_GET_MAC_ADDRESS       = 0x06,
        CONTROL_REQUEST_SET_MAC_ADDRESS       = 0x07,
        CONTROL_REQUEST_GET_PROMISCUOUS_FLAGS = 0x0A,
        CONTROL_REQUEST_SET_PROMISCUOUS_FLAGS = 0x0B,
        CONTROL_REQUEST_BEEP                  = 0x0C,
        CONTROL_REQUEST_READ_CORE             = 0x0D,
        CONTROL_REQUEST_READ_BUILD_ID         = 0x0E,
        CONTROL_REQUEST_SET_TIME              = 0x0F,
    };

    /**
     * The delivery status codes reported in message delivery reports.
     */
    enum
    {
        MDR_STATUS_OK,
        MDR_STATUS_NOT_ASSOCIATED,
        MDR_STATUS_NOT_ACKNOWLEDGED,
        MDR_STATUS_NO_CLEAR_CHANNEL,
    };

    /**
     * The number of level-triggered errors.
     */
    constexpr unsigned int ERROR_LT_COUNT = 20;

    /**
     * The number of edge-triggered errors.
     */
    constexpr unsigned int ERROR_ET_COUNT = 4;

    /**
     * The total number of errors.
     */
    constexpr unsigned int ERROR_COUNT = ERROR_LT_COUNT + ERROR_ET_COUNT;

    /**
     * The number of bytes to hold the error bitmask.
     */
    constexpr unsigned int ERROR_BYTES = (ERROR_COUNT + CHAR_BIT - 1) / CHAR_BIT;

    /**
     * The message patterns for the level-triggered error messages.
     * These messages fire continuously as long as the condition is true.
     */
    const std::array<const char*, ERROR_LT_COUNT> ERROR_LT_MESSAGES = {
        "charge timeout",
        "wheel 0 motor hot",
        "wheel 1 motor hot",
        "wheel 2 motor hot",
        "wheel 3 motor hot",
        "dribbler motor hot",
        "wheel 0 encoder not commutating",
        "wheel 1 encoder not commutating",
        "wheel 2 encoder not commutating",
        "wheel 3 encoder not commutating",
        "wheel 0 Hall sensor stuck low",
        "wheel 1 Hall sensor stuck low",
        "wheel 2 Hall sensor stuck low",
        "wheel 3 Hall sensor stuck low",
        "dribbler Hall sensor stuck low",
        "wheel 0 Hall sensor stuck high",
        "wheel 1 Hall sensor stuck high",
        "wheel 2 Hall sensor stuck high",
        "wheel 3 Hall sensor stuck high",
        "dribbler Hall sensor stuck high",
    };

    /**
     * The message patterns for the edge-triggered error messages.
     * This message fires only once when the condition occurs.
     */
    const std::array<const char*, ERROR_ET_COUNT> ERROR_ET_MESSAGES = {
        "ICB CRC error",
        "receive frame check sequence failure",
        "crashed (core dumped)",
        "crashed (no core dump)",
    };

    /**
     * Electrical-related thresholds and messages.
     */
    constexpr double MIN_CAP_VOLTAGE             = 5.0;
    static constexpr const char* LOW_CAP_MESSAGE = "Low capacitor voltage (fuse blown?)";

    constexpr double MIN_BATTERY_VOLTAGE             = 13.5;
    static constexpr const char* LOW_BATTERY_MESSAGE = "Low battery voltage";

    constexpr double MAX_BOARD_TEMPERATURE               = 90.0;
    static constexpr const char* HIGH_BOARD_TEMP_MESSAGE = "High board temperature";

    static constexpr const char* ROBOT_DEAD_MESSAGE = "Lost communication, dead?";

    /**
     * The total number of SD card-related messages.
     */
    constexpr unsigned int SD_MESSAGE_COUNT = 33;

    /**
     * The SD card messages.
     */
    const std::array<const char*, SD_MESSAGE_COUNT> SD_MESSAGES = {
        nullptr,
        "SD card uninitialized",
        nullptr,
        "SD card incompatible",
        "SD card sent illegal response",
        "SD layer logical error",
        "SD card CRC error",
        "SD card claimed illegal command",
        "SD card in unexpected state",
        "SD card internal error",
        "SD card command response timeout",
        "SD card parameter out of range",
        "SD card address misaligned",
        "SD card block length error",
        "SD card erase sequence error",
        "SD card erase parameter error",
        "SD card write protect violation",
        "SD card locked",
        "SD card lock or unlock failed",
        "SD card command CRC error",
        "SD card ECC error",
        "SD card CC error",
        "SD card generic error",
        "SD card CSD write error",
        "SD card partial erase due to write protection",
        "SD card ECC disabled",
        "SD card erase sequence cancelled",
        "SD card authentication sequence error",
        "SD card initialization timeout",
        "SD card data timeout",
        "SD card data CRC error",
        "SD card missing data start bit",
        "SD card FIFO overrun or underrun",
    };

    /* Robot logger messages */
    constexpr unsigned int LOGGER_MESSAGE_COUNT                         = 4;
    const std::array<const char*, LOGGER_MESSAGE_COUNT> LOGGER_MESSAGES = {
        nullptr,
        "Logger uninitialized",
        nullptr,
        "SD card full",
    };

    /* Dongle messages */
    static constexpr const char* ESTOP_BROKEN_MESSAGE = "EStop missing/broken";
    static constexpr const char* RX_FCS_FAIL_MESSAGE  = "Dongle receive FCS fail";
    static constexpr const char* SECOND_DONGLE_MESSAGE =
        "Second dongle on this channel+PAN";
    static constexpr const char* TRANSMIT_QUEUE_FULL_MESSAGE = "Transmit Queue Full";
    static constexpr const char* RECEIVE_QUEUE_FULL_MESSAGE  = "Receive Queue Full";
    static constexpr const char* PACKET_ABOVE_MAX_SIZE =
        "Attempted to send packet over radio greater than the maximum size";
}  // namespace MRF
