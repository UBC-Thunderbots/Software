#ifndef CONSTANTS_H
#define CONSTANTS_H

/**
 * \brief The vendor ID.
 */
#define VENDOR_ID 0x0483U

/**
 * \brief The product ID.
 */
#define PRODUCT_ID 0x497CU

/**
 * \brief The interface numbers.
 */
enum
{
    INTERFACE_RADIO,
    INTERFACE_DFU,
    INTERFACE_COUNT,
};

/**
 * \brief The interface subclass numbers used by the interfaces.
 */
enum
{
    SUBCLASS_RADIO = 0x01U,
};

/**
 * \brief The alternate setting numbers used by the radio interface.
 */
enum
{
    RADIO_ALTSETTING_OFF         = 0U,
    RADIO_ALTSETTING_NORMAL      = 1U,
    RADIO_ALTSETTING_PROMISCUOUS = 2U,
};

/**
 * \brief The protocol numbers used by the radio interface.
 *
 * These numbers act both to differentiate the alternate settings and also as
 * version numbers; any change to the operation of the alternate setting will
 * increment the protocol number.
 */
enum
{
    RADIO_PROTOCOL_OFF         = 0x01U,
    RADIO_PROTOCOL_NORMAL      = 0x45U,
    RADIO_PROTOCOL_PROMISCUOUS = 0x82U,
};

/**
 * \brief The string indices understood by a GET DESCRIPTOR(String) request.
 */
enum
{
    STRING_INDEX_ZERO = 0U,
    STRING_INDEX_MANUFACTURER,
    STRING_INDEX_PRODUCT,
    STRING_INDEX_RADIO_OFF,
    STRING_INDEX_NORMAL,
    STRING_INDEX_PROMISCUOUS,
    STRING_INDEX_SERIAL,
};

/**
 * \brief The vendor-specific control requests understood by the dongle.
 */
enum
{
    CONTROL_REQUEST_GET_CHANNEL           = 0x00U,
    CONTROL_REQUEST_SET_CHANNEL           = 0x01U,
    CONTROL_REQUEST_GET_SYMBOL_RATE       = 0x02U,
    CONTROL_REQUEST_SET_SYMBOL_RATE       = 0x03U,
    CONTROL_REQUEST_GET_PAN_ID            = 0x04U,
    CONTROL_REQUEST_SET_PAN_ID            = 0x05U,
    CONTROL_REQUEST_GET_MAC_ADDRESS       = 0x06U,
    CONTROL_REQUEST_SET_MAC_ADDRESS       = 0x07U,
    CONTROL_REQUEST_GET_PROMISCUOUS_FLAGS = 0x0AU,
    CONTROL_REQUEST_SET_PROMISCUOUS_FLAGS = 0x0BU,
    CONTROL_REQUEST_BEEP                  = 0x0CU,
    CONTROL_REQUEST_READ_CORE             = 0x0DU,
    CONTROL_REQUEST_READ_BUILD_ID         = 0x0EU,
    CONTROL_REQUEST_SET_TIME              = 0x0FU,
};

/**
 * The delivery status codes reported in message delivery reports.
 */
enum
{
    MDR_STATUS_OK               = 0U,
    MDR_STATUS_NOT_ASSOCIATED   = 1U,
    MDR_STATUS_NOT_ACKNOWLEDGED = 2U,
    MDR_STATUS_NO_CLEAR_CHANNEL = 3U,
};

#endif
