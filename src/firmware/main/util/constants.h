#ifndef CONSTANTS_H
#define CONSTANTS_H

/**
 * \brief The vendor ID.
 */
#define VENDOR_ID 0x0483U

/**
 * \brief The product ID in main mode.
 */
#define PRODUCT_ID 0x497EU

/**
 * \brief The product ID in DFU mode.
 */
#define PRODUCT_ID_DFU 0x497FU

/**
 * \brief The interface numbers.
 */
enum
{
    INTERFACE_CDC_ACM,
    INTERFACE_COUNT,
};

/**
 * \brief The string indices understood by a GET DESCRIPTOR(String) request.
 */
enum
{
    STRING_INDEX_ZERO = 0U,
    STRING_INDEX_MANUFACTURER,
    STRING_INDEX_PRODUCT,
    STRING_INDEX_PRODUCT_DFU,
    STRING_INDEX_SERIAL,
    STRING_INDEX_DFU_FW,
    STRING_INDEX_DFU_FPGA,
    STRING_INDEX_COUNT,
};

/**
 * \brief The vendor-specific control requests understood by the robot.
 */
enum
{
    CONTROL_REQUEST_READ_CORE     = 0x0DU,
    CONTROL_REQUEST_READ_BUILD_ID = 0x0EU,
};

#endif
