#ifndef CONSTANTS_H
#define CONSTANTS_H

/**
 * \brief The vendor ID.
 */
#define VENDOR_ID 0x483U

/**
 * \brief The product ID in main mode.
 */
#define PRODUCT_ID 0x4980U

/**
 * \brief The string indices understood by a GET DESCRIPTOR(String) request
 */
enum
{
    STRING_INDEX_ZERO = 0U,
    STRING_INDEX_MANUFACTURER,
    STRING_INDEX_PRODUCT,
    STRING_INDEX_SERIAL,
    STRING_INDEX_COUNT,
};

/**
 * \brief CDC_ACM Task Priority (Low)
 */
#define PRIO_TASK_CDC_ACM 1U

#endif
