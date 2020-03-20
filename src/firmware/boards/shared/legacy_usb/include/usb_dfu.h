#ifndef USB_USB_DFU_H
#define USB_USB_DFU_H

/**
 * \defgroup DFU USB device firmware upgrade class definitions
 *
 * These data structures and constants are defined by the USB Device Class
 * Specification for Device Firmware Upgrade, version 1.1.
 *
 * \{
 */

/**
 * \brief DFU-related control requests for \ref usb_setup_packet_t::bRequest.
 */
typedef enum
{
    USB_DFU_CREQ_DETACH    = 0,
    USB_DFU_CREQ_DNLOAD    = 1,
    USB_DFU_CREQ_UPLOAD    = 2,
    USB_DFU_CREQ_GETSTATUS = 3,
    USB_DFU_CREQ_CLRSTATUS = 4,
    USB_DFU_CREQ_GETSTATE  = 5,
    USB_DFU_CREQ_ABORT     = 6,
} usb_dfu_creq_t;

/**
 * \brief DFU-related descriptor types.
 */
typedef enum
{
    USB_DFU_DTYPE_FUNCTIONAL = 0x21,
} usb_dfu_dtype_t;

/**
 * \brief DFU-related class numbers.
 */
typedef enum
{
    /**
     * \brief The application-specific class.
     *
     * This class is used for all DFU interfaces.
     */
    USB_DFU_CLASS_APPLICATION_SPECIFIC = 0xFE,
} usb_dfu_class_t;

/**
 * \brief DFU-related subclass numbers.
 */
typedef enum
{
    /**
     * \brief The DFU subclass.
     *
     * This subclass is used for all DFU interfaces.
     */
    USB_DFU_SUBCLASS_DFU = 0x01,
} usb_dfu_subclass_t;

/**
 * \brief DFU-related protocol numbers.
 */
typedef enum
{
    /**
     * \brief The runtime protocol.
     *
     * This protocol is used in runtime DFU interfaces, which are exhibited
     * while the device is operating normally.
     */
    USB_DFU_PROTOCOL_RUNTIME = 0x01,

    /**
     * \brief The DFU mode protocol.
     *
     * This protocol is used in DFU-mode DFU interfaces, which are exhibited
     * while the device is performing a firmware upgrade operation.
     */
    USB_DFU_PROTOCOL_DFU = 0x02,
} usb_dfu_protocol_t;

/**
 * \brief The DFU device status values.
 */
typedef enum
{
    USB_DFU_STATUS_OK               = 0x00,
    USB_DFU_STATUS_ERR_TARGET       = 0x01,
    USB_DFU_STATUS_ERR_FILE         = 0x02,
    USB_DFU_STATUS_ERR_WRITE        = 0x03,
    USB_DFU_STATUS_ERR_ERASE        = 0x04,
    USB_DFU_STATUS_ERR_CHECK_ERASED = 0x05,
    USB_DFU_STATUS_ERR_PROG         = 0x06,
    USB_DFU_STATUS_ERR_VERIFY       = 0x07,
    USB_DFU_STATUS_ERR_ADDRESS      = 0x08,
    USB_DFU_STATUS_ERR_NOTDONE      = 0x09,
    USB_DFU_STATUS_ERR_FIRMWARE     = 0x0A,
    USB_DFU_STATUS_ERR_VENDOR       = 0x0B,
    USB_DFU_STATUS_ERR_USBR         = 0x0C,
    USB_DFU_STATUS_ERR_POR          = 0x0D,
    USB_DFU_STATUS_ERR_UNKNOWN      = 0x0E,
    USB_DFU_STATUS_ERR_STALLEDPKT   = 0x0F,
} usb_dfu_status_t;

/**
 * \brief The DFU device state values.
 */
typedef enum
{
    USB_DFU_STATE_APP_IDLE                = 0,
    USB_DFU_STATE_APP_DETACH              = 1,
    USB_DFU_STATE_DFU_IDLE                = 2,
    USB_DFU_STATE_DFU_DNLOAD_SYNC         = 3,
    USB_DFU_STATE_DFU_DNBUSY              = 4,
    USB_DFU_STATE_DFU_DNLOAD_IDLE         = 5,
    USB_DFU_STATE_DFU_MANIFEST_SYNC       = 6,
    USB_DFU_STATE_DFU_MANIFEST            = 7,
    USB_DFU_STATE_DFU_MANIFEST_WAIT_RESET = 8,
    USB_DFU_STATE_DFU_UPLOAD_IDLE         = 9,
    USB_DFU_STATE_DFU_ERROR               = 10,
} usb_dfu_state_t;

/**
 * \brief A DFU functional descriptor.
 *
 * This descriptor appears immediately following the DFU interface descriptor.
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;          ///< The length of this descriptor (\c
                              ///< sizeof(usb_dfu_functional_descriptor_t))
    uint8_t bDescriptorType;  ///< The type of this descriptor (\ref usb_dfu_dtype_t
                              ///< "USB_DFU_DTYPE_FUNCTIONAL")
    struct __attribute__((packed))
    {
        unsigned bitCanDnload : 1;              ///< Whether this device can download
        unsigned bitCanUpload : 1;              ///< Whether this device can upload
        unsigned bitManifestationTolerant : 1;  ///< Whether this device can communicate
                                                ///< after manifestation
        unsigned bitWillDetach : 1;  ///< Whether this device will detach when sent \c
                                     ///< USB_DFU_CREQ_DETACH
        unsigned : 4;
    } bmAttributes;           ///< Miscellaneous attributes of the device
    uint16_t wDetachTimeout;  ///< How many milliseconds the device waits to start DFU
                              ///< after a detach request
    uint16_t wTransferSize;   ///< Maximum size of a single download or upload transfer
                              ///< (not transaction)
    uint16_t bcdDFUVersion;   ///< Version of the DFU spec the device understands
} usb_dfu_functional_descriptor_t;

/**
 * \brief A response block to \ref USB_DFU_CREQ_GETSTATUS.
 */
typedef struct __attribute__((packed))
{
    usb_dfu_status_t bStatus : 8;
    unsigned bwPollTimeout : 24;
    usb_dfu_state_t bState : 8;
    unsigned int iString : 8;
} usb_dfu_status_block_t;

/**
 * \cond HIDDEN
 */
_Static_assert(sizeof(usb_dfu_functional_descriptor_t) == 9,
               "usb_dfu_functional_descriptor_t wrong size");
_Static_assert(sizeof(usb_dfu_status_block_t) == 6, "usb_dfu_status_t wrong size");
/**
 * \endcond
 */

/**
 * \}
 */

#endif
