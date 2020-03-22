#ifndef USB_USB_H
#define USB_USB_H

#include <FreeRTOS.h>
#include <event_groups.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uchar.h>

/**
 * \ingroup UEP
 * \brief The maximum endpoint number supported by the stack.
 *
 * This value is limited by the STM32F4’s hardware
 */
#define UEP_MAX_ENDPOINT 3U

/**
 * \ingroup UEP
 * \brief Extracts and returns only the endpoint number part of an endpoint address.
 */
#define UEP_NUM(x) ((x)&0x7FU)

/**
 * \ingroup UEP
 * \brief Extracts only the direction part of an endpoint address and returns it as 1 for
 * IN or 0 for OUT
 */
#define UEP_DIR(x) ((x) >> 7U)

/**
 * \ingroup UEP
 * \brief Converts an endpoint address into an index number, suitable for use to index an
 * array or bit mask.
 *
 * If the array has length (\ref UEP_MAX_ENDPOINT × 2), then this macro can be used to
 * select a unique array element per endpoint address.
 */
#define UEP_IDX(x) ((UEP_NUM(x) - 1U) + (UEP_DIR(x) * UEP_MAX_ENDPOINT))



/**
 * \defgroup STD USB standard types and constants
 *
 * These data structures and constants match values defined in the USB specification.
 *
 * \{
 */

/**
 * \brief Standard control requests for \ref usb_setup_packet_t::bRequest.
 */
typedef enum
{
    USB_CREQ_GET_STATUS        = 0,
    USB_CREQ_CLEAR_FEATURE     = 1,
    USB_CREQ_SET_FEATURE       = 3,
    USB_CREQ_SET_ADDRESS       = 5,
    USB_CREQ_GET_DESCRIPTOR    = 6,
    USB_CREQ_SET_DESCRIPTOR    = 7,
    USB_CREQ_GET_CONFIGURATION = 8,
    USB_CREQ_SET_CONFIGURATION = 9,
    USB_CREQ_GET_INTERFACE     = 10,
    USB_CREQ_SET_INTERFACE     = 11,
    USB_CREQ_SYNCH_FRAME       = 12,
} usb_creq_t;

/**
 * \brief Standard descriptor types.
 */
typedef enum
{
    USB_DTYPE_DEVICE                    = 1,
    USB_DTYPE_CONFIGURATION             = 2,
    USB_DTYPE_STRING                    = 3,
    USB_DTYPE_INTERFACE                 = 4,
    USB_DTYPE_ENDPOINT                  = 5,
    USB_DTYPE_DEVICE_QUALIFIER          = 6,
    USB_DTYPE_OTHER_SPEED_CONFIGURATION = 7,
    USB_DTYPE_INTERFACE_POWER           = 8,
    USB_DTYPE_INTERFACE_ASSOCIATION     = 11,
} usb_dtype_t;

/**
 * \brief Standard class numbers.
 */
typedef enum
{
    USB_CLASS_MISC = 0xEF,  ///< The miscellaneous device class
} usb_class_t;

/**
 * \brief Standard subclass numbers.
 */
typedef enum
{
    USB_SUBCLASS_IAD = 0x02,  ///< The device subclass for a device using interface
                              ///< association descriptors
} usb_subclass_t;

/**
 * \brief Standard protocol numbers.
 */
typedef enum
{
    USB_PROTOCOL_IAD = 0x01,  ///< The device protocol for a device using interface
                              ///< association descriptors
} usb_protocol_t;

/**
 * \brief Standard feature identifiers used in CLEAR FEATURE and SET FEATURE requests.
 */
typedef enum
{
    USB_FEATURE_ENDPOINT_HALT        = 0,
    USB_FEATURE_DEVICE_REMOTE_WAKEUP = 1,
    USB_FEATURE_TEST_MODE            = 2,
} usb_feature_t;

/**
 * \brief Standard values for \ref usb_setup_packet_t::recipient.
 */
typedef enum
{
    USB_RECIPIENT_DEVICE    = 0,
    USB_RECIPIENT_INTERFACE = 1,
    USB_RECIPIENT_ENDPOINT  = 2,
    USB_RECIPIENT_OTHER     = 3,
} usb_recipient_t;

/**
 * \brief Standard values for \ref usb_setup_packet_t::type.
 */
typedef enum
{
    USB_CTYPE_STANDARD = 0,
    USB_CTYPE_CLASS    = 1,
    USB_CTYPE_VENDOR   = 2,
} usb_ctype_t;

/**
 * \brief Types of endpoints as indicated in endpoint descriptors.
 */
typedef enum
{
    USB_EPTYPE_CONTROL     = 0,
    USB_EPTYPE_ISOCHRONOUS = 1,
    USB_EPTYPE_BULK        = 2,
    USB_EPTYPE_INTERRUPT   = 3,
} usb_endpoint_type_t;

/**
 * \brief A device descriptor.
 */
typedef struct __attribute__((packed))
{
    uint8_t
        bLength;  ///< The length of this descriptor (\c sizeof(usb_device_descriptor_t))
    uint8_t bDescriptorType;  ///< The type of this descriptor (\ref usb_dtype_t
                              ///< "USB_DTYPE_DEVICE")
    uint16_t bcdUSB;  ///< The version of the USB standard with which the device complies
    uint8_t bDeviceClass;     ///< The class specification to which the device as a whole
                              ///< complies, 0 if each interface independently declares a
                              ///< class, or 0xFF for a vendor-specific class
    uint8_t bDeviceSubClass;  ///< The subclass code for the device as a whole
    uint8_t bDeviceProtocol;  ///< The protocol code for the device as a whole
    uint8_t bMaxPacketSize0;  ///< The maximum amount of data carried in an IN our OUT
                              ///< transaction on endpoint zero (one of 8, 16, 32, or 64)
    uint16_t idVendor;        ///< The device vendor ID
    uint16_t idProduct;       ///< The device product ID
    uint16_t bcdDevice;       ///< The version number of the device
    uint8_t iManufacturer;  ///< The string index of the manufacturer’s name string, or 0
                            ///< to omit
    uint8_t iProduct;  ///< The string index of the product’s name string, or 0 to omit
    uint8_t
        iSerialNumber;  ///< The string index of the device’s serial number, or 0 to omit
    uint8_t bNumConfigurations;  ///< The number of configurations available
} usb_device_descriptor_t;

/**
 * \brief A configuration descriptor.
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;              ///< The length of this descriptor (\c
                                  ///< sizeof(usb_configuration_descriptor_t))
    uint8_t bDescriptorType;      ///< The type of this descriptor (\ref usb_dtype_t
                                  ///< "USB_DTYPE_CONFIGURATION")
    uint16_t wTotalLength;        ///< The length of this descriptor and all descriptors
                                  ///< following and subordinate to it (such as interface,
                                  ///< endpoint, and other descriptors)
    uint8_t bNumInterfaces;       ///< The number of interfaces in this configuration
    uint8_t bConfigurationValue;  ///< The ID number passed in GET CONFIGURATION and SET
                                  ///< CONFIGURATION to identify this configuration
    uint8_t iConfiguration;       ///< The string index of a string describing this
                                  ///< configuration, or 0 to omit
    struct __attribute__((packed))
    {
        unsigned : 5;
        unsigned remoteWakeup : 1;  ///< 1 if the device is capable of performing remote
                                    ///< wakeup while in this configuration
        unsigned selfPowered : 1;   ///< 1 if the device can only use this configuration
                                    ///< when its external power supply is attached
        unsigned one : 1;           ///< Always set
    } bmAttributes;                 ///< Miscellaneous attributes of the configuration
    uint8_t bMaxPower;  ///< The maximum number of 2 mA units of current drawn from VBUS
                        ///< while this configuration is active
} usb_configuration_descriptor_t;

/**
 * \brief An interface descriptor.
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;            ///< The length of this descriptor (\c
                                ///< sizeof(usb_interface_descriptor_t))
    uint8_t bDescriptorType;    ///< The type of this descriptor (\ref usb_dtype_t
                                ///< "USB_DTYPE_INTERFACE")
    uint8_t bInterfaceNumber;   ///< The index of the interface this descriptor is
                                ///< describing, from 0 to \ref
                                ///< usb_configuration_descriptor_t::bNumInterfaces − 1
    uint8_t bAlternateSetting;  ///< The index of the alternate setting of the interface
                                ///< described by this descriptor, starting from 0 for the
                                ///< first alternate setting
    uint8_t bNumEndpoints;  ///< The number of endpoints used by this alternate setting,
                            ///< whose endpoint descriptors must follow it
    uint8_t
        bInterfaceClass;  ///< The class specification to which the interface alternate
                          ///< setting complies, or 0xFF for a vendor-specific class
    uint8_t
        bInterfaceSubClass;  ///< The subclass code for the interface alternate setting
    uint8_t
        bInterfaceProtocol;  ///< The protocol code for the interface alternate setting
    uint8_t iInterface;      ///< The string index of a string describing this
                             ///< interface/alternate setting, or 0 to omit
} usb_interface_descriptor_t;

/**
 * \brief An endpoint descriptor.
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;           ///< The length of this descriptor (\c
                               ///< sizeof(usb_endpoint_descriptor_t))
    uint8_t bDescriptorType;   ///< The type of this descriptor (\ref usb_dtype_t
                               ///< "USB_DTYPE_ENDPOINT")
    uint8_t bEndpointAddress;  ///< The endpoint address described
    struct __attribute__((packed))
    {
        usb_endpoint_type_t type : 2;  ///< The type of endpoint
        unsigned syncType : 2;  ///< The synchronization type for an isochronous endpoint,
                                ///< or zero otherwise
        unsigned usageType : 2;  ///< The usage type for an isochronous endpoint, or zero
                                 ///< otherwise
        unsigned : 2;
    } bmAttributes;           ///< Miscellaneous attributes of the endpoint
    uint16_t wMaxPacketSize;  ///< The maximum number of bytes carried in a single
                              ///< transaction to or from this endpoint
    uint8_t bInterval;        ///< The endpoint polling interval in milliseconds for an
                        ///< interrupt endpoint, one less than its binary logarithm for an
                        ///< isochronous endpoint, or zero otherwise
} usb_endpoint_descriptor_t;

/**
 * \brief A string descriptor.
 *
 * \see USB_STRING_DESCRIPTOR_INITIALIZER
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;  ///< The length of this descriptor, which must include the flexible
                      ///< array member
    uint8_t bDescriptorType;  ///< The type of this descriptor (\ref usb_dtype_t
                              ///< "USB_DTYPE_STRING")
    char16_t bString[];       ///< The UTF-16 characters making up the string
} usb_string_descriptor_t;

/**
 * \brief A string descriptor zero, containing a list of supported languages.
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;  ///< The length of this descriptor, which must include the flexible
                      ///< array member
    uint8_t bDescriptorType;  ///< The type of this descriptor (\ref usb_dtype_t
                              ///< "USB_DTYPE_STRING")
    uint16_t wLANGID[];       ///< The language IDs supported by the device
} usb_string_zero_descriptor_t;

/**
 * \brief The type of an interface association descriptor.
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;            ///< The length of this descriptor (\c
                                ///< sizeof(usb_interface_association_descriptor_t))
    uint8_t bDescriptorType;    ///< The type of this descriptor (\ref usb_dtype_t
                                ///< "USB_DTYPE_INTERFACE_ASSOCIATION")
    uint8_t bFirstInterface;    ///< The first interface number in the association
    uint8_t bInterfaceCount;    ///< The number of interfaces in the association
    uint8_t bFunctionClass;     ///< The class of the function in the association
    uint8_t bFunctionSubClass;  ///< The subclass of the function in the association
    uint8_t bFunctionProtocol;  ///< The protocol of the function in the association
    uint8_t iFunction;  ///< The string index of the function’s name, or 0 to omit
} usb_interface_association_descriptor_t;

/**
 * \brief The payload of a SETUP transaction.
 */
typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        usb_recipient_t recipient : 5;  ///< The type of entity to which the control
                                        ///< transfer is addressed
        usb_ctype_t type : 2;  ///< The level of standard which specifies the behaviour of
                               ///< the control transfer
        unsigned direction : 1;  ///< 1 if the data stage (if any) will be IN, or 0 if OUT
    } bmRequestType;             ///< Miscellaneous attributes of the control transfer
    uint8_t bRequest;            ///< The specific request being made
    uint16_t wValue;  ///< A general-purpose parameter whose meaning is request-specific
    uint16_t
        wIndex;  ///< For \ref usb_recipient_t "USB_RECIPIENT_ENDPOINT", generally a
                 ///< device address; for \ref usb_recipient_t "USB_RECIPIENT_INTERFACE",
                 ///< generally an interface number; otherwise, a general-purpose
                 ///< parameter whose meaning is request-specific
    uint16_t wLength;  ///< For \ref direction set, the maximum number of bytes the host
                       ///< will accept in the data stage; otherwise, the exact number of
                       ///< bytes the host will send in the data stage
} usb_setup_packet_t;

/**
 * \brief Evaluates to a struct initializer for a \ref usb_string_descriptor_t for a given
 * string.
 *
 * One might use this macro as follows:
 * \code
 * static const usb_string_descriptor_t GREETING_EN_CA =
 * USB_STRING_DESCRIPTOR_INITIALIZER(u"Hello World!"); \endcode
 *
 * \param[in] text the string literal to initialize with, which should be UTF-16
 */
#define USB_STRING_DESCRIPTOR_INITIALIZER(text)                                          \
    {                                                                                    \
        .bLength = sizeof(usb_string_descriptor_t) + sizeof((text)) - sizeof(char16_t),  \
        .bDescriptorType = USB_DTYPE_STRING, .bString = (text)                           \
    }

/**
 * \cond HIDDEN
 */
_Static_assert(sizeof(usb_device_descriptor_t) == 18,
               "usb_device_descriptor_t wrong size");
_Static_assert(sizeof(usb_configuration_descriptor_t) == 9,
               "usb_configuration_descriptor_t wrong size");
_Static_assert(sizeof(usb_interface_descriptor_t) == 9,
               "usb_interface_descriptor_t wrong size");
_Static_assert(sizeof(usb_endpoint_descriptor_t) == 7,
               "usb_endpoint_descriptor_t wrong size");
_Static_assert(sizeof(usb_string_descriptor_t) == 2,
               "usb_string_descriptor_t wrong size");
_Static_assert(sizeof(usb_string_zero_descriptor_t) == 2,
               "usb_string_zero_descriptor_t wrong size");
_Static_assert(sizeof(usb_interface_association_descriptor_t) == 8,
               "usb_interface_association_descriptor_t wrong size");
_Static_assert(sizeof(usb_setup_packet_t) == 8, "usb_setup_packet_t wrong size");
/**
 * \endcond
 */

/**
 * \}
 */



/**
 * \defgroup CONF Configuration tables
 *
 * These data structures define the configuration tables that define device operation.
 * An application must define, in memory (typically Flash, marked \c const), an instance
 * of \ref udev_info_t and pass it to \ref udev_init. The USB stack uses this structure to
 * find all the device’s descriptors, callbacks, and other information it needs.
 *
 * \{
 */

/**
 * \brief Callbacks for an endpoint.
 *
 * When an endpoint callback is needed, the stack searches for the single most-specific
 * instance of this structure. If the alternate setting containing the endpoint has an
 * info block for the endpoint, that instance will be used. Otherwise, if the interface
 * has an info block, that will be used. Otherwise, if the configuration has an info
 * block, that will be used. Otherwise, no info block will be used.
 *
 * Once the info block is found, the appropriate callback, if any, will be invoked.
 * If the info block is found at a particular level but no callback is provided, no
 * callback will be invoked—the search will not continue. Therefore, if a specific level
 * should “inherit” a higher-level callback, it must point to it directly.
 *
 * If no info block can be found for an endpoint, a default info block containing all null
 * callback pointers is assumed.
 */
typedef struct
{
    /**
     * \brief Checks whether an endpoint’s halt feature can be cleared.
     *
     * If omitted, this callback is assumed to allow clearing the halt feature at any
     * time.
     *
     * \retval true if the endpoint’s halt feature can be cleared
     * \retval false if the endpoint’s halt feature cannot be cleared
     */
    bool (*can_clear_halt)(void);

    /**
     * \brief Indicates that the endpoint’s halt feature has been set due to a commanded
     * halt.
     *
     * This function is invoked specifically due to a SET FEATURE request that sets the
     * endpoint’s halt feature. It is not invoked if the endpoint is halted for any other
     * reason, such as due to application request. It is also not invoked if a SET FEATURE
     * request for the endpoint halt feature is received while the endpoint is already
     * halted.
     */
    void (*on_commanded_halt)(void);

    /**
     * \brief Indicates that the endpoint’s halt feature has been cleared.
     *
     * This function is not invoked if a CLEAR FEATURE request for the endpoint halt
     * feature is received while the endpoint is not halted.
     */
    void (*on_clear_halt)(void);
} udev_endpoint_info_t;

/**
 * \brief Callbacks for one alternate setting of an interface.
 */
typedef struct
{
    /**
     * \brief Checks if the alternate setting can be used.
     *
     * When the host issues a SET INTERFACE request asking the device to enter this
     * alternate setting, this callback is invoked. The application can use this callback
     * to veto entry into the new alternate setting, if desired. For example, an alternate
     * setting that can only be used when specific conditions are met could check those
     * conditions
     *
     * Note that this callback is invoked \em before \ref on_exit for the previous
     * alternate setting, if any. This is because, should this callback veto the alternate
     * setting change, the previous alternate setting will remain active and should not be
     * torn down.
     *
     * This callback is optional; if omitted, the alternate setting is assumed to always
     * be usable.
     *
     * \retval true if this alternate setting can be entered now
     * \retval false if this alternate setting cannot be entered now and the request
     * should be rejected
     *
     * \pre This callback is invoked on the stack internal task.
     * \pre This alternate setting is not currently active.
     *
     * \note This callback must not be provided for alternate setting zero of an
     * interface; alternate setting zero must always be usable.
     */
    bool (*can_enter)(void);

    /**
     * \brief Sets up the alternate setting.
     *
     * When the host issues a SET INTERFACE request asking the device to enter this
     * alternate setting, this callback is invoked. This callback should do any necessary
     * setup work and launch any tasks that handle endpoints in this alternate setting.
     *
     * This callback is optional.
     *
     * \pre This callback is invoked on the stack internal task.
     * \pre Any previous alternate setting for the same interface has been torn down,
     * including calling the alternate setting’s \ref on_exit callback (if any). \pre All
     * endpoints in the alternate setting have been activated.
     */
    void (*on_enter)(void);

    /**
     * \brief Tears down the configuration.
     *
     * When the alternate setting must be left for any reason (SET INTERFACE, SET
     * CONFIGURATION request, USB reset signalling, cable unplugged, etc.), this callback
     * is invoked.
     *
     * This callback must not return until any operations on the interface’s endpoints
     * have stopped and any tasks have committed not to start further endpoint operations.
     *
     * This callback is optional.
     *
     * \pre This callback is invoked on the stack internal task.
     * \pre If changing configurations, the \ref udev_config_info_t::can_enter "can_enter"
     * callback for the new configuration being entered, if any, has been called and
     * returned \c true. \pre All endpoints in the alternate setting have been
     * deactivated.
     */
    void (*on_exit)(void);

    /**
     * \brief Handles control requests to this interface or its endpoints while in this
     * alternate setting.
     *
     * A request targeted at this interface or an endpoint used in this alternate setting
     * is passed to this callback first if this alternate setting is active.
     *
     * If this handler declines the request, it will be passed to the interface-specific
     * control handler, if there is one. If this handler accepts the request, no further
     * processing will occur, and this handler is responsible for calling \ref
     * uep0_data_read or \ref uep0_data_write to run the data stage of the transfer if
     * \ref usb_setup_packet_t::wLength "wLength" is nonzero.
     *
     * Device-targeted and other-targeted requests are never passed to this function.
     *
     * This callback is optional.
     *
     * \param[in] pkt the payload of the SETUP transaction starting the transfer
     *
     * \retval true if this handler handled the transfer
     * \retval false if this handler did not handle the transfer
     *
     * \pre This callback is invoked on the stack internal task.
     * \pre This alternate setting is active.
     * \pre The transfer is addressed to this interface or an endpoint within it.
     * \pre No part of the data or status stage for the transfer has occurred.
     *
     * \post If the callback returns \c true and \ref usb_setup_packet_t::wLength
     * "wLength" is nonzero, then the callback has completed the data stage with \ref
     * uep0_data_read or \ref uep0_data_write.
     */
    bool (*control_handler)(const usb_setup_packet_t *pkt);

    /**
     * \brief Pointers to information blocks for the endpoints which may be part of this
     * alternate setting.
     *
     * The array must be indexed by the \ref UEP_IDX macro.
     * Entries for all possible endpoints are provided in the array; those not present in
     * the alternate setting should be null. Endpoints that do not need information blocks
     * may also be null.
     */
    const udev_endpoint_info_t *endpoints[UEP_MAX_ENDPOINT * 2U];
} udev_alternate_setting_info_t;

/**
 * \brief Callbacks and configuration information for one interface.
 */
typedef struct
{
    /**
     * \brief Handles control requests to this interface or its endpoints.
     *
     * A request targeted at this interface or an endpoint used in the interface’s current
     * alternate setting is passed to this callback after the alternate setting, if any.
     *
     * If this handler declines the request, it will be passed to the
     * configuration-specific control handler, if there is one. If this handler accepts
     * the request, no further processing will occur, and this handler is responsible for
     * calling \ref uep0_data_read or \ref uep0_data_write to run the data stage of the
     * transfer if \ref usb_setup_packet_t::wLength "wLength" is nonzero.
     *
     * Device-targeted and other-targeted requests are never passed to this function.
     *
     * This callback is optional.
     *
     * \param[in] pkt the payload of the SETUP transaction starting the transfer
     *
     * \retval true if this handler handled the transfer
     * \retval false if this handler did not handle the transfer
     *
     * \pre This callback is invoked on the stack internal task.
     * \pre This alternate setting is active.
     * \pre The transfer is addressed to this interface or an endpoint within it.
     * \pre No part of the data or status stage for the transfer has occurred.
     *
     * \post If the callback returns \c true and \ref usb_setup_packet_t::wLength
     * "wLength" is nonzero, then the callback has completed the data stage with \ref
     * uep0_data_read or \ref uep0_data_write.
     */
    bool (*control_handler)(const usb_setup_packet_t *pkt);

    /**
     * \brief Pointers to information blocks for the endpoints which may be part of this
     * interface.
     *
     * The array must be indexed by the \ref UEP_IDX macro.
     * Entries for all possible endpoints are provided in the array; those not present in
     * the interface should be null. Endpoints that do not need information blocks may
     * also be null.
     */
    const udev_endpoint_info_t *endpoints[UEP_MAX_ENDPOINT * 2U];

    /**
     * \brief The alternate settings.
     *
     * There must be one entry in this array for each alternate setting of this interface,
     * in order.
     */
    udev_alternate_setting_info_t alternate_settings[];
} udev_interface_info_t;

/**
 * \brief Callbacks and descriptors for one device configuration.
 *
 * The application must store an instance of this structure in memory for each of the
 * device’s configurations. The USB stack finds these structures from the \ref
 * udev_info_t::configurations array.
 */
typedef struct
{
    /**
     * \brief Checks if the configuration can be used.
     *
     * When the host issues a SET CONFIGURATION request asking the device to enter this
     * configuration, this callback is invoked. The application can use this callback to
     * veto entry into the new configuration, if desired. For example, a configuration
     * that can only be used when an external power supply is available could check
     * whether that supply is available.
     *
     * Note that this callback is invoked \em before \ref on_exit for the previous
     * configuration, if any. This is because, should this callback veto the configuration
     * change, the previous configuration will remain active and should not be torn down.
     *
     * This callback is optional; if omitted, the configuration is assumed to always be
     * usable.
     *
     * \retval true if this configuration can be entered now
     * \retval false if this configuration cannot be entered now and the request should be
     * rejected
     *
     * \pre This callback is invoked on the stack internal task.
     * \pre This configuration is not currently active.
     */
    bool (*can_enter)(void);

    /**
     * \brief Sets up the configuration.
     *
     * When the host issues a SET CONFIGURATION request asking the device to enter this
     * configuration and the request is accepted, this callback is invoked. If some
     * interfaces in this configuration have only one setting (as opposed to multiple
     * alternate settings), this callback should launch the tasks that will handle the
     * endpoints in those interfaces. The application can also launch any other
     * configuration-specific tasks and do any other configuration-specific setup here.
     *
     * This callback is optional.
     *
     * \pre This callback is invoked on the stack internal task.
     * \pre Any previous configuration has been torn down, including tearing down any
     * interface alternate settings and calling the configuration’s \ref on_exit callback
     * (if any). \pre The \ref can_enter callback, if any, has been called and returned \c
     * true. \pre All endpoints in the configuration (except those under nonzero alternate
     * settings of interfaces) have been activated. \pre Interface alternate settings for
     * this configuration, if applicable, have \em not been set up yet.
     */
    void (*on_enter)(void);

    /**
     * \brief Tears down the configuration.
     *
     * When the configuration must be left for any reason (SET CONFIGURATION request, USB
     * reset signalling, cable unplugged, etc.), this callback is invoked.
     *
     * If some interfaces in this configuration have only one setting (as opposed to
     * multiple alternate settings), this callback must not return until any operations on
     * the endpoints have stopped and any tasks have committed not to start further
     * endpoint operations.
     *
     * This callback is optional.
     *
     * \pre This callback is invoked on the stack internal task.
     * \pre Any interface alternate settings within this configuration have already been
     * torn down. \pre The \ref can_enter callback for the new configuration being
     * entered, if any, has been called and returned \c true. \pre All endpoints in the
     * configuration have been deactivated.
     */
    void (*on_exit)(void);

    /**
     * \brief Handles control requests received while in this configuration.
     *
     * All requests targeted for the device as a whole (with \ref
     * usb_setup_packet_t::recipient "recipient" set to \ref usb_recipient_t
     * "USB_RECIPIENT_DEVICE") are passed to this callback first. A request targeted at a
     * specific interface or endpoint is passed to this callback if a more specific
     * callback declines to handle it.
     *
     * If this handler declines the request, it will be passed to the device-wide control
     * handler, if there is one. If this handler accepts the request, no further
     * processing will occur, and this handler is responsible for calling \ref
     * uep0_data_read or \ref uep0_data_write to run the data stage of the transfer if
     * \ref usb_setup_packet_t::wLength "wLength" is nonzero.
     *
     * This callback is optional.
     *
     * \param[in] pkt the payload of the SETUP transaction starting the transfer
     *
     * \retval true if this handler handled the transfer
     * \retval false if this handler did not handle the transfer
     *
     * \pre This callback is invoked on the stack internal task.
     * \pre No more specific callback has accepted the transfer.
     * \pre This configuration is active.
     * \pre No part of the data or status stage for the transfer has occurred.
     *
     * \post If the callback returns \c true and \ref usb_setup_packet_t::wLength
     * "wLength" is nonzero, then the callback has completed the data stage with \ref
     * uep0_data_read or \ref uep0_data_write.
     */
    bool (*control_handler)(const usb_setup_packet_t *pkt);

    /**
     * \brief The configuration descriptor and its subsidiary descriptors.
     *
     * USB requires that a single GET DESCRIPTOR request return a block of data starting
     * with a configuration descriptor and containing, following it, a mixture of
     * interface, endpoint, and other descriptors, with \ref
     * usb_configuration_descriptor_t::wTotalLength "wTotalLength" indicating the length
     * of that block. This pointer must point at such a block in memory. A typical method
     * for an application to construct such a block is to define a struct type containing
     * the descriptors in order, marked with <code>__attribute__((packed))</code>, and
     * define a constant instance of that struct type stored in Flash with the descriptors
     * filled in appropriately. The following example shows how this might be done:
     *
     * \code
     * const struct __attribute__((packed)) {
     * 	usb_configuration_descriptor_t config;
     * 	usb_interface_descriptor_t intf0;
     * 	usb_endpoint_descriptor_t ep1i;
     * } CONFIG_DESCRIPTOR = {
     * 	.config = {
     * 		.bLength = sizeof(usb_configuration_descriptor_t),
     * 		.bDescriptorType = USB_DTYPE_CONFIGURATION,
     * 		.wTotalLength = sizeof(CONFIG_DESCRIPTOR),
     * 		.bNumInterfaces = 1,
     * 		…
     * 	},
     *
     * 	.intf0 = {
     * 		.bLength = sizeof(usb_interface_descriptor_t),
     * 		.bDescriptorType = USB_DTYPE_INTERFACE,
     * 		.bInterfaceNumber = 0,
     * 		.bAlternateSetting = 0,
     * 		…
     * 	},
     *
     * 	.ep1i = {
     * 		.bLength = sizeof(usb_endpoint_descriptor_t),
     * 		.bDescriptorType = USB_DTYPE_ENDPOINT,
     * 		.bEndpointAddress = 0x81,
     * 		…
     * 	},
     * };
     * \endcode
     *
     * This field could then be initialized to \c &CONFIG_DESCRIPTOR.config.
     */
    const usb_configuration_descriptor_t *descriptors;

    /**
     * \brief The sizes of the nonzero endpoint transmit FIFOs, in words.
     *
     * Each transmit FIFO must be at least large enough to hold one full packet for the
     * endpoint; larger FIFOs will improve performance, though it is never useful to
     * allocate more than seven packets’ worth of FIFO space for an endpoint.
     *
     * In all cases, at least 16 words will be allocated to each FIFO due to hardware
     * requirements. If the relevant entry in this array is less than 16, it will be
     * ignored and 16 words will be allocated.
     *
     * The USB stack allocates a fixed amount of space to each transmit FIFO for the
     * lifetime of a configuration. Reallocating space from one FIFO to another between
     * alternate settings is not possible.
     *
     * In some cases, the USB stack may set a FIFO smaller than specified here, as a
     * mechanism to reduce the frequency of interrupts. If this occurs, there will be
     * unused spaces between different endpoints’ FIFOs; these are harmless.
     *
     * Element zero of this array corresponds to IN endpoint one.
     *
     * \internal
     *
     * Allocating a smaller-than-requested FIFO stems from the fact that the hardware
     * requires that the FIFO contain no more than seven packets at a time. Fortunately,
     * when seven packets are in the FIFO, the FIFO reports as completely full and the
     * half-empty interrupt is deasserted, even if not all words are filled. If the FIFO
     * is at least 14 packets long, the half-empty interrupt will trigger as soon as a
     * single packet has left the FIFO, because it will then be half-empty. This means
     * that an interrupt will be taken on every packet, which is inefficient. Reducing the
     * FIFO size ensures that even when the FIFO is non-full packet-wise, it will still be
     * more than half full word-wise. Thus, interrupts will be amortized down to the point
     * where the FIFO becomes half empty word-wise.
     *
     * Because the FIFO must be at least 16 words deep, this optimization is impossible to
     * apply for maximum packet sizes of 8 bytes or smaller, and has only modest benefit
     * until larger packet sizes are reached.
     */
    unsigned int transmit_fifo_words[UEP_MAX_ENDPOINT];

    /**
     * \brief Pointers to information blocks for the endpoints which may be part of this
     * interface.
     *
     * The array must be indexed by the \ref UEP_IDX macro.
     * Entries for all possible endpoints are provided in the array; those not present in
     * the interface should be null. Endpoints that do not need information blocks may
     * also be null.
     */
    const udev_endpoint_info_t *endpoints[UEP_MAX_ENDPOINT * 2U];

    /**
     * \brief The interfaces in this configuration.
     *
     * This array must contain one pointer for each interface, in order.
     * However, a pointer may be null if no special handling is needed for an interface.
     * In that case, the configuration entry/exit handler must start up and tear down the
     * endpoints in the interface.
     */
    const udev_interface_info_t *interfaces[];
} udev_config_info_t;

/**
 * \brief Configuration table for a language.
 */
typedef struct
{
    /**
     * \brief The language ID.
     *
     * This must be one of the numbers provided in the USB Language Identifiers document.
     */
    uint16_t id;

    /**
     * \brief A pointer to an array of pointers to string descriptors for this language,
     * in index order.
     *
     * A GET DESCRIPTOR request for string descriptor index \a N (1 ≤ \a N ≤ \ref
     * udev_info_t::string_count "string_count") is served from element \a N − 1 of this
     * array.
     */
    const usb_string_descriptor_t *const *strings;
} udev_language_info_t;

/**
 * \brief The root of the configuration tables.
 *
 * The application must store an instance of this structure in memory and pass it to \ref
 * udev_init.
 */
typedef struct
{
    struct
    {
        /**
         * \brief Indicates whether VBUS is attached to I/O pin PA9.
         */
        unsigned vbus_sensing : 1;

        /**
         * \brief Indicates whether to minimize interrupts at all costs.
         *
         * This flag instructs the stack to sacrifice USB performance in favour of saving
         * CPU time.
         */
        unsigned minimize_interrupts : 1;

        /**
         * \brief Whether the device should report itself as being self-powered
         * in response to a device-scope Get Status request.
         */
        unsigned self_powered : 1;
    }
    /**
     * \brief Flags describing the device’s operation.
     */
    flags;

    /**
     * \brief The priority of the internal task.
     */
    unsigned long internal_task_priority;

    /**
     * \brief The stack for the internal task.
     */
    void *internal_task_stack;

    /**
     * \brief The size of the stack for the internal task, in bytes.
     */
    size_t internal_task_stack_size;

    /**
     * \brief The size of the receive FIFO, in words.
     *
     * This must be large enough to hold all of the following:
     * \li three SETUP transaction payloads at 2 words each (6 words)
     * \li three SETUP transaction markers at 1 word each (3 words)
     * \li one control setup stage complete marker (1 word)
     * \li one global NAK status marker (1 word)
     * \li one OUT transaction payload of the maximum size for any endpoint
     * \li one OUT transaction marker (1 word)
     * \li one additional OUT transaction payload and transaction marker, if multiple
     * endpoints are used and at least one is isochronous \li one transfer complete marker
     * for each OUT endpoint that will be enabled at a time, at 1 word each
     *
     * It must also be at least 16.
     *
     * This is the bare minimum; providing a larger receive FIFO yields better performance
     * on both the control endpoint and any OUT endpoints.
     */
    unsigned int receive_fifo_words;

    /**
     * \brief The device descriptor.
     */
    usb_device_descriptor_t device_descriptor;

    /**
     * \brief The number of strings handled by the USB stack.
     *
     * This counts the number of distinct string indices used in the device, which are
     * given values from 1 to the value of this field. When filling this field, the number
     * of distinct languages is not considered. For example, a device that has a
     * manufacturer name and a product name would set this field to 2, no matter how many
     * languages those strings are translated into.
     *
     * If an application wishes, it can provide dynamically generated strings by using
     * indices beyond this value. The application must handle GET DESCRIPTOR requests for
     * these strings in a custom control handler.
     */
    size_t string_count;

    /**
     * \brief The string zero descriptor, or null if strings are not used.
     *
     * The table of language IDs in string descriptor zero must agree with the language
     * IDs provided in \ref language_table.
     */
    const usb_string_zero_descriptor_t *string_zero_descriptor;

    /**
     * \brief The language table, or null if strings are not used.
     *
     * The language table is an array of entries, ending with an entry with a null string
     * table pointer. An entry must be provided for each language that \ref
     * string_zero_descriptor claims is supported.
     */
    const udev_language_info_t *language_table;

    /**
     * \brief Handles control requests.
     *
     * All requests that are not handled at more specific locations are eventually passed
     * to this callback. This means requests that are not handled by the current
     * configuration’s control handler callback, if any, or by an endpoint-, alternate
     * setting-, or interface-specific callback, for requests targetting those recipients.
     *
     * If this handler declines the request, it will be passed to the stack’s built-in
     * handler (which handles all USB standard requests except for SET DESCRIPTOR). If
     * this handler accepts the request, no further processing will occur, and this
     * handler is responsible for calling \ref uep0_data_read or \ref uep0_data_write to
     * run the data stage of the transfer if \ref usb_setup_packet_t::wLength "wLength" is
     * nonzero.
     *
     * This callback is optional.
     *
     * \param[in] pkt the SETUP packet starting the transfer
     *
     * \retval true if this handler handled the transfer
     * \retval false if this handler did not handle the transfer
     *
     * \pre This callback is invoked on the stack internal task.
     * \pre No more specific callback has accepted the transfer.
     * \pre No part of the data or status stage for the transfer has occurred.
     */
    bool (*control_handler)(const usb_setup_packet_t *pkt);

    /**
     * \brief Pointers to the configuration information.
     *
     * The number of elements in this array must be equal to \ref
     * usb_device_descriptor_t::bNumConfigurations. When the host issues GET DESCRIPTOR to
     * retrieve configuration descriptor index \a N (0 ≤ N < \ref
     * usb_device_descriptor_t::bNumConfigurations "bNumConfigurations"), element \a N of
     * this array will be returned.
     *
     * The value used in SET CONFIGURATION has no correlation to the order or size of this
     * array. Values used in SET CONFIGURATION must match the configuration values stored
     * in \ref usb_configuration_descriptor_t::bConfigurationValue. A configuration’s \ref
     * usb_configuration_descriptor_t::bConfigurationValue may be any arbitrary nonzero
     * byte value. The USB stack automatically handles SET CONFIGURATION requests by
     * scanning this array for an element with matching \ref
     * usb_configuration_descriptor_t::bConfigurationValue "bConfigurationValue".
     */
    const udev_config_info_t *configurations[];
} udev_info_t;

/**
 * \}
 */



/**
 * \defgroup CBS Callbacks
 *
 * These types define the types of callback functions.
 *
 * \{
 */

/**
 * \brief Callback for completion of an asynchronous operation.
 *
 * \param[in] ep the endpoint address
 *
 * \param[in, out] from_isr_yield \c null if the notification is coming from
 * thread code, or a pointer to a boolean flag if the notification is coming
 * from an ISR which must be set to \c pdTRUE if a FreeRTOS yield is needed
 */
typedef void (*uep_async_cb_t)(unsigned int ep, BaseType_t *from_isr_yield);

/**
 * \}
 */



void udev_isr(void);
void udev_init(const udev_info_t *info);
void udev_attach(void);
void udev_detach(void);



bool uep_read(unsigned int ep, void *buffer, size_t max_length, size_t *length);
bool uep_write(unsigned int ep, const void *data, size_t length, bool zlp);
bool uep_halt_wait(unsigned int ep);
bool uep_async_read_start(unsigned int ep, void *buffer, size_t max_length,
                          uep_async_cb_t cb);
bool uep_async_read_finish(unsigned int ep, size_t *length);
bool uep_async_write_start(unsigned int ep, const void *data, size_t length, bool zlp,
                           uep_async_cb_t cb);
bool uep_async_write_finish(unsigned int ep);
bool uep_async_halt_wait_start(unsigned int ep, uep_async_cb_t cb);
bool uep_async_halt_wait_finish(unsigned int ep);
bool uep_halt(unsigned int ep);



bool uep0_data_read(void *buffer);
bool uep0_data_write(const void *data, size_t length);
void uep0_set_poststatus(void (*cb)(void));

#endif
