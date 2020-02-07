#ifndef CDCACM_CDCACM_H
#define CDCACM_CDCACM_h

#include <stddef.h>
#include <stdint.h>
#include <usb.h>

/**
 * \brief Descriptor types used in CDC ACM devices.
 */
typedef enum
{
    CDCACM_DTYPE_CS_INTERFACE = 0x24U,
} cdcacm_dtype_t;

/**
 * \brief Descriptor subtypes used in CDC ACM devices.
 */
enum
{
    CDCACM_DSTYPE_HEADER                      = 0x00U,
    CDCACM_DSTYPE_CALL_MANAGEMENT             = 0x01U,
    CDCACM_DSTYPE_ABSTRACT_CONTROL_MANAGEMENT = 0x02U,
    CDCACM_DSTYPE_UNION                       = 0x06U,
};

/**
 * \brief Class codes for CDC ACM devices, interface associations, and interfaces.
 */
typedef enum
{
    CDCACM_CLASS_CONTROL =
        0x02U,  ///< The class for a control interface or interface association
    CDCACM_CLASS_DATA = 0x0AU,  ///< The class for a data interface
} cdcacm_class_t;

/**
 * \brief Subclass codes for CDC ACM devices, interface associations, and interfaces.
 */
typedef enum
{
    CDCACM_SUBCLASS_CONTROL =
        0x02U,  ///< The subclass for a control interface or interface association
    CDCACM_SUBCLASS_DATA = 0x00,  ///< The subclass for a data interface
} cdcacm_subclass_t;

/**
 * \brief Protocol codes for CDC ACM devices, interface associations, and interfaces.
 */
typedef enum
{
    CDCACM_PROTOCOL_CONTROL =
        0x01U,  ///< The protocol for a control interface or interface association
    CDCACM_PROTOCOL_DATA = 0x00U,  ///< The protocol for a data interface
} cdcacm_protocol_t;

/**
 * \brief The type of a CDC header descriptor.
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;             ///< The length of this descriptor (\c
                                 ///< sizeof(cdcacm_interface_association_descriptor_t))
    uint8_t bDescriptorType;     ///< The type of this descriptor (\ref cdcacm_dtype_t
                                 ///< "CDCACM_DTYPE_CS_INTERFACE")
    uint8_t bDescriptorSubtype;  ///< The subtype of this descriptor (\ref cdcacm_dstype_t
                                 ///< "CDCACM_DSTYPE_HEADER")
    uint16_t bcdCDC;  ///< The version number of the CDC specification to which the device
                      ///< complies
} cdcacm_header_descriptor_t;

/**
 * \brief The type of a CDC union descriptor.
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;             ///< The length of this descriptor (\c
                                 ///< sizeof(cdcacm_interface_association_descriptor_t))
    uint8_t bDescriptorType;     ///< The type of this descriptor (\ref cdcacm_dtype_t
                                 ///< "CDCACM_DTYPE_CS_INTERFACE")
    uint8_t bDescriptorSubtype;  ///< The subtype of this descriptor (\ref cdcacm_dstype_t
                                 ///< "CDCACM_DSTYPE_UNION")
    uint8_t bControlInterface;   ///< The control interface number
    uint8_t bSubordinateInterface0;  ///< The data interface number
} cdcacm_union_descriptor_t;

/**
 * \brief The type of a CDC call management descriptor.
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;             ///< The length of this descriptor (\c
                                 ///< sizeof(cdcacm_interface_association_descriptor_t))
    uint8_t bDescriptorType;     ///< The type of this descriptor (\ref cdcacm_dtype_t
                                 ///< "CDCACM_DTYPE_CS_INTERFACE")
    uint8_t bDescriptorSubtype;  ///< The subtype of this descriptor (\ref cdcacm_dstype_t
                                 ///< "CDCACM_DSTYPE_CALL_MANAGEMENT")
    struct __attribute__((packed))
    {
        unsigned call_management : 1;  ///< Whether this device handles call management
        unsigned call_management_on_data : 1;  ///< Whether call management data can use
                                               ///< the data interface
        unsigned : 6;
    } bmCapabilities;        ///< The device’s capabilities
    uint8_t bDataInterface;  ///< The data interface used if call management is done on a
                             ///< data interface
} cdcacm_call_management_descriptor_t;

/**
 * \brief The type of a CDC abstract control management descriptor.
 */
typedef struct __attribute__((packed))
{
    uint8_t bLength;             ///< The length of this descriptor (\c
                                 ///< sizeof(cdcacm_interface_association_descriptor_t))
    uint8_t bDescriptorType;     ///< The type of this descriptor (\ref cdcacm_dtype_t
                                 ///< "CDCACM_DTYPE_CS_INTERFACE")
    uint8_t bDescriptorSubtype;  ///< The subtype of this descriptor (\ref cdcacm_dstype_t
                                 ///< "CDCACM_DSTYPE_ABSTRACT_CONTROL_MANAGEMENT")
    struct __attribute__((packed))
    {
        unsigned comm_features : 1;  ///< Whether the device supports communication
                                     ///< feature requests
        unsigned line_coding : 1;    ///< Whether the device supports line coding requests
        unsigned send_break : 1;  ///< whether the device supports the send break request
        unsigned connection_notify : 1;  ///< Whether the device supports connection
                                         ///< notifications
        unsigned : 4;
    } bmCapabilities;  ///< The device’s capabilities
} cdcacm_abstract_control_management_descriptor_t;

/**
 * \brief The type of the full set of descriptors needed to describe a CDC ACM device.
 *
 * This structure begins with an interface association descriptor and includes all the
 * interfaces and endpoints. The device descriptor should therefore indicate that the
 * device uses interface association descriptors.
 */
typedef struct __attribute__((packed))
{
    usb_interface_association_descriptor_t iad;  ///< The interface association descriptor
    usb_interface_descriptor_t
        control_intf;  ///< The interface descriptor for the control interface
    cdcacm_header_descriptor_t header;     ///< The header functional descriptor
    cdcacm_union_descriptor_t union_desc;  ///< The union functional descriptor
    cdcacm_call_management_descriptor_t
        call_management;  ///< The call management descriptor
    cdcacm_abstract_control_management_descriptor_t
        abstract_control_management;  ///< The abstract control management descriptor
    usb_endpoint_descriptor_t
        notification_ep;  ///< The endpoint descriptor for the notification endpoint
    usb_interface_descriptor_t
        data_intf;  ///< The interface descriptor for the data interface
    usb_endpoint_descriptor_t data_out_ep;  ///< The endpoint descriptor for OUT data
    usb_endpoint_descriptor_t data_in_ep;   ///< The endpoint descriptor for IN data
} cdcacm_descriptors_t;

/**
 * \brief Constructs a static initializer for a CDC descriptor list.
 *
 * \param[in] intf_num the interface number of the first interface to use (a CDC ACM uses
 * two interfaces) \param[in] iad_string the string index to use in the interface
 * association descriptor \param[in] control_string the string index to use in the control
 * interface descriptor \param[in] data_string the string index to use in the data
 * interface descriptor \param[in] notification_ep_num the endpoint number of an IN
 * endpoint for notification messages \param[in] data_out_ep_num the endpoint number of an
 * OUT endpoint for data \param[in] data_in_ep_num the endpoint number of an IN endpoint
 * for data
 */
#define CDCACM_DESCRIPTORS_INIT(intf_num, iad_string, control_string, data_string,       \
                                notification_ep_num, data_out_ep_num, data_in_ep_num)    \
    {                                                                                    \
        .iad =                                                                           \
            {                                                                            \
                .bLength           = sizeof(usb_interface_association_descriptor_t),     \
                .bDescriptorType   = USB_DTYPE_INTERFACE_ASSOCIATION,                    \
                .bFirstInterface   = (intf_num),                                         \
                .bInterfaceCount   = 2U,                                                 \
                .bFunctionClass    = CDCACM_CLASS_CONTROL,                               \
                .bFunctionSubClass = 0U,                                                 \
                .bFunctionProtocol = 0U,                                                 \
                .iFunction         = (iad_string),                                       \
            },                                                                           \
        .control_intf =                                                                  \
            {                                                                            \
                .bLength            = sizeof(usb_interface_descriptor_t),                \
                .bDescriptorType    = USB_DTYPE_INTERFACE,                               \
                .bInterfaceNumber   = (intf_num),                                        \
                .bAlternateSetting  = 0U,                                                \
                .bNumEndpoints      = 1U,                                                \
                .bInterfaceClass    = CDCACM_CLASS_CONTROL,                              \
                .bInterfaceSubClass = CDCACM_SUBCLASS_CONTROL,                           \
                .bInterfaceProtocol = CDCACM_PROTOCOL_CONTROL,                           \
                .iInterface         = (control_string),                                  \
            },                                                                           \
        .header =                                                                        \
            {                                                                            \
                .bLength            = sizeof(cdcacm_header_descriptor_t),                \
                .bDescriptorType    = CDCACM_DTYPE_CS_INTERFACE,                         \
                .bDescriptorSubtype = CDCACM_DSTYPE_HEADER,                              \
                .bcdCDC             = 0x0102U,                                           \
            },                                                                           \
        .union_desc =                                                                    \
            {                                                                            \
                .bLength                = sizeof(cdcacm_union_descriptor_t),             \
                .bDescriptorType        = CDCACM_DTYPE_CS_INTERFACE,                     \
                .bDescriptorSubtype     = CDCACM_DSTYPE_UNION,                           \
                .bControlInterface      = (intf_num),                                    \
                .bSubordinateInterface0 = (intf_num) + 1U,                               \
            },                                                                           \
        .call_management =                                                               \
            {                                                                            \
                .bLength            = sizeof(cdcacm_call_management_descriptor_t),       \
                .bDescriptorType    = CDCACM_DTYPE_CS_INTERFACE,                         \
                .bDescriptorSubtype = CDCACM_DSTYPE_CALL_MANAGEMENT,                     \
                .bmCapabilities =                                                        \
                    {                                                                    \
                        .call_management         = 0U,                                   \
                        .call_management_on_data = 0U,                                   \
                    },                                                                   \
                .bDataInterface = 0U,                                                    \
            },                                                                           \
        .abstract_control_management =                                                   \
            {                                                                            \
                .bLength = sizeof(cdcacm_abstract_control_management_descriptor_t),      \
                .bDescriptorType    = CDCACM_DTYPE_CS_INTERFACE,                         \
                .bDescriptorSubtype = CDCACM_DSTYPE_ABSTRACT_CONTROL_MANAGEMENT,         \
                .bmCapabilities =                                                        \
                    {                                                                    \
                        .comm_features     = 0U,                                         \
                        .line_coding       = 0U,                                         \
                        .send_break        = 0U,                                         \
                        .connection_notify = 0U,                                         \
                    },                                                                   \
            },                                                                           \
        .notification_ep =                                                               \
            {                                                                            \
                .bLength          = sizeof(usb_endpoint_descriptor_t),                   \
                .bDescriptorType  = USB_DTYPE_ENDPOINT,                                  \
                .bEndpointAddress = 0x80U | (notification_ep_num),                       \
                .bmAttributes =                                                          \
                    {                                                                    \
                        .type      = USB_EPTYPE_INTERRUPT,                               \
                        .syncType  = 0U,                                                 \
                        .usageType = 0U,                                                 \
                    },                                                                   \
                .wMaxPacketSize = 8U,                                                    \
                .bInterval      = 255U,                                                  \
            },                                                                           \
        .data_intf =                                                                     \
            {                                                                            \
                .bLength            = sizeof(usb_interface_descriptor_t),                \
                .bDescriptorType    = USB_DTYPE_INTERFACE,                               \
                .bInterfaceNumber   = (intf_num) + 1U,                                   \
                .bNumEndpoints      = 2U,                                                \
                .bInterfaceClass    = CDCACM_CLASS_DATA,                                 \
                .bInterfaceSubClass = CDCACM_SUBCLASS_DATA,                              \
                .bInterfaceProtocol = CDCACM_PROTOCOL_DATA,                              \
                .iInterface         = (data_string),                                     \
            },                                                                           \
        .data_out_ep =                                                                   \
            {                                                                            \
                .bLength          = sizeof(usb_endpoint_descriptor_t),                   \
                .bDescriptorType  = USB_DTYPE_ENDPOINT,                                  \
                .bEndpointAddress = (data_out_ep_num),                                   \
                .bmAttributes =                                                          \
                    {                                                                    \
                        .type      = USB_EPTYPE_BULK,                                    \
                        .syncType  = 0U,                                                 \
                        .usageType = 0U,                                                 \
                    },                                                                   \
                .wMaxPacketSize = 64U,                                                   \
                .bInterval      = 0U,                                                    \
            },                                                                           \
        .data_in_ep = {                                                                  \
            .bLength          = sizeof(usb_endpoint_descriptor_t),                       \
            .bDescriptorType  = USB_DTYPE_ENDPOINT,                                      \
            .bEndpointAddress = 0x80U | (data_in_ep_num),                                \
            .bmAttributes =                                                              \
                {                                                                        \
                    .type      = USB_EPTYPE_BULK,                                        \
                    .syncType  = 0U,                                                     \
                    .usageType = 0U,                                                     \
                },                                                                       \
            .wMaxPacketSize = 64U,                                                       \
            .bInterval      = 0U,                                                        \
        },                                                                               \
    }

void cdcacm_init(unsigned int in_data_ep_num, unsigned int task_priority);
void cdcacm_start(void);
void cdcacm_stop(void);
void cdcacm_write(const void *data, size_t length);

#endif
