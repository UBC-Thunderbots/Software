#include "usb_config.h"

#include <cdcacm.h>
#include <usb.h>
#include <usb_dfu.h>

#include "main.h"
#include "upgrade/dfu.h"

typedef struct __attribute__((packed))
{
    usb_configuration_descriptor_t config;
    cdcacm_descriptors_t acm;
    usb_interface_descriptor_t dfu_interface;
    usb_dfu_functional_descriptor_t dfu_functional;
} config_descriptor_t;

static const config_descriptor_t CONFIG_DESCRIPTOR = {
    .config =
        {
            .bLength             = sizeof(usb_configuration_descriptor_t),
            .bDescriptorType     = USB_DTYPE_CONFIGURATION,
            .wTotalLength        = sizeof(config_descriptor_t),
            .bNumInterfaces      = 3U,
            .bConfigurationValue = 1U,
            .iConfiguration      = 0U,
            .bmAttributes =
                {
                    .remoteWakeup = 0U,
                    .selfPowered  = 1U,
                    .one          = 1U,
                },
            .bMaxPower = 50U,
        },
    .acm = CDCACM_DESCRIPTORS_INIT(0U, 0U, 0U, 0U, 1U, 2U, 2U),
    .dfu_interface =
        {
            .bLength            = sizeof(usb_interface_descriptor_t),
            .bDescriptorType    = USB_DTYPE_INTERFACE,
            .bInterfaceNumber   = 2U,
            .bAlternateSetting  = 0U,
            .bNumEndpoints      = 0U,
            .bInterfaceClass    = USB_DFU_CLASS_APPLICATION_SPECIFIC,
            .bInterfaceSubClass = USB_DFU_SUBCLASS_DFU,
            .bInterfaceProtocol = USB_DFU_PROTOCOL_RUNTIME,
            .iInterface         = 0U,
        },
    .dfu_functional =
        {
            .bLength         = sizeof(usb_dfu_functional_descriptor_t),
            .bDescriptorType = USB_DFU_DTYPE_FUNCTIONAL,
            .bmAttributes =
                {
                    .bitCanDnload             = 1U,
                    .bitCanUpload             = 1U,
                    .bitManifestationTolerant = 0U,
                    .bitWillDetach            = 1U,
                },
            .wDetachTimeout = 0U,
            .wTransferSize  = UPGRADE_DFU_MAX_TRANSFER_SIZE,
            .bcdDFUVersion  = 0x0110U,
        },
};

static const udev_interface_info_t CONTROL_INTF = {
    .control_handler = 0,
    .endpoints       = {0, 0, 0, 0, 0, 0},
    .alternate_settings =
        {
            {
                .can_enter       = 0,
                .on_enter        = 0,
                .on_exit         = 0,
                .control_handler = 0,
                .endpoints       = {0, 0, 0, 0, 0, 0},
            },
        },
};

static const udev_interface_info_t DATA_INTF = {
    .control_handler = 0,
    .endpoints       = {0, 0, 0, 0, 0, 0},
    .alternate_settings =
        {
            {
                .can_enter       = 0,
                .on_enter        = 0,
                .on_exit         = 0,
                .control_handler = 0,
                .endpoints       = {0, 0, 0, 0, 0, 0},
            },
        },
};

static void dfu_detach_poststatus(void)
{
    main_shutdown(MAIN_SHUT_MODE_DFU);
}

static bool dfu_control_handler(const usb_setup_packet_t *pkt)
{
    if (pkt->bmRequestType.type == USB_CTYPE_CLASS)
    {
        static const usb_dfu_status_block_t DFU_STATUS = {
            .bStatus       = USB_DFU_STATUS_OK,
            .bwPollTimeout = 1,
            .bState        = USB_DFU_STATE_APP_IDLE,
            .iString       = 0,
        };
        if (pkt->bRequest == USB_DFU_CREQ_DETACH && !pkt->wLength)
        {
            uep0_set_poststatus(&dfu_detach_poststatus);
            return true;
        }
        else if (pkt->bRequest == USB_DFU_CREQ_GETSTATUS &&
                 pkt->bmRequestType.direction && !pkt->wValue &&
                 pkt->wLength == sizeof(DFU_STATUS))
        {
            uep0_data_write(&DFU_STATUS, sizeof(DFU_STATUS));
            return true;
        }
        else if (pkt->bRequest == USB_DFU_CREQ_GETSTATE && pkt->bmRequestType.direction &&
                 !pkt->wValue && pkt->wLength == 1U)
        {
            uint8_t state = DFU_STATUS.bState;
            uep0_data_write(&state, 1U);
            return true;
        }
    }

    return false;
}

static const udev_interface_info_t DFU_INTF = {
    .control_handler = &dfu_control_handler,
    .endpoints       = {0, 0, 0, 0, 0, 0},
    .alternate_settings =
        {
            {
                .can_enter       = 0,
                .on_enter        = 0,
                .on_exit         = 0,
                .control_handler = 0,
                .endpoints       = {0, 0, 0, 0, 0, 0},
            },
        },
};

const udev_config_info_t USB_CONFIGURATION = {
    .can_enter           = 0,
    .on_enter            = &cdcacm_start,
    .on_exit             = &cdcacm_stop,
    .control_handler     = 0,
    .descriptors         = &CONFIG_DESCRIPTOR.config,
    .transmit_fifo_words = {32, 32, 32},
    .endpoints           = {0, 0, 0, 0, 0, 0},
    .interfaces          = {&CONTROL_INTF, &DATA_INTF, &DFU_INTF},
};
