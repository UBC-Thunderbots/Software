#include "enabled.h"

#include <FreeRTOS.h>
#include <core_progmem.h>
#include <init.h>
#include <registers/scb.h>
#include <unused.h>
#include <usb.h>
#include <usb_dfu.h>

#include "buzzer.h"
#include "constants.h"
#include "led.h"
#include "normal.h"
#include "promiscuous.h"
#include "radio_config.h"

static const struct __attribute__((packed))
{
    usb_configuration_descriptor_t config;
    usb_interface_descriptor_t radio_off_intf;
    usb_interface_descriptor_t normal_intf;
    usb_endpoint_descriptor_t drive_data_ep;
    usb_endpoint_descriptor_t camera_ep;
    usb_endpoint_descriptor_t message_ep;
    usb_endpoint_descriptor_t mdr_ep;
    usb_endpoint_descriptor_t rx_ep;
    usb_endpoint_descriptor_t estop_ep;
    usb_interface_descriptor_t promisc_intf;
    usb_endpoint_descriptor_t promisc_rx_ep;
    usb_interface_descriptor_t dfu_intf;
    usb_dfu_functional_descriptor_t dfu;
} DESCRIPTORS = {
    .config =
        {
            .bLength             = sizeof(usb_configuration_descriptor_t),
            .bDescriptorType     = USB_DTYPE_CONFIGURATION,
            .wTotalLength        = sizeof(DESCRIPTORS),
            .bNumInterfaces      = 2U,
            .bConfigurationValue = 1U,
            .iConfiguration      = 0U,
            .bmAttributes =
                {
                    .one          = 1,
                    .selfPowered  = 0,
                    .remoteWakeup = 0,
                },
            .bMaxPower = 150U,
        },

    .radio_off_intf =
        {
            .bLength            = sizeof(usb_interface_descriptor_t),
            .bDescriptorType    = USB_DTYPE_INTERFACE,
            .bInterfaceNumber   = INTERFACE_RADIO,
            .bAlternateSetting  = RADIO_ALTSETTING_OFF,
            .bNumEndpoints      = 0U,
            .bInterfaceClass    = 0xFFU,
            .bInterfaceSubClass = SUBCLASS_RADIO,
            .bInterfaceProtocol = RADIO_PROTOCOL_OFF,
            .iInterface         = STRING_INDEX_RADIO_OFF,
        },

    .normal_intf =
        {
            .bLength            = sizeof(usb_interface_descriptor_t),
            .bDescriptorType    = USB_DTYPE_INTERFACE,
            .bInterfaceNumber   = INTERFACE_RADIO,
            .bAlternateSetting  = RADIO_ALTSETTING_NORMAL,
            .bNumEndpoints      = 6U,
            .bInterfaceClass    = 0xFFU,
            .bInterfaceSubClass = SUBCLASS_RADIO,
            .bInterfaceProtocol = RADIO_PROTOCOL_NORMAL,
            .iInterface         = STRING_INDEX_NORMAL,
        },

    .drive_data_ep =
        {
            .bLength          = sizeof(usb_endpoint_descriptor_t),
            .bDescriptorType  = USB_DTYPE_ENDPOINT,
            .bEndpointAddress = 0x01U,
            .bmAttributes =
                {
                    .type      = USB_EPTYPE_BULK,
                    .syncType  = 0U,
                    .usageType = 0U,
                },
            .wMaxPacketSize = 64U,
            .bInterval      = 5U,
        },


    .camera_ep =
        {
            .bLength          = sizeof(usb_endpoint_descriptor_t),
            .bDescriptorType  = USB_DTYPE_ENDPOINT,
            .bEndpointAddress = 0x02U,
            .bmAttributes =
                {
                    .type      = USB_EPTYPE_BULK,
                    .syncType  = 0U,
                    .usageType = 0U,
                },
            .wMaxPacketSize = 64U,
            .bInterval      = 5U,
        },

    .message_ep =
        {
            .bLength          = sizeof(usb_endpoint_descriptor_t),
            .bDescriptorType  = USB_DTYPE_ENDPOINT,
            .bEndpointAddress = 0x03U,
            .bmAttributes =
                {
                    .type      = USB_EPTYPE_BULK,
                    .syncType  = 0U,
                    .usageType = 0U,
                },
            .wMaxPacketSize = 64U,
            .bInterval      = 5U,
        },

    .mdr_ep =
        {
            .bLength          = sizeof(usb_endpoint_descriptor_t),
            .bDescriptorType  = USB_DTYPE_ENDPOINT,
            .bEndpointAddress = 0x81U,
            .bmAttributes =
                {
                    .type      = USB_EPTYPE_BULK,
                    .syncType  = 0U,
                    .usageType = 0U,
                },
            .wMaxPacketSize = 8U,
            .bInterval      = 0U,
        },

    .rx_ep =
        {
            .bLength          = sizeof(usb_endpoint_descriptor_t),
            .bDescriptorType  = USB_DTYPE_ENDPOINT,
            .bEndpointAddress = 0x82U,
            .bmAttributes =
                {
                    .type      = USB_EPTYPE_BULK,
                    .syncType  = 0U,
                    .usageType = 0U,
                },
            .wMaxPacketSize = 64U,
            .bInterval      = 5U,
        },

    .estop_ep =
        {
            .bLength          = sizeof(usb_endpoint_descriptor_t),
            .bDescriptorType  = USB_DTYPE_ENDPOINT,
            .bEndpointAddress = 0x83U,
            .bmAttributes =
                {
                    .type      = USB_EPTYPE_INTERRUPT,
                    .syncType  = 0U,
                    .usageType = 0U,
                },
            .wMaxPacketSize = 1U,
            .bInterval      = 10U,
        },

    .promisc_intf =
        {
            .bLength            = sizeof(usb_interface_descriptor_t),
            .bDescriptorType    = USB_DTYPE_INTERFACE,
            .bInterfaceNumber   = INTERFACE_RADIO,
            .bAlternateSetting  = RADIO_ALTSETTING_PROMISCUOUS,
            .bNumEndpoints      = 1U,
            .bInterfaceClass    = 0xFFU,
            .bInterfaceSubClass = SUBCLASS_RADIO,
            .bInterfaceProtocol = RADIO_PROTOCOL_PROMISCUOUS,
            .iInterface         = STRING_INDEX_PROMISCUOUS,
        },

    .promisc_rx_ep =
        {
            .bLength          = sizeof(usb_endpoint_descriptor_t),
            .bDescriptorType  = USB_DTYPE_ENDPOINT,
            .bEndpointAddress = 0x81U,
            .bmAttributes =
                {
                    .type      = USB_EPTYPE_BULK,
                    .syncType  = 0U,
                    .usageType = 0U,
                },
            .wMaxPacketSize = 64U,
            .bInterval      = 0U,
        },

    .dfu_intf =
        {
            .bLength            = sizeof(usb_interface_descriptor_t),
            .bDescriptorType    = USB_DTYPE_INTERFACE,
            .bInterfaceNumber   = INTERFACE_DFU,
            .bAlternateSetting  = 0U,
            .bNumEndpoints      = 0U,
            .bInterfaceClass    = USB_DFU_CLASS_APPLICATION_SPECIFIC,
            .bInterfaceSubClass = USB_DFU_SUBCLASS_DFU,
            .bInterfaceProtocol = USB_DFU_PROTOCOL_RUNTIME,
            .iInterface         = 0U,
        },

    .dfu =
        {
            .bLength         = sizeof(usb_dfu_functional_descriptor_t),
            .bDescriptorType = USB_DFU_DTYPE_FUNCTIONAL,
            .bmAttributes =
                {
                    .bitCanDnload             = 1,
                    .bitCanUpload             = 1,
                    .bitManifestationTolerant = 0,
                    .bitWillDetach            = 1,
                },
            .wDetachTimeout = 255U,
            .wTransferSize  = 8U,
            .bcdDFUVersion  = 0x011AU,
        },
};



static bool radio_control_handler(const usb_setup_packet_t *pkt)
{
    if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
        pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
        pkt->bRequest == CONTROL_REQUEST_GET_CHANNEL && !pkt->wValue)
    {
        uep0_data_write(&radio_config.channel, sizeof(radio_config.channel));
        return true;
    }
    else if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
             pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
             pkt->bRequest == CONTROL_REQUEST_GET_SYMBOL_RATE && !pkt->wValue)
    {
        uep0_data_write(&radio_config.symbol_rate, sizeof(radio_config.symbol_rate));
        return true;
    }
    else if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
             pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
             pkt->bRequest == CONTROL_REQUEST_GET_PAN_ID && !pkt->wValue)
    {
        uep0_data_write(&radio_config.pan_id, sizeof(radio_config.pan_id));
        return true;
    }
    else if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
             pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
             pkt->bRequest == CONTROL_REQUEST_GET_MAC_ADDRESS && !pkt->wValue)
    {
        uep0_data_write(&radio_config.mac_address, sizeof(radio_config.mac_address));
        return true;
    }
    else if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
             pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
             pkt->bRequest == CONTROL_REQUEST_BEEP && !pkt->wLength)
    {
        buzzer_start(pkt->wValue);
        return true;
    }
    return false;
}

static bool radio_off_control_handler(const usb_setup_packet_t *pkt)
{
    if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
        pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
        pkt->bRequest == CONTROL_REQUEST_SET_CHANNEL && 0x0BU <= pkt->wValue &&
        pkt->wValue <= 0x1AU && !pkt->wLength)
    {
        radio_config.channel = pkt->wValue;
        return true;
    }
    else if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
             pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
             pkt->bRequest == CONTROL_REQUEST_SET_SYMBOL_RATE && pkt->wValue <= 1U &&
             !pkt->wLength)
    {
        radio_config.symbol_rate = pkt->wValue;
        return true;
    }
    else if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
             pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
             pkt->bRequest == CONTROL_REQUEST_SET_PAN_ID && pkt->wValue != 0xFFFFU &&
             !pkt->wLength)
    {
        radio_config.pan_id = pkt->wValue;
        return true;
    }
    else if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
             pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
             pkt->bRequest == CONTROL_REQUEST_SET_MAC_ADDRESS && !pkt->wValue &&
             pkt->wLength == sizeof(radio_config.mac_address))
    {
        return uep0_data_read(&radio_config.mac_address);
    }
    return false;
}

static const udev_interface_info_t RADIO_INTERFACE_INFO = {
    .control_handler = &radio_control_handler,
    .endpoints       = {0},
    .alternate_settings =
        {
            [RADIO_ALTSETTING_OFF] =
                {
                    .can_enter       = 0,
                    .on_enter        = 0,
                    .on_exit         = 0,
                    .control_handler = &radio_off_control_handler,
                    .endpoints       = {0},
                },
            [RADIO_ALTSETTING_NORMAL] =
                {
                    .can_enter       = &normal_can_enter,
                    .on_enter        = &normal_on_enter,
                    .on_exit         = &normal_on_exit,
                    .control_handler = 0,
                    .endpoints       = {0},
                },
            [RADIO_ALTSETTING_PROMISCUOUS] =
                {
                    .can_enter       = 0,
                    .on_enter        = &promiscuous_on_enter,
                    .on_exit         = &promiscuous_on_exit,
                    .control_handler = &promiscuous_control_handler,
                    .endpoints       = {0},
                },
        },
};



static void dfu_detach_poststatus(void)
{
    udev_detach();
    init_bootload();
}

static bool dfu_control_handler(const usb_setup_packet_t *pkt)
{
    if (pkt->bmRequestType.type == USB_CTYPE_CLASS)
    {
        static const uint8_t DFU_STATUS[] = {
            0x00,        // bStatus (OK)
            10,   0, 0,  // bwPollTimeout
            0,           // bState (appIDLE)
            0,           // iString
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
            uep0_data_write(DFU_STATUS, sizeof(DFU_STATUS));
            return true;
        }
        else if (pkt->bRequest == USB_DFU_CREQ_GETSTATE && pkt->bmRequestType.direction &&
                 !pkt->wValue && pkt->wLength == 1U)
        {
            uep0_data_write(&DFU_STATUS[4U], 1U);
            return true;
        }
    }

    return false;
}

static const udev_interface_info_t DFU_INTERFACE_INFO = {
    .control_handler = &dfu_control_handler,
    .endpoints       = {0},
    .alternate_settings =
        {
            [0U] =
                {
                    .can_enter       = 0,
                    .on_enter        = 0,
                    .on_exit         = 0,
                    .control_handler = 0,
                    .endpoints       = {0},
                },
        },
};



static void config_on_enter(void)
{
    led_on(LED_POWER);
}

static void config_on_exit(void)
{
    led_off(LED_POWER);
    buzzer_stop();
}



const udev_config_info_t ENABLED_CONFIGURATION = {
    .can_enter       = 0,
    .on_enter        = &config_on_enter,
    .on_exit         = &config_on_exit,
    .control_handler = 0,
    .descriptors     = &DESCRIPTORS.config,
    .transmit_fifo_words =
        {
            [UEP_NUM(0x81U) - 1U] = 512U / 4U,
            [UEP_NUM(0x82U) - 1U] = 256U / 4U,
            [UEP_NUM(0x83U) - 1U] = 64U / 4U,
        },
    .endpoints = {0},
    .interfaces =
        {
            [INTERFACE_RADIO] = &RADIO_INTERFACE_INFO,
            [INTERFACE_DFU]   = &DFU_INTERFACE_INFO,
        },
};



/**
 * \brief A semaphore that an individual operating mode can use to sequence
 * initialization and shutdown of tasks in that mode.
 *
 * This semaphore must only be used by a particular operating mode during the
 * \ref udev_alternate_setting_info_t::on_enter or \ref
 * udev_alternate_setting_info_t::on_exit callbacks, thus guaranteeing that
 * multiple operations will not try to use the semaphore simultaneously.
 */
SemaphoreHandle_t enabled_mode_change_sem;



/**
 * \brief Initializes the objects used by the whole enabled configuration.
 */
void enabled_init(void)
{
    // Give the largest possible count. Mostly gives and takes will be aligned
    // 1:1, but sometimes multiple tasks might shut down simultaneously.
    static StaticSemaphore_t sem_storage;
    enabled_mode_change_sem =
        xSemaphoreCreateCountingStatic((UBaseType_t)-1, 0, &sem_storage);
}
