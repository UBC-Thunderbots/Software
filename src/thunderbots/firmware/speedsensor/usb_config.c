#include "usb_config.h"
#include <cdcacm.h>
#include <usb.h>

typedef struct __attribute__((packed)) {
  usb_configuration_descriptor_t config;
  cdcacm_descriptors_t acm;
} config_descriptor_t;

static const config_descriptor_t CONFIG_DESCRIPTOR = {
  .config = {
    .bLength = sizeof(usb_configuration_descriptor_t),
    .bDescriptorType = USB_DTYPE_CONFIGURATION,
    .wTotalLength = sizeof(config_descriptor_t),
    .bNumInterfaces = 2U,
    .bConfigurationValue = 1U,
    .iConfiguration = 0U,
    .bmAttributes = {
      .remoteWakeup = 0U,
      .selfPowered = 0U,
      .one = 1U,
    },
    .bMaxPower = 150U, /* 150 * 2 = 300 mA */
  },
  /* This may be incorrect */
  .acm = CDCACM_DESCRIPTORS_INIT(0U, 0U, 0U, 0U, 1U, 2U, 2U),
};

static const udev_interface_info_t CONTROL_INTF = {
  .control_handler = 0,
  .endpoints = {0, 0, 0, 0, 0, 0},
  .alternate_settings = {
    {
      .can_enter = 0,
      .on_enter = 0,
      .on_exit = 0,
      .control_handler = 0,
      .endpoints = {0, 0, 0, 0, 0, 0},
    },
  },
};

static const udev_interface_info_t DATA_INTF = {
  .control_handler = 0,
  .endpoints = {0, 0, 0, 0, 0, 0},
  .alternate_settings = {
    {
      .can_enter = 0,
      .on_enter = 0,
      .on_exit = 0,
      .control_handler = 0,
      .endpoints = {0, 0, 0, 0, 0, 0},
    },
  },
};

const udev_config_info_t USB_CONFIGURATION = {
  .can_enter = 0,
  .on_enter = &cdcacm_start,
  .on_exit = &cdcacm_stop,
  .control_handler = 0,
  .descriptors = &CONFIG_DESCRIPTOR.config,
  .transmit_fifo_words = {32, 32, 32},
  .endpoints = {0, 0, 0, 0, 0, 0},
  .interfaces = { &CONTROL_INTF, &DATA_INTF },
};
