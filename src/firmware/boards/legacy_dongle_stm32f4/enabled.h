#ifndef ENABLED_H
#define ENABLED_H

#include <FreeRTOS.h>
#include <semphr.h>
#include <usb.h>

void enabled_init(void);

extern SemaphoreHandle_t enabled_mode_change_sem;
extern const udev_config_info_t ENABLED_CONFIGURATION;

#endif
