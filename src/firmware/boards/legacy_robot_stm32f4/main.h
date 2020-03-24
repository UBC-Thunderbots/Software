#ifndef MAIN_H
#define MAIN_H

// We disable formatting here because we need to include `FreeRTOS.h` before
// `semphr.h`, and clang-format can re-order includes
// clang-format off
#include "FreeRTOS.h"
#include <semphr.h>
// clang-format on

#include <stdbool.h>
#include <stdint.h>
#include <usb.h>

#include "firmware/boards/shared/legacy_freertos/include/FreeRTOS.h"

/**
 * \ingroup MAIN
 *
 * \brief The sources that are checked for liveness by the supervisor task.
 */
typedef enum
{
    MAIN_WDT_SOURCE_TICK,    ///< The normal ticks.
    MAIN_WDT_SOURCE_HSTICK,  ///< The fast ticks.
    MAIN_WDT_SOURCE_COUNT,   ///< The number of liveness sources.
} main_wdt_source_t;

/**
 * \ingroup MAIN
 *
 * \brief The modes in which a system shutdown can occur.
 */
typedef enum
{
    MAIN_SHUT_MODE_POWER,   ///< Power off after shutdown
    MAIN_SHUT_MODE_REBOOT,  ///< Reboot after shutdown
    MAIN_SHUT_MODE_DFU,     ///< Go into DFU after shutdown
} main_shut_mode_t;

extern const udev_language_info_t MAIN_LANGUAGE_TABLE[];
extern const usb_string_zero_descriptor_t MAIN_STRING_ZERO;
extern SemaphoreHandle_t main_shutdown_sem;

void main_kick_wdt(main_wdt_source_t source);
void main_shutdown(main_shut_mode_t mode);
uint32_t main_read_clear_idle_cycles(void);

#endif
