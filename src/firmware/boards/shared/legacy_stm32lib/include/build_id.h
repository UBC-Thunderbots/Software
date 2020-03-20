/**
 * \defgroup BUILDID Build ID functions
 *
 * @{
 */

#ifndef STM32LIB_BUILDID_H
#define STM32LIB_BUILDID_H

#include <stdint.h>

void build_id_init(void);
uint32_t build_id_get(void);

#endif

/**
 * @}
 */
