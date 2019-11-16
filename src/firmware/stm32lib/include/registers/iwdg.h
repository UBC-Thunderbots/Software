/**
 * \ingroup REG
 * \defgroup REGIWDG Independent watchdog
 * @{
 */
#ifndef STM32LIB_REGISTERS_IWDG_H
#define STM32LIB_REGISTERS_IWDG_H

#include <stdint.h>

typedef struct
{
    unsigned PVU : 1;
    unsigned RVU : 1;
    unsigned : 30;
} IWDG_SR_t;
_Static_assert(sizeof(IWDG_SR_t) == 4U, "IWDG_SR_t is wrong size");

typedef struct
{
    uint32_t KR;
    uint32_t PR;
    uint32_t RLR;
    IWDG_SR_t SR;
} IWDG_t;
_Static_assert(sizeof(IWDG_t) == 16U, "IWDG_t is wrong size");

extern volatile IWDG_t IWDG;

#endif

/**
 * @}
 */
