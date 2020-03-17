/**
 * \ingroup REG
 * \defgroup REGEXTI External interrupt/event controller
 * @{
 */
#ifndef STM32LIB_REGISTERS_EXTI_H
#define STM32LIB_REGISTERS_EXTI_H

#include <stdint.h>

typedef struct
{
    uint32_t IMR;
    uint32_t EMR;
    uint32_t RTSR;
    uint32_t FTSR;
    uint32_t SWIER;
    uint32_t PR;
} EXTI_t;
_Static_assert(sizeof(EXTI_t) == 24U, "EXTI_t is wrong size");

extern volatile EXTI_t EXTI;

#endif

/**
 * @}
 */
