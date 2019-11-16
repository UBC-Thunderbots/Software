/**
 * \ingroup REG
 * \defgroup REGID Device electronic signature
 * @{
 */
#ifndef STM32LIB_REGISTERS_ID_H
#define STM32LIB_REGISTERS_ID_H

#include <stdint.h>

typedef struct
{
    uint32_t L;
    uint32_t M;
    uint32_t H;
} U_ID_t;
_Static_assert(sizeof(U_ID_t) == 12U, "U_ID_t is wrong size");

extern const volatile U_ID_t U_ID;
extern const volatile uint16_t F_ID;

#endif

/**
 * @}
 */
