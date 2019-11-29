/**
 * \ingroup REG
 * \defgroup REGCRC CRC calculation unit
 * @{
 */
#ifndef STM32LIB_INCLUDE_REGISTERS_CRC_H
#define STM32LIB_INCLUDE_REGISTERS_CRC_H

#include <stdint.h>

typedef struct
{
    unsigned RESET : 1;
    unsigned : 31;
} CRC_CR_t;
_Static_assert(sizeof(CRC_CR_t) == 4U, "CRC_CR_t is wrong size");

typedef struct
{
    uint32_t DR;
    uint8_t IDR;
    CRC_CR_t CR;
} CRC_t;
_Static_assert(sizeof(CRC_t) == 12U, "CRC_t is wrong size");

extern volatile CRC_t CRC;

#endif

/**
 * @}
 */
