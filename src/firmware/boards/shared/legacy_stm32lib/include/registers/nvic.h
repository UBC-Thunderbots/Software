/**
 * \ingroup REG
 * \defgroup REGNVIC Nested vectored interrupt controller
 * @{
 */
#ifndef STM32LIB_REGISTERS_NVIC_H
#define STM32LIB_REGISTERS_NVIC_H

#include <stdint.h>

typedef uint32_t NVIC_ibits_t[8];

typedef uint32_t NVIC_inybbles_t[60];

typedef struct
{
    NVIC_ibits_t ISER;
    uint32_t pad1[24U];
    NVIC_ibits_t ICER;
    uint32_t pad2[24U];
    NVIC_ibits_t ISPR;
    uint32_t pad3[24U];
    NVIC_ibits_t ICPR;
    uint32_t pad4[24U];
    NVIC_ibits_t IABR;
    uint32_t pad5[56U];
    NVIC_inybbles_t IPR;
} NVIC_t;
_Static_assert(sizeof(NVIC_t) == 1008U, "NVIC_t is wrong size");

extern volatile NVIC_t NVIC;

typedef struct
{
    unsigned INTID : 9;
    unsigned : 23;
} STIR_t;
_Static_assert(sizeof(STIR_t) == 4U, "STIR_t is wrong size");

extern volatile STIR_t STIR;

#endif

/**
 * @}
 */
