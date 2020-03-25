/**
 * \ingroup REG
 * \defgroup REGRNG Random number generator
 * @{
 */
#ifndef STM32LIB_INCLUDE_REGISTERS_RNG_H
#define STM32LIB_INCLUDE_REGISTERS_RNG_H

#include <stdint.h>

typedef struct
{
    unsigned : 2;
    unsigned RNGEN : 1;
    unsigned IE : 1;
    unsigned : 28;
} RNG_CR_t;
_Static_assert(sizeof(RNG_CR_t) == 4U, "RNG_CR_t is wrong size");

typedef struct
{
    unsigned DRDY : 1;
    unsigned CECS : 1;
    unsigned SECS : 1;
    unsigned : 2;
    unsigned CEIS : 1;
    unsigned SEIS : 1;
    unsigned : 25;
} RNG_SR_t;
_Static_assert(sizeof(RNG_SR_t) == 4U, "RNG_SR_t is wrong size");

typedef struct
{
    RNG_CR_t CR;
    RNG_SR_t SR;
    uint32_t DR;
} RNG_t;
_Static_assert(sizeof(RNG_t) == 12U, "RNG_t is wrong size");

extern volatile RNG_t RNG;

#endif

/**
 * @}
 */
