/**
 * \ingroup REG
 * \defgroup REGSYSTICK System timer
 * @{
 */
#ifndef STM32LIB_REGISTERS_SYSTICK_H
#define STM32LIB_REGISTERS_SYSTICK_H

#include <stdint.h>

typedef struct
{
    unsigned ENABLE : 1;
    unsigned TICKINT : 1;
    unsigned CLKSOURCE : 1;
    unsigned : 13;
    unsigned COUNTFLAG : 1;
    unsigned : 15;
} SYST_CSR_t;
_Static_assert(sizeof(SYST_CSR_t) == 4U, "SYST_CSR_t is wrong size");

typedef struct
{
    unsigned TENMS : 24;
    unsigned : 6;
    unsigned SKEW : 1;
    unsigned NOREF : 1;
} SYST_CALIB_t;
_Static_assert(sizeof(SYST_CALIB_t) == 4U, "SYST_CALIB_t is wrong size");

typedef struct
{
    SYST_CSR_t CSR;
    uint32_t RVR;
    uint32_t CVR;
    SYST_CALIB_t CALIB;
} SYSTICK_t;
_Static_assert(sizeof(SYSTICK_t) == 16U, "SYSTICK_t is wrong size");

extern volatile SYSTICK_t SYSTICK;

#endif

/**
 * @}
 */
