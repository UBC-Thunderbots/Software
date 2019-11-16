/**
 * \ingroup REG
 * \defgroup REGPWR Power control
 * @{
 */
#ifndef STM32LIB_REGISTERS_POWER_H
#define STM32LIB_REGISTERS_POWER_H

typedef struct
{
    unsigned LPDS : 1;
    unsigned PDDS : 1;
    unsigned CWUF : 1;
    unsigned CSBF : 1;
    unsigned PVDE : 1;
    unsigned PLS : 3;
    unsigned DBP : 1;
    unsigned FPDS : 1;
    unsigned : 4;
    unsigned VOS : 2;
    unsigned : 16;
} PWR_CR_t;
_Static_assert(sizeof(PWR_CR_t) == 4U, "PWR_CR_t is wrong size");

extern volatile PWR_CR_t PWR_CR;

#endif

/**
 * @}
 */
