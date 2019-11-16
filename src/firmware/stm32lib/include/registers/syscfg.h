/**
 * \ingroup REG
 * \defgroup REGSYSCFG System configuration controller
 * @{
 */
#ifndef STM32LIB_REGISTERS_SYSCFG_H
#define STM32LIB_REGISTERS_SYSCFG_H

#include <stdint.h>

typedef struct
{
    unsigned MEM_MODE : 2;
    unsigned : 30;
} SYSCFG_MEMRMP_t;
_Static_assert(sizeof(SYSCFG_MEMRMP_t) == 4U, "SYSCFG_MEMRMP_t is wrong size");

typedef struct
{
    unsigned : 23;
    unsigned MII_RMII_SEL : 1;
    unsigned : 8;
} SYSCFG_PMC_t;
_Static_assert(sizeof(SYSCFG_PMC_t) == 4U, "SYSCFG_PMC_t is wrong size");

typedef struct
{
    unsigned CMP_PD : 1;
    unsigned : 7;
    unsigned READY : 1;
    unsigned : 23;
} SYSCFG_CMPCR_t;
_Static_assert(sizeof(SYSCFG_CMPCR_t) == 4U, "SYSCFG_CMPCR_t is wrong size");

typedef struct
{
    SYSCFG_MEMRMP_t MEMRMP;
    SYSCFG_PMC_t PMC;
    uint32_t EXTICR[4U];
    unsigned int pad1;
    unsigned int pad2;
    SYSCFG_CMPCR_t CMPCR;
} SYSCFG_t;
_Static_assert(sizeof(SYSCFG_t) == 0x24U, "SYSCFG_t is wrong size");

extern volatile SYSCFG_t SYSCFG;

#endif

/**
 * @}
 */
