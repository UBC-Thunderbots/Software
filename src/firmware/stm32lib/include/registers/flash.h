/**
 * \ingroup REG
 * \defgroup REGFLASH Embedded Flash memory interface
 * @{
 */
#ifndef STM32LIB_REGISTERS_FLASH_H
#define STM32LIB_REGISTERS_FLASH_H

#include <stdint.h>

typedef struct
{
    unsigned LATENCY : 3;
    unsigned : 5;
    unsigned PRFTEN : 1;
    unsigned ICEN : 1;
    unsigned DCEN : 1;
    unsigned ICRST : 1;
    unsigned DCRST : 1;
    unsigned : 19;
} FLASH_ACR_t;
_Static_assert(sizeof(FLASH_ACR_t) == 4U, "FLASH_ACR_t is wrong size");

typedef struct
{
    unsigned EOP : 1;
    unsigned OPERR : 1;
    unsigned : 2;
    unsigned WRPERR : 1;
    unsigned PGAERR : 1;
    unsigned PGPERR : 1;
    unsigned PGSERR : 1;
    unsigned : 8;
    unsigned BSY : 1;
    unsigned : 15;
} FLASH_SR_t;
_Static_assert(sizeof(FLASH_SR_t) == 4U, "FLASH_SR_t is wrong size");

typedef struct
{
    unsigned PG : 1;
    unsigned SER : 1;
    unsigned MER : 1;
    unsigned SNB : 5;
    unsigned PSIZE : 2;
    unsigned : 5;
    unsigned MER1 : 1;
    unsigned STRT : 1;
    unsigned : 7;
    unsigned EOPIE : 1;
    unsigned ERRIE : 1;
    unsigned : 5;
    unsigned LOCK : 1;
} FLASH_CR_t;
_Static_assert(sizeof(FLASH_CR_t) == 4U, "FLASH_CR_t is wrong size");

typedef struct
{
    unsigned OPTLOCK : 1;
    unsigned OPTSTRT : 1;
    unsigned BOR_LEV : 2;
    unsigned : 1;
    unsigned WDG_SW : 1;
    unsigned nRST_STOP : 1;
    unsigned nRST_STDBY : 1;
    unsigned RDP : 8;
    unsigned nWRP : 12;
    unsigned : 4;
} FLASH_OPTCR_t;
_Static_assert(sizeof(FLASH_OPTCR_t) == 4U, "FLASH_OPTCR_t is wrong size");

typedef struct
{
    unsigned : 16;
    unsigned nWRP : 12;
    unsigned : 4;
} FLASH_OPTCR1_t;
_Static_assert(sizeof(FLASH_OPTCR1_t) == 4U, "FLASH_OPTCR1_t is wrong size");

typedef struct
{
    FLASH_ACR_t ACR;
    uint32_t KEYR;
    uint32_t OPTKEYR;
    FLASH_SR_t SR;
    FLASH_CR_t CR;
    FLASH_OPTCR_t OPTCR;
    FLASH_OPTCR1_t OPTCR1;
} FLASH_t;
_Static_assert(sizeof(FLASH_t) == 28U, "FLASH_t is wrong size");

extern volatile FLASH_t FLASH;

#endif

/**
 * @}
 */
