/**
 * \ingroup REG
 * \defgroup REGSPI Serial peripheral interface
 * @{
 */
#ifndef STM32LIB_REGISTERS_SPI_H
#define STM32LIB_REGISTERS_SPI_H

typedef struct
{
    unsigned CPHA : 1;
    unsigned CPOL : 1;
    unsigned MSTR : 1;
    unsigned BR : 3;
    unsigned SPE : 1;
    unsigned LSBFIRST : 1;
    unsigned SSI : 1;
    unsigned SSM : 1;
    unsigned RXONLY : 1;
    unsigned DFF : 1;
    unsigned CRCNEXT : 1;
    unsigned CRCEN : 1;
    unsigned BIDIOE : 1;
    unsigned BIDIMODE : 1;
    unsigned : 16;
} SPI_CR1_t;
_Static_assert(sizeof(SPI_CR1_t) == 4U, "SPI_CR1_t is wrong size");

typedef struct
{
    unsigned RXDMAEN : 1;
    unsigned TXDMAEN : 1;
    unsigned SSOE : 1;
    unsigned : 1;
    unsigned FRF : 1;
    unsigned ERRIE : 1;
    unsigned RXNEIE : 1;
    unsigned TXEIE : 1;
    unsigned : 24;
} SPI_CR2_t;
_Static_assert(sizeof(SPI_CR2_t) == 4U, "SPI_CR2_t is wrong size");

typedef struct
{
    unsigned RXNE : 1;
    unsigned TXE : 1;
    unsigned CHSIDE : 1;
    unsigned UDR : 1;
    unsigned CRCERR : 1;
    unsigned MODF : 1;
    unsigned OVR : 1;
    unsigned BSY : 1;
    unsigned FRE : 1;
    unsigned : 23;
} SPI_SR_t;
_Static_assert(sizeof(SPI_SR_t) == 4U, "SPI_SR_t is wrong size");

typedef struct
{
    unsigned CHLEN : 1;
    unsigned DATLEN : 2;
    unsigned CKPOL : 1;
    unsigned I2SSTD : 2;
    unsigned : 1;
    unsigned PCMSYNC : 1;
    unsigned I2SCFG : 2;
    unsigned I2SE : 1;
    unsigned I2SMOD : 1;
    unsigned : 20;
} SPI_I2SCFGR_t;
_Static_assert(sizeof(SPI_I2SCFGR_t) == 4U, "SPI_I2SCFGR_t is wrong size");

typedef struct
{
    unsigned I2SDIV : 8;
    unsigned ODD : 1;
    unsigned MCKOE : 1;
    unsigned : 22;
} SPI_I2SPR_t;
_Static_assert(sizeof(SPI_I2SPR_t) == 4U, "SPI_I2SPR_t is wrong size");

typedef struct
{
    SPI_CR1_t CR1;
    SPI_CR2_t CR2;
    SPI_SR_t SR;
    uint32_t DR;
    uint32_t CRCPR;
    uint32_t RXCRCR;
    uint32_t TXCRCR;
    SPI_I2SCFGR_t I2SCFGR;
    SPI_I2SPR_t I2SPR;
} SPI_t;
_Static_assert(sizeof(SPI_t) == 0x24U, "SPI_t is wrong size");

extern volatile SPI_t SPI1, SPI2, SPI3;

#endif

/**
 * @}
 */
