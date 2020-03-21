#include <core_progmem.h>
#include <registers/flash.h>

uint32_t core_progmem_dump[256 * 1024 / 4] __attribute__((section(".coredump")));
static volatile uint32_t *pointer;

static void start(void)
{
    // Enable Flash writing.
    FLASH.KEYR = 0x45670123;
    FLASH.KEYR = 0xCDEF89AB;

    // Clear any pending errors.
    while (FLASH.SR.BSY)
        ;
    FLASH.SR = FLASH.SR;

    // Disable and flush caches while erasing Flash.
    FLASH_ACR_t acr = FLASH.ACR;
    acr.PRFTEN      = 0;
    acr.ICEN        = 0;
    acr.DCEN        = 0;
    FLASH.ACR       = acr;
    acr.ICRST       = 1;
    acr.DCRST       = 1;
    FLASH.ACR       = acr;
    acr.ICRST       = 0;
    acr.DCRST       = 0;
    FLASH.ACR       = acr;

    // Erase sectors 10 and 11 which are where we keep core dumps.
    {
        FLASH_CR_t tmp = {
            .LOCK  = 0,
            .ERRIE = 0,
            .EOPIE = 0,
            .STRT  = 0,
            .PSIZE = 2,
            .SNB   = 10,
            .MER   = 0,
            .SER   = 1,
            .PG    = 0,
        };
        FLASH.CR = tmp;
        tmp.STRT = 1;
        FLASH.CR = tmp;
        while (FLASH.SR.BSY)
            ;
        tmp.STRT = 0;
        tmp.SNB  = 11;
        FLASH.CR = tmp;
        tmp.STRT = 1;
        FLASH.CR = tmp;
        while (FLASH.SR.BSY)
            ;
    }

    // Turn caches back on.
    acr.PRFTEN = 1;
    acr.ICEN   = 1;
    acr.DCEN   = 1;
    FLASH.ACR  = acr;

    // Enable Flash programming.
    FLASH.CR.PG = 1;

    // Set up the pointer.
    pointer = core_progmem_dump;

    // Ensure all writes to the control registers occur before any writes to the memory.
    __sync_synchronize();
}

static void write(const void *data, size_t length)
{
    const uint32_t *source = data;
    while (length)
    {
        *pointer = *source;
        while (FLASH.SR.BSY)
            ;
        if (length > 4)
        {
            length -= 4;
        }
        else
        {
            length = 0;
        }
        ++pointer;
        ++source;
    }
}

static bool end(void)
{
    // Ensure all writes to the memory finish before touching the control registers.
    __sync_synchronize();

    // Check if anything failed.
    bool failed =
        !!FLASH.SR.PGSERR || !!FLASH.SR.PGPERR || !!FLASH.SR.PGAERR || !!FLASH.SR.WRPERR;

    // Relock the Flash programming interface.
    {
        FLASH_CR_t tmp = {
            .LOCK  = 1,
            .ERRIE = 0,
            .EOPIE = 0,
            .STRT  = 0,
            .PSIZE = 2,
            .SNB   = 0,
            .MER   = 0,
            .SER   = 0,
            .PG    = 0,
        };
        FLASH.CR = tmp;
    }

    return !failed;
}

const exception_core_writer_t core_progmem_writer = {
    .start = &start,
    .write = &write,
    .end   = &end,
};
