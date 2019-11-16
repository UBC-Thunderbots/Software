/**
 * \ingroup UPGRADE
 *
 * \{
 */
#include "fw.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <build_id.h>
#include <crc32.h>
#include <gpio.h>
#include <minmax.h>
#include <registers/flash.h>
#include <registers/gpio.h>
#include <registers/mpu.h>
#include <registers/scb.h>
#include <registers/sdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unused.h>
#include <usb.h>
#include <usb_dfu.h>

#include "common.h"
#include "internal.h"
#include "io/dma.h"
#include "io/pins.h"
#include "io/sdcard.h"
#include "main.h"
#include "upgrade/constants.h"
#include "util/constants.h"

/**
 * \brief Erases the entire Flash memory space and copies data from the SD card
 * into it.
 *
 * \pre The SD host controller must be fully initialized and the card brought
 * up into operational mode, with no operation currently in progress.
 *
 * \warning This function must not call any functions that are not also in the
 * <tt>.ramtext</tt> ELF section. This is because functions in other sections
 * live in Flash, and Flash is erased as part of the copying process, meaning
 * those functions no longer exist. Functions in <tt>.ramtext</tt> sections are
 * safe because they are copied from Flash to RAM at system startup, and
 * therefore continue to exist even when Flash is erased.
 *
 * \param[in] length the length of the new firmware, in bytes
 * \param[in] sdhc \c true if the card is high capacity, or \c false if normal
 * capacity
 * \param[in] buffer a buffer used for temporary storage of sector data
 */
static void upgrade_fw_erase_and_copy(size_t length, bool sdhc, void *buffer)
    __attribute__((noclone, noinline, noreturn, section(".ramtext")));
static void upgrade_fw_erase_and_copy(size_t length, bool sdhc, void *buffer)
{
    // Start from the sector immediately following the header.
    uint32_t sector = UPGRADE_FW_FIRST_SECTOR + 1;

    // We can’t take interrupts because they would be handled by an ISR that no
    // longer exists. Disable all interrupts.
    asm volatile("cpsid i");

    // ARMv7-M Architecture Reference Manual B5.2.1 (CPS instruction),
    // Visibility of changes, states that a CPS that increases execution
    // priority serializes the change to the instruction stream, so no ISB is
    // needed here.

    // Do not permit the compiler to hoist non-volatile-qualified memory
    // accesses above this point.
    __atomic_signal_fence(__ATOMIC_ACQUIRE);

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

    // Erase all Flash sectors.
    {
        FLASH_CR_t tmp = {
            .LOCK  = 0,
            .ERRIE = 0,
            .EOPIE = 0,
            .STRT  = 0,
            .PSIZE = 2,
            .SNB   = 0,
            .MER   = 1,
            .SER   = 0,
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

    // Ensure all writes to the control registers occur before any writes to the memory.
    __sync_synchronize();

    // Disable interrupts on the SD host controller.
    {
        SDIO_MASK_t mask = {0};
        SDIO.MASK        = mask;
    }

    // Transfer the data.
    volatile uint32_t *dest = (volatile uint32_t *)0x08000000U;
    while (length)
    {
        // Clear old interrupts.
        SDIO_ICR_t icr_temp = {
            .DBCKENDC  = 1,
            .STBITERRC = 1,
            .DATAENDC  = 1,
            .RXOVERRC  = 1,
            .TXUNDERRC = 1,
            .DTIMEOUTC = 1,
            .DCRCFAILC = 1,
            .CMDSENTC  = 1,
            .CMDRENDC  = 1,
            .CTIMEOUTC = 1,
            .CCRCFAILC = 1,
        };
        SDIO.ICR = icr_temp;

        // A dummy read appears to be necessary to delay the CPU long enough
        // for the SDIO.STA flags to actually show as clear.
        (void)SDIO.ICR;

        // Enable the DPSM before sending the command, because the card may
        // start sending back data at any time.
        SDIO_DCTRL_t dctrl_temp = {
            .DTEN = 1, .DTDIR = 1, .DTMODE = 0, .DMAEN = 0, .DBLOCKSIZE = 9};
        SDIO.DCTRL = dctrl_temp;

        // Start up command path state machine.
        SDIO.ARG            = sdhc ? sector : (sector * SD_SECTOR_SIZE);
        SDIO_CMD_t cmd_temp = {
            .CPSMEN   = 1,
            .CMDINDEX = 17,    // Read single block
            .WAITRESP = 0b01,  // Expect short response
        };
        SDIO.CMD = cmd_temp;

        // Wait for command path operation to finish.
        SDIO_STA_t sta_temp;
        do
        {
            sta_temp = SDIO.STA;
        } while (!(sta_temp.CTIMEOUT || sta_temp.CMDREND || sta_temp.CCRCFAIL));

        // Check what happened.
        if (sta_temp.CMDREND && (SDIO.RESPCMD == 17) &&
            ((SDIO.RESP[0] & 0xFFF9FE28) == (4 /* STATE_TRAN */ << 9)))
        {
            // Now that the CPSM is finished, wait for the DPSM to also finish.
            // Copy data as we go.
            uint32_t *bufptr = buffer;
            do
            {
                sta_temp = SDIO.STA;
                if (sta_temp.RXDAVL)
                {
                    *bufptr++ = SDIO.FIFO;
                }
            } while (!(sta_temp.DBCKEND || sta_temp.DCRCFAIL || sta_temp.DTIMEOUT ||
                       sta_temp.RXOVERR) ||
                     sta_temp.RXDAVL);

            // Check what happened.
            if (sta_temp.DBCKEND && !SDIO.DCOUNT)
            {
                // We received a block. Copy it to Flash and update the
                // accounting data.
                bufptr = buffer;
                for (unsigned int i = 0; i < SD_SECTOR_SIZE / 4; ++i)
                {
                    *dest++ = *bufptr++;
                }
                length -= MIN(length, SD_SECTOR_SIZE);
                ++sector;
            }
            else
            {
                // Something went wrong; abort DPSM and try this block again.
                dctrl_temp.DTEN = 0;
                SDIO.DCTRL      = dctrl_temp;
            }
        }
        else
        {
            // Something broke. Abort DPSM and try the same block again later.
            dctrl_temp.DTEN = 0;
            SDIO.DCTRL      = dctrl_temp;
        }
    }

    // Ensure all writes to the memory finish before touching the control
    // registers.
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

    if (failed)
    {
        // Show the red light.
        GPIO_BSRR_t bsrr_tmp = {
            .BS = 1 << 15,
            .BR = (1 << 13) | (1 << 14),
        };
        GPIOB.BSRR = bsrr_tmp;
        __sync_synchronize();
        for (;;)
            ;
    }
    else
    {
        __sync_synchronize();
        AIRCR_t aircr_tmp     = SCB.AIRCR;
        aircr_tmp.VECTKEY     = 0x05FA;
        aircr_tmp.SYSRESETREQ = 1;
        SCB.AIRCR             = aircr_tmp;
        __sync_synchronize();
        for (;;)
            ;
    }
}

/**
 * \brief Checks if new firmware is available on the SD card and, if so,
 * installs it. Also erases any ephemeral firmware image that is already
 * installed.
 *
 * If new firmware is installed, this function does not return, but rather
 * resets the CPU.
 */
void upgrade_fw_check_install(void)
{
    size_t length;
    uint32_t flags, crc;

    // Check if we have a valid firmware image in the storage area.
    if (upgrade_int_check_area(UPGRADE_FW_FIRST_SECTOR, UPGRADE_FW_MAGIC, &length, &flags,
                               &crc))
    {
        if (crc == build_id_get())
        {
            // The CRC matches the build ID, so firmware is already installed.
            // Erase if ephemeral. In any case, there is no need to install the
            // upgrade.
            if (flags & 1)
            {
                // Do not check the return value. There is nothing useful we
                // can do on error anyway.
                sd_erase(0, UPGRADE_SD_AREA_SECTORS);
            }
        }
        else
        {
            // The upgrade code is different to the code currently on the MCU
            // and should therefore be installed. Turn off LEDs.
            gpio_reset(PIN_LED_STATUS);
            gpio_reset(PIN_LED_LINK);
            gpio_reset(PIN_LED_CHARGED);
            // The MPU needs to be turned off in order to allow execution of
            // code in RAM. Also, turn off all LEDs before
            MPU.CTRL.ENABLE = 0;
            upgrade_fw_erase_and_copy(length, sd_is_hc(),
                                      upgrade_common_get_sector_dma_buffer());
        }
    }
}

/**
 * \}
 */
