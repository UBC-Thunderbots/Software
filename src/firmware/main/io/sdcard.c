#include "sdcard.h"

#include <FreeRTOS.h>
#include <exception.h>
#include <gpio.h>
#include <inttypes.h>
#include <limits.h>
#include <minmax.h>
#include <nvic.h>
#include <rcc.h>
#include <registers/dma.h>
#include <registers/exti.h>
#include <registers/sdio.h>
#include <registers/syscfg.h>
#include <semphr.h>
#include <stddef.h>
#include <stdio.h>
#include <task.h>

#include "dma.h"
#include "pins.h"
#include "priority.h"

typedef enum
{
    GO_IDLE_STATE           = 0,
    ALL_SEND_CID            = 2,
    SEND_RELATIVE_ADDR      = 3,
    SWITCH_FUNC             = 6,
    SELECT_CARD             = 7,
    SEND_IF_COND            = 8,
    SEND_CSD                = 9,
    SEND_CID                = 10,
    STOP_TRANSMISSION       = 12,
    SEND_STATUS             = 13,
    SET_BLOCKLEN            = 16,
    READ_SINGLE_BLOCK       = 17,
    READ_MULTIPLE_BLOCK     = 18,
    WRITE_BLOCK             = 24,
    WRITE_MULTIPLE_BLOCK    = 25,
    PROGRAM_CSD             = 27,
    SET_WRITE_PROT          = 28,
    CLR_WRITE_PROT          = 29,
    SEND_WRITE_PROT         = 30,
    ERASE_WR_BLK_START_ADDR = 32,
    ERASE_WR_BLK_END_ADDR   = 33,
    ERASE                   = 38,
    LOCK_UNLOCK             = 42,
    APP_CMD                 = 55,
    GEN_CMD                 = 56,
} sd_cmd_t;

typedef enum
{
    SET_BUS_WIDTH          = 6,
    SD_STATUS              = 13,
    SEND_NUM_WR_BLOCKS     = 22,
    SET_WR_BLK_ERASE_COUNT = 23,
    SD_SEND_OP_COND        = 41,
    SET_CLR_CARD_DETECT    = 42,
    SEND_SCR               = 51,
} sd_acmd_t;

typedef enum
{
    COMMAND_FLAG_NO_RESPONSE    = 0x01,
    COMMAND_FLAG_IGNORE_CRC     = 0x02,
    COMMAND_FLAG_LONG_RESPONSE  = 0x04,
    COMMAND_FLAG_IGNORE_RESPCMD = 0x08,
} sd_command_flags_t;

#define STATE_IDLE 0
#define STATE_READY 1
#define STATE_IDENT 2
#define STATE_STBY 3
#define STATE_TRAN 4
#define STATE_DATA 5
#define STATE_RCV 6
#define STATE_PRG 7

#define CLOCK 48000000U

#define SD_DMA_STREAM 6U
#define SD_DMA_CHANNEL 4U

static SemaphoreHandle_t sd_mutex;
static SemaphoreHandle_t int_semaphore = 0, d0_exti_int_semaphore = 0;

typedef struct
{
    bool initialized;
    bool sdhc;
    uint32_t sector_count;
    uint32_t read_dtimer;
} sd_ctx_t;

static sd_ctx_t card_state = {
    .initialized  = false,
    .sdhc         = false,
    .sector_count = 0,
};

/**
 * \brief Handles SD card interrupts.
 *
 * This function should be registered in the application’s interrupt vector table at
 * position 49.
 */
void sd_isr(void)
{
    // clear the mask

    SDIO_MASK_t mask_temp = {0};
    SDIO.MASK             = mask_temp;
    BaseType_t yield      = pdFALSE;
    xSemaphoreGiveFromISR(int_semaphore, &yield);
    if (yield)
    {
        portYIELD_FROM_ISR();
    }

    EXCEPTION_RETURN_BARRIER();
}

/**
 * \brief Handles SD card data pin 0 EXTI interrupts.
 *
 * This function should be registered in the application’s interrupt vector
 * table at position PIN_SD_D0_EXTI_VECTOR.
 */
void sd_d0_exti_isr(void)
{
    // Clear the pending flag.
    EXTI.PR = 1 << PIN_SD_D0_EXTI_PIN;

    // Notify the waiting task.
    BaseType_t yield = pdFALSE;
    xSemaphoreGiveFromISR(d0_exti_int_semaphore, &yield);
    if (yield)
    {
        portYIELD_FROM_ISR();
    }

    EXCEPTION_RETURN_BARRIER();
}

static bool sd_card_present(void)
{
    // When a card is inserted, it will apply a 50 kΩ pull-up resistor to D3.
    // Because the STM32’s internal pull-ups are also on the order of 50 kΩ,
    // they must be disabled. We will drive the pin low, then completely float
    // it and wait a tick. If a card is present, the resistor should pull the
    // pin high very quickly. If no card is present, pin capacitance should
    // keep the pin low.
    //
    // The connector does have a physical card presence switch, but it
    // occasionally fails. We might as well use D3-based detection instead,
    // since if that fails, the card won’t work anyway.
    gpio_reset(PIN_SD_D3);
    gpio_set_mode(PIN_SD_D3, GPIO_MODE_OUT);
    gpio_set_pupd(PIN_SD_D3, GPIO_PUPD_NONE);
    vTaskDelay(1U);
    gpio_set_mode(PIN_SD_D3, GPIO_MODE_IN);
    vTaskDelay(1U);
    bool present = gpio_get_input(PIN_SD_D3);
    gpio_set_pupd(PIN_SD_D3, GPIO_PUPD_PU);
    gpio_set_mode(PIN_SD_D3, GPIO_MODE_AF);
    return present;
}

static float sd_taac_to_nanoseconds(uint8_t taac)
{
    static const float VALUES[16] = {
        0.0f, 1.0f, 1.2f, 1.3f, 1.5f, 2.0f, 2.5f, 3.0f,
        3.5f, 4.0f, 4.5f, 5.0f, 5.5f, 6.0f, 7.0f, 8.0f,
    };
    uint8_t multiplier = taac & 0b00000111;
    uint8_t value      = (taac >> 3) & 15;
    value &= 0b00001111;
    float nanoseconds = VALUES[value];
    while (multiplier--)
    {
        nanoseconds *= 10.0f;
    }
    return nanoseconds;
}

static void sd_clear_cpsm_interrupts(void)
{
    // Clear the flags.
    SDIO_ICR_t temp = {
        .CMDSENTC  = 1,
        .CMDRENDC  = 1,
        .CTIMEOUTC = 1,
        .CCRCFAILC = 1,
    };
    SDIO.ICR = temp;

    // A dummy read appears to be necessary to delay the CPU long enough for the SDIO.STA
    // flags to actually show as clear.
    (void)SDIO.ICR;
}

static void sd_clear_dpsm_interrupts(void)
{
    // Clear the flags.
    SDIO_ICR_t temp = {
        .DBCKENDC  = 1,
        .STBITERRC = 1,
        .DATAENDC  = 1,
        .RXOVERRC  = 1,
        .TXUNDERRC = 1,
        .DTIMEOUTC = 1,
        .DCRCFAILC = 1,
    };
    SDIO.ICR = temp;

    // A dummy read appears to be necessary to delay the CPU long enough for the SDIO.STA
    // flags to actually show as clear.
    (void)SDIO.ICR;
}

static sd_status_t sd_send_command(uint8_t command, uint32_t argument, unsigned int flags)
{
    // All long responses do not contain a response command field.
    if (flags & COMMAND_FLAG_LONG_RESPONSE)
    {
        flags |= COMMAND_FLAG_IGNORE_RESPCMD;
    }

    // Clear all old interrupts.
    sd_clear_cpsm_interrupts();

    // Enable interrupts based on type of command.
    if (flags & COMMAND_FLAG_NO_RESPONSE)
    {
        SDIO_MASK_t temp = {.CMDSENTIE = 1};
        SDIO.MASK        = temp;
    }
    else
    {
        SDIO_MASK_t temp = {.CTIMEOUTIE = 1, .CMDRENDIE = 1, .CCRCFAILIE = 1};
        SDIO.MASK        = temp;
    }

    // Start up command path state machine.
    SDIO.ARG            = argument;
    SDIO_CMD_t cmd_temp = {
        .CPSMEN   = 1,
        .CMDINDEX = command,
    };
    if (flags & COMMAND_FLAG_NO_RESPONSE)
    {
        cmd_temp.WAITRESP = 0b00;
    }
    else if (flags & COMMAND_FLAG_LONG_RESPONSE)
    {
        cmd_temp.WAITRESP = 0b11;
    }
    else
    {
        cmd_temp.WAITRESP = 0b01;
    }
    SDIO.CMD = cmd_temp;

    // Wait for operation to finish.
    xSemaphoreTake(int_semaphore, portMAX_DELAY);

    // Check what happened.
    if (SDIO.STA.CMDREND)
    {
        // Response received with correct CRC.
        if ((flags & COMMAND_FLAG_IGNORE_RESPCMD) || SDIO.RESPCMD == command)
        {
            // Response was for the same command.
            return SD_STATUS_OK;
        }
        else
        {
            iprintf("SD: Command %" PRIu8 " had different response command %" PRIu8
                    ".\r\n",
                    command, (uint8_t)SDIO.RESPCMD);
            return SD_STATUS_ILLEGAL_RESPONSE;
        }
    }
    else if ((flags & COMMAND_FLAG_NO_RESPONSE) && SDIO.STA.CMDSENT)
    {
        // Command sent, and no response expected.
        return SD_STATUS_OK;
    }
    else if (SDIO.STA.CCRCFAIL)
    {
        // CRC failed.
        if (flags & COMMAND_FLAG_IGNORE_CRC)
        {
            return SD_STATUS_OK;
        }
        else
        {
            iprintf("SD: Command %" PRIu8 " CRC error.\r\n", command);
            return SD_STATUS_CRC_ERROR;
        }
    }
    else if (SDIO.STA.CTIMEOUT)
    {
        // Timeout waiting for response.
        iprintf("SD: Command %" PRIu8 " response timeout.\r\n", command);
        return SD_STATUS_COMMAND_RESPONSE_TIMEOUT;
    }
    else
    {
        // No idea how we got here.
        iprintf("SD: Command %" PRIu8 " host controller logic error.\r\n", command);
        return SD_STATUS_LOGICAL_ERROR;
    }
}

// send command in response 1 - no close
static bool sd_send_command_r1(uint8_t command, uint32_t argument, uint8_t state_expected,
                               bool allow_idle_or_ready)
{
    sd_status_t status = sd_send_command(command, argument, 0U);
    if (status != SD_STATUS_OK)
    {
        return status;
    }

    // grab the card response gained by sending command
    uint32_t r1           = SDIO.RESP[0];
    uint8_t current_state = (r1 >> 9) & 0x0F;
    r1 &= 0xFFF9E008;  // zero everything that doesn't need to be checked.

    bool state_ok;
    if (allow_idle_or_ready)
    {
        state_ok = current_state == STATE_IDLE || current_state == STATE_READY;
    }
    else
    {
        state_ok = current_state == state_expected;
    }
    if (!state_ok)
    {
        iprintf("SD: Command %" PRIu8 ": Incorrect starting state %" PRIu8
                ", expected %" PRIu8 ".\r\n",
                command, current_state, state_expected);
        return SD_STATUS_ILLEGAL_STATE;
    }

    if (!r1)
    {
        return SD_STATUS_OK;
    }

    printf("SD: Command %" PRIu8 " failed (R1 = 0x%" PRIX32 ").\r\n", command,
           SDIO.RESP[0U]);

    if ((r1 >> 31) & 0x01)
    {
        return SD_STATUS_OUT_OF_RANGE;
    }
    else if ((r1 >> 30) & 0x01)
    {
        return SD_STATUS_ADDRESS_MISALIGN;
    }
    else if ((r1 >> 29) & 0x01)
    {
        return SD_STATUS_BLOCK_LEN_ERROR;
    }
    else if ((r1 >> 28) & 0x01)
    {
        return SD_STATUS_ERASE_SEQ_ERROR;
    }
    else if ((r1 >> 27) & 0x01)
    {
        return SD_STATUS_ERASE_PARAM;
    }
    else if ((r1 >> 26) & 0x01)
    {
        return SD_STATUS_WP_VIOLATION;
    }
    else if ((r1 >> 25) & 0x01)
    {
        return SD_STATUS_CARD_IS_LOCKED;
    }
    else if ((r1 >> 24) & 0x01)
    {
        return SD_STATUS_LOCK_UNLOCK_FAILED;
    }
    else if ((r1 >> 23) & 0x01)
    {
        return SD_STATUS_COM_CRC_ERROR;
    }
    else if ((r1 >> 22) & 0x01)
    {
        return SD_STATUS_ILLEGAL_COMMAND;
    }
    else if ((r1 >> 21) & 0x01)
    {
        return SD_STATUS_CARD_ECC_FAILED;
    }
    else if ((r1 >> 20) & 0x01)
    {
        return SD_STATUS_CC_ERROR;
    }
    else if ((r1 >> 19) & 0x01)
    {
        return SD_STATUS_ERROR;
    }
    else if ((r1 >> 16) & 0x01)
    {
        return SD_STATUS_CSD_OVERWRITE;
    }
    else if ((r1 >> 15) & 0x01)
    {
        return SD_STATUS_WP_ERASE_SKIP;
    }
    else if ((r1 >> 14) & 0x01)
    {
        return SD_STATUS_CARD_ECC_DISABLED;
    }
    else if ((r1 >> 13) & 0x01)
    {
        return SD_STATUS_ERASE_RESET;
    }
    else if ((r1 >> 3) & 0x01)
    {
        return SD_STATUS_AKE_SEQ_ERROR;
    }
    else
    {
        return SD_STATUS_LOGICAL_ERROR;
    }
}

static sd_status_t sd_send_command_r6(uint8_t command, uint32_t argument, uint16_t *rca,
                                      uint8_t state_expected)
{
    sd_status_t status = sd_send_command(command, argument, 0U);
    if (status != SD_STATUS_OK)
    {
        return status;
    }

    // grab the card response  gained by sending command
    uint32_t r6           = SDIO.RESP[0];
    *rca                  = r6 >> 16;
    uint8_t current_state = (r6 >> 9) & 0x0F;
    r6 &= 0x0000E008;  // zero everything that doesn't need to be checked.

    if (current_state != state_expected)
    {
        iprintf("SD: Command %" PRIu8 ": Incorrect starting state %" PRIu8
                ", expected %" PRIu8 ".\r\n",
                command, current_state, state_expected);
        return SD_STATUS_ILLEGAL_STATE;
    }

    if (!r6)
    {
        return SD_STATUS_OK;
    }

    printf("SD: Command %" PRIu8 " failed (R6 = 0x%" PRIX32 ").\r\n", command,
           SDIO.RESP[0U]);

    if ((r6 >> 15) & 0x01)
    {
        return SD_STATUS_COM_CRC_ERROR;
    }
    else if ((r6 >> 14) & 0x01)
    {
        return SD_STATUS_ILLEGAL_COMMAND;
    }
    else if ((r6 >> 13) & 0x01)
    {
        return SD_STATUS_ERROR;
    }
    else if ((r6 >> 3) & 0x01)
    {
        return SD_STATUS_AKE_SEQ_ERROR;
    }
    else
    {
        return SD_STATUS_LOGICAL_ERROR;
    }
}

/**
 * \brief Waits until pin data 0 goes high.
 *
 * Pin data 0 is held low at the end of a write or erase operation to indicate
 * the card being busy; it goes high when the card is no longer busy. For a
 * write operation this fact is reflected in the status bit \c TXACT, which
 * remains high until the card exits busy status then goes low to indicate the
 * DPSM has returned to idle.
 *
 * However, there are two problems with \c TXACT:
 * \li there is no interrupt that fires when \c TXACT goes low.
 * \li \c TXACT is not used for an erase operation, because an erase operation
 * does not use the DPSM
 *
 * Therefore, this function uses an external interrupt (EXTI) to wait until D0
 * goes high. If desired, one can then spin until \c TXACT goes low, but this
 * should take almost no time as the card has released idle status and all that
 * remains to be done is the DPSM itself to return to idle.
 */
static void sd_wait_d0_high(void)
{
    if (!gpio_get_input(PIN_SD_D0))
    {
        EXTI.RTSR |= 1 << PIN_SD_D0_EXTI_PIN;
        // DANGER: Do not observe the if(!gpio_get_input) above and optimize
        // this to a do…while loop! It is essential that another test happen
        // before taking the semaphore and after setting RTSR, otherwise a race
        // could happen where the pin goes high after the first test but before
        // RTSR is set, which results in deadlock.
        while (!gpio_get_input(PIN_SD_D0))
        {
            xSemaphoreTake(d0_exti_int_semaphore, portMAX_DELAY);
        }
        EXTI.RTSR &= ~(1 << PIN_SD_D0_EXTI_PIN);
    }
}

/**
 * \brief Initializes the SD card.
 *
 * \return the result of the initialization attempt
 */
sd_status_t sd_init(void)
{
    // Create the card access mutex.
    static StaticSemaphore_t sd_mutex_storage;
    sd_mutex = xSemaphoreCreateMutexStatic(&sd_mutex_storage);

    // Check for card.
    if (!sd_card_present())
    {
        fputs("SD: No card\r\n", stdout);
        return SD_STATUS_NO_CARD;
    }

    // Enable the SD host controller.
    rcc_enable_reset(APB2, SDIO);

    // Initialize the STM32 SD card registers
    SDIO.POWER.PWRCTRL = 0b11;
    {
        SDIO_CLKCR_t tmp = {
            .HWFC_EN = 0,  // Errata: hardware flow control doesn’t work.
            .NEGEDGE =
                0,  // Clock output pin matches rising edge of internal clock signal.
            .WIDBUS = 0U,   // One-bit bus mode.
            .BYPASS = 0,    // Do not bypass clock divider.
            .PWRSAV = 0,    // Output clock always, not only when accessing card.
            .CLKEN  = 1,    // Enable clock.
            .CLKDIV = 118,  // 48M/400k = 120 = 118 + 2.
        };
        SDIO.CLKCR = tmp;
    }

    // Setting up interrupts and related things.
    static StaticSemaphore_t int_semaphore_storage, d0_exti_int_semaphore_storage;
    int_semaphore         = xSemaphoreCreateBinaryStatic(&int_semaphore_storage);
    d0_exti_int_semaphore = xSemaphoreCreateBinaryStatic(&d0_exti_int_semaphore_storage);

    // Unmask SD card interrupts.
    portENABLE_HW_INTERRUPT(NVIC_IRQ_SDIO);

    // Unmask DMA interrupts for the channel. These should never actually
    // happen, because the specific interrupt causes we enable for the DMA
    // channel are always error conditions. Thus, there isn’t actually a handler
    // for this interrupt; instead, if it ever happens, it will crash the
    // system.
    portENABLE_HW_INTERRUPT(NVIC_IRQ_DMA2_STREAM6);

    // Configure and unmask the D0 EXTI interrupt, but do not activate the
    // rising edge trigger yet (do that only as needed).
    rcc_enable(APB2, SYSCFG);
    {
        unsigned int ELT   = PIN_SD_D0_EXTI_PIN / 4;
        unsigned int SHIFT = (PIN_SD_D0_EXTI_PIN % 4) * 4;
        SYSCFG.EXTICR[ELT] =
            (SYSCFG.EXTICR[ELT] & ~(15 << SHIFT)) | (PIN_SD_D0_EXTI_PORT << SHIFT);
    }
    rcc_disable(APB2, SYSCFG);
    portENABLE_HW_INTERRUPT(NVIC_IRQ_EXTI9_5);
    EXTI.IMR |= 1 << PIN_SD_D0_EXTI_PIN;

    // Having enabled clocks, wait a little bit for the card to initialize. The
    // SD card specification states, in section 6.4, that after VDD passes its
    // minimum threshold, the host must provide at least one millisecond and at
    // least 74 clock cycles to the card before sending the first command. At
    // 400 kHz, 74 clock cycles is 185 µs, an insignificant amount of time.
    // Just wait one millisecond.
    vTaskDelay(1U);

    // Reset card and enter idle state in SD mode.
    sd_send_command(GO_IDLE_STATE, 0U, COMMAND_FLAG_NO_RESPONSE);

    // Check interface condition and whether card is version 1 or version 2.
    bool v2;
    {
        uint32_t r7;
        sd_status_t ret = sd_send_command(SEND_IF_COND, (0b0001 << 8) | 0x5A, 0U);
        r7              = SDIO.RESP[0];
        if (ret == SD_STATUS_OK)
        {
            // Command was accepted, so this must be a version 2 card.
            v2 = true;
            // Check the interface condition.
            uint8_t check_pattern = r7 & 0xFF;
            if (check_pattern != 0x5A)
            {
                printf("SD: Bad R7: 0x%" PRIX32 ", expected LSB 0x5A\r\n", r7);
                return SD_STATUS_ILLEGAL_RESPONSE;
            }
            uint8_t voltage_mask = (r7 >> 8) & 0x0F;
            if (voltage_mask != 0x1)
            {
                printf("SD: Bad R7: accepted voltage mask=0x%" PRIX8 ", expected 0x1\r\n",
                       voltage_mask);
                return SD_STATUS_INCOMPATIBLE_CARD;
            }
        }
        else if (ret == SD_STATUS_COMMAND_RESPONSE_TIMEOUT)
        {
            // Command response timeout indicates this is a version 1 card, and is not
            // actually an error.
            v2 = false;
        }
        else
        {
            // Command failed for some other reason which should not happen.
            return ret;
        }
    }
    {
        printf("SD: V%c card, interface OK\r\n", v2 ? '2' : '1');
    }

    // 30th bit is HCS -> set it for v2 cards
    uint32_t arg = v2 ? (UINT32_C(1) << 30 | 0x00300000)
                      : 0x00300000;  // Only V2 cards are allowed to see an SDHC host.

    // SD mode specific ACMD41 routine
    // need to be able to time for 1 second. During 1 second, repeatedly send ACMD41 until
    // bit 31 (busy bit) in response to ACMD41 is no longer 0.
    TickType_t start_time = xTaskGetTickCount();
    bool init_done        = false;
    do
    {
        sd_status_t ret = sd_send_command_r1(APP_CMD, 0, STATE_IDLE, true);
        if (ret != SD_STATUS_OK)
        {
            return ret;
        }
        ret = sd_send_command(SD_SEND_OP_COND, arg, COMMAND_FLAG_IGNORE_CRC);
        if (ret != SD_STATUS_OK)
        {
            return ret;
        }
        // when busy bit = 1, initialization complete
        init_done = !!(SDIO.RESP[0] & 0x80000000);
        vTaskDelay(1U);
    } while (!init_done &&
             (xTaskGetTickCount() - start_time < 1000 / portTICK_PERIOD_MS));
    if (!init_done)
    {
        fputs("SD: Card initialization timeout.\r\n", stdout);
        return SD_STATUS_INIT_TIMEOUT;
    }

    if (!(((SDIO.RESP[0] >> 20) & 0x01) || ((SDIO.RESP[0] >> 21) & 0x01)))
    {
        fputs("SD: Unacceptable voltage.\r\n", stdout);
        return SD_STATUS_INCOMPATIBLE_CARD;
    }

    // Determine card capacity class (SDSC vs SDHC/SDXC).
    if (v2)
    {
        card_state.sdhc = (SDIO.RESP[0] >> 30) & 0x01;
    }
    else
    {
        card_state.sdhc = false;
    }
    {
        printf("SD: SD%cC init OK\r\n", card_state.sdhc ? 'H' : 'S');
    }

    // CMD2 and CMD3 to finish in SD mode
    uint16_t rca;
    {
        sd_status_t ret = sd_send_command(ALL_SEND_CID, 0, COMMAND_FLAG_LONG_RESPONSE);
        if (ret != SD_STATUS_OK)
        {
            return ret;
        }
        ret = sd_send_command_r6(SEND_RELATIVE_ADDR, 0, &rca, STATE_IDENT);
        if (ret != SD_STATUS_OK)
        {
            return ret;
        }
    }

    {
        uint8_t csd[16];
        sd_status_t ret =
            sd_send_command(SEND_CSD, rca << 16U, COMMAND_FLAG_LONG_RESPONSE);
        if (ret != SD_STATUS_OK)
        {
            return ret;
        }

        for (unsigned int i = 0; i < 4; ++i)
        {
            for (unsigned int k = 0; k < 4; ++k)
            {
                csd[i * 4 + k] = SDIO.RESP[i] >> (4 - k - 1) * 8;
            }
        }

        uint8_t csd_structure = csd[0] >> 6;
        if (csd_structure == 0)
        {
            // CSD structure version 1.0
            uint8_t read_bl_len = csd[5] & 0x0F;
            uint16_t c_size     = (((uint16_t)(csd[6] & 0x03)) << 10) |
                              (((uint16_t)csd[7]) << 2) | (csd[8] >> 6);
            uint8_t c_size_mult = ((csd[9] & 0x03) << 1) | (csd[10] >> 7);
            uint16_t block_len  = 1 << read_bl_len;
            uint16_t mult       = 1 << (c_size_mult + 2);
            uint32_t blocknr    = ((uint32_t)(c_size + 1)) * mult;
            uint32_t bytes      = blocknr * block_len;

            card_state.sector_count = bytes / 512;

            // Per 4.6.2.1, for SDSC, read timeout is min{100 × (TAAC + NSAC),
            // 100 ms}. Then add an extra 10 ms to account for the time taken
            // for firmware to get through the CPSM, since it starts the DPSM
            // before the CPSM but the timeout according to the spec is from
            // the end of the command to the start of the data.
            float taac_time        = sd_taac_to_nanoseconds(csd[1]) * 1.0e-9f;
            float nsac_time        = ((float)csd[2] * 100.0f) / CLOCK;
            float read_timeout     = MIN(100.0f * (taac_time + nsac_time), 0.1f) + 0.01f;
            card_state.read_dtimer = (uint32_t)(read_timeout * CLOCK + 0.5f);
        }
        else if (csd_structure == 1)
        {
            // CSD structure version 2.0
            uint32_t c_size =
                (((uint32_t)(csd[7] & 0x3F)) << 16) | (((uint32_t)csd[8]) << 8) | csd[9];
            card_state.sector_count =
                (c_size + 1) * 1024;  // bytes = (c_size + 1) * 512 kB

            // Per 4.6.2.1, for SDHC and SDXC, read timeout is to be at least
            // 100 ms and does not depend on CSD. Use 250 ms to have a nice
            // margin of error.
            card_state.read_dtimer = (uint32_t)(0.25f * CLOCK + 0.5f);
        }
        else
        {
            iprintf("SD: CSD_STRUCTURE = %" PRIu8 " (expected 0 or 1)\r\n",
                    (uint8_t)(csd[0] >> 6));
            return SD_STATUS_ILLEGAL_RESPONSE;
        }

        iprintf("SD: %" PRIu32 " sectors\r\n", card_state.sector_count);
    }

    {
        sd_status_t ret = sd_send_command_r1(SELECT_CARD, rca << 16U, STATE_STBY, false);
        if (ret != SD_STATUS_OK)
        {
            return ret;
        }

        // Set block length to 512 bytes (this is ignored in all the ways we
        // care about for SDHC/SDXC, as they always use a 512 byte block
        // length, but harmless).
        ret = sd_send_command_r1(SET_BLOCKLEN, SD_SECTOR_SIZE, STATE_TRAN, false);
        if (ret != SD_STATUS_OK)
        {
            return ret;
        }
    }

    // Set the host controller’s sector size to the same value.
    SDIO.DLEN = (uint32_t)SD_SECTOR_SIZE;

    // Set bus width to 4 bits and increase clock frequency.
    {
        sd_status_t ret = sd_send_command_r1(APP_CMD, rca << 16U, STATE_TRAN, false);
        if (ret != SD_STATUS_OK)
        {
            return ret;
        }
        ret = sd_send_command_r1(SET_BUS_WIDTH, 2U, STATE_TRAN, false);
        if (ret != SD_STATUS_OK)
        {
            return ret;
        }
    }
    SDIO_CLKCR_t clkcr_tmp = SDIO.CLKCR;
    clkcr_tmp.WIDBUS       = 0b01;
    clkcr_tmp.CLKDIV       = 0;
    SDIO.CLKCR             = clkcr_tmp;

    // Initialization is now complete.
    card_state.initialized = true;
    return SD_STATUS_OK;
}

static sd_status_t sd_read_impl(uint32_t sector, void *buffer)
{
    // Check if initialized.
    if (!card_state.initialized)
    {
        return SD_STATUS_UNINITIALIZED;
    }

    // Sanity check.
    assert(dma_check(buffer, SD_SECTOR_SIZE));
    assert(!(((uintptr_t)buffer) & 15U));

    // Clear pending DMA interrupts.
    DMA_HIFCR_t temp_hifcr = {
        .CFEIF6  = 1U,
        .CDMEIF6 = 1U,
        .CTEIF6  = 1U,
        .CHTIF6  = 1U,
        .CTCIF6  = 1U,
    };
    DMA2.HIFCR = temp_hifcr;

    // Initialize DMA engine.
    DMA2.streams[SD_DMA_STREAM].M0AR = buffer;
    DMA2.streams[SD_DMA_STREAM].PAR  = &SDIO.FIFO;
    DMA_SxFCR_t temp_FCR             = {
        .FTH   = DMA_FIFO_THRESHOLD_FULL,
        .DMDIS = 1,
        .FEIE  = 0,
    };
    DMA2.streams[SD_DMA_STREAM].FCR = temp_FCR;
    DMA_SxCR_t temp_CR              = {
        .EN     = 0,
        .DMEIE  = 1,
        .TEIE   = 1,
        .TCIE   = 0,
        .PFCTRL = 1,
        .DIR    = DMA_DIR_P2M,
        .CIRC   = 0,
        .PINC   = 0,
        .MINC   = 1,
        .PSIZE  = DMA_DSIZE_WORD,
        .MSIZE  = DMA_DSIZE_WORD,
        .PINCOS = 0,
        .PL     = 0,
        .DBM    = 0,
        .CT     = 0,
        .PBURST = DMA_BURST_INCR4,
        .MBURST = DMA_BURST_INCR4,
        .CHSEL  = SD_DMA_CHANNEL,
    };
    DMA2.streams[SD_DMA_STREAM].CR = temp_CR;
    temp_CR.EN                     = 1;
    DMA2.streams[SD_DMA_STREAM].CR = temp_CR;

    // Clear old DPSM interrupts.
    sd_clear_dpsm_interrupts();

    // Enable the DPSM before sending the command, because the card may start sending back
    // data at any time.
    SDIO.DTIMER             = card_state.read_dtimer;
    SDIO_DCTRL_t dctrl_temp = {
        .DTEN = 1, .DTDIR = 1, .DTMODE = 0, .DMAEN = 1, .DBLOCKSIZE = 9};
    SDIO.DCTRL = dctrl_temp;

    // Send the command.
    sd_status_t ret = sd_send_command_r1(
        READ_SINGLE_BLOCK, card_state.sdhc ? sector : (sector * SD_SECTOR_SIZE),
        STATE_TRAN, false);
    if (ret != SD_STATUS_OK)
    {
        // Disable the DMA stream.
        temp_CR.EN                     = 0;
        DMA2.streams[SD_DMA_STREAM].CR = temp_CR;
        while (DMA2.streams[SD_DMA_STREAM].CR.EN)
            ;
        SDIO_DCTRL_t dctrl_temp = {.DTEN = 0};
        SDIO.DCTRL              = dctrl_temp;
        return ret;
    }

    // Now that the CPSM is finished, wait for the DPSM to also finish.
    SDIO_MASK_t mask_temp = {.DBCKENDIE  = 1,
                             .STBITERRIE = 1,
                             .RXOVERRIE  = 1,
                             .DCRCFAILIE = 1,
                             .DTIMEOUTIE = 1};
    SDIO.MASK             = mask_temp;
    xSemaphoreTake(int_semaphore, portMAX_DELAY);
    SDIO_STA_t status = SDIO.STA;

    // Decode the completion status.
    if (status.DBCKEND)
    {
        ret = SD_STATUS_OK;
    }
    else if (status.STBITERR)
    {
        fputs("SD: Start bit missing on data line\r\n", stdout);
        ret = SD_STATUS_MISSING_START_BIT;
    }
    else if (status.RXOVERR)
    {
        fputs("SD: FIFO overrun\r\n", stdout);
        ret = SD_STATUS_FIFO_ERROR;
    }
    else if (status.DCRCFAIL)
    {
        fputs("SD: Data CRC failure\r\n", stdout);
        ret = SD_STATUS_DATA_CRC_ERROR;
    }
    else if (status.DTIMEOUT)
    {
        fputs("SD: Data timeout\r\n", stdout);
        ret = SD_STATUS_DATA_TIMEOUT;
    }
    else
    {
        fputs("SD: Unknown SDIO_STA value\r\n", stdout);
        ret = SD_STATUS_LOGICAL_ERROR;
    }

    // Abort the DMA transfer if an error occurred.
    if (ret != SD_STATUS_OK)
    {
        temp_CR.EN                     = 0;
        DMA2.streams[SD_DMA_STREAM].CR = temp_CR;
    }

    // Wait for the DMA controller to shut down.
    while (DMA2.streams[SD_DMA_STREAM].CR.EN)
        ;

    // Ensure all DMA memory writes are complete before CPU memory reads start.
    __atomic_thread_fence(__ATOMIC_ACQUIRE);

    return ret;
}

/**
 * \brief Reads a sector from the SD card.
 *
 * \param[in] sector the sector to read
 *
 * \param[out] buffer a 512-byte buffer in which to store the sector data
 *
 * \return the result of the read attempt
 */
sd_status_t sd_read(uint32_t sector, void *buffer)
{
    xSemaphoreTake(sd_mutex, portMAX_DELAY);
    sd_status_t ret = sd_read_impl(sector, buffer);
    xSemaphoreGive(sd_mutex);
    return ret;
}

static sd_status_t sd_write_impl(uint32_t sector, const void *data)
{
    // Check if initialized.
    if (!card_state.initialized)
    {
        return SD_STATUS_UNINITIALIZED;
    }

    // Sanity check.
    assert(dma_check(data, SD_SECTOR_SIZE));
    assert(!(((uintptr_t)data) & 15U));

    // Clear pending DMA interrupts.
    DMA_HIFCR_t temp_hifcr = {
        .CFEIF6  = 1U,
        .CDMEIF6 = 1U,
        .CTEIF6  = 1U,
        .CHTIF6  = 1U,
        .CTCIF6  = 1U,
    };
    DMA2.HIFCR = temp_hifcr;

    // Ensure all CPU memory writes are complete before DMA memory reads start.
    __atomic_thread_fence(__ATOMIC_RELEASE);

    // Initialize the DMA engine.
    DMA2.streams[SD_DMA_STREAM].M0AR = (void *)data;
    DMA2.streams[SD_DMA_STREAM].PAR  = &SDIO.FIFO;
    DMA_SxFCR_t temp_FCR             = {
        .FTH   = DMA_FIFO_THRESHOLD_FULL,
        .DMDIS = 1,
        .FEIE  = 0,
    };
    DMA2.streams[SD_DMA_STREAM].FCR = temp_FCR;
    DMA_SxCR_t temp_CR              = {
        .EN     = 0,
        .DMEIE  = 1,
        .TEIE   = 1,
        .TCIE   = 0,
        .PFCTRL = 1,
        .DIR    = DMA_DIR_M2P,
        .CIRC   = 0,
        .PINC   = 0,
        .MINC   = 1,
        .PSIZE  = DMA_DSIZE_WORD,
        .MSIZE  = DMA_DSIZE_WORD,
        .PINCOS = 0,
        .PL     = 0,
        .DBM    = 0,
        .CT     = 0,
        .PBURST = DMA_BURST_INCR4,
        .MBURST = DMA_BURST_INCR4,
        .CHSEL  = SD_DMA_CHANNEL,
    };
    DMA2.streams[SD_DMA_STREAM].CR = temp_CR;
    temp_CR.EN                     = 1;
    DMA2.streams[SD_DMA_STREAM].CR = temp_CR;

    // Clear old DPSM interrupts.
    sd_clear_dpsm_interrupts();

    // Send the command.
    sd_status_t ret = sd_send_command_r1(
        WRITE_BLOCK, card_state.sdhc ? sector : (sector * SD_SECTOR_SIZE), STATE_TRAN,
        false);
    if (ret != SD_STATUS_OK)
    {
        // Disable the DMA stream.
        temp_CR.EN                     = 0;
        DMA2.streams[SD_DMA_STREAM].CR = temp_CR;
        while (DMA2.streams[SD_DMA_STREAM].CR.EN)
            ;
        return ret;
    }

    // Enable the DPSM and transfer the data.
    SDIO.DTIMER             = UINT32_MAX;
    SDIO_DCTRL_t dctrl_temp = {
        .DTEN = 1, .DTDIR = 0, .DTMODE = 0, .DMAEN = 1, .DBLOCKSIZE = 9};
    SDIO.DCTRL            = dctrl_temp;
    SDIO_MASK_t mask_temp = {.DBCKENDIE  = 1,
                             .STBITERRIE = 1,
                             .TXUNDERRIE = 1,
                             .DCRCFAILIE = 1,
                             .DTIMEOUTIE = 1};
    SDIO.MASK             = mask_temp;

    // Wait for SD controller to interrupt with error/transfer complete.
    xSemaphoreTake(int_semaphore, portMAX_DELAY);
    SDIO_STA_t status = SDIO.STA;

    // Decode the completion status.
    if (status.DBCKEND)
    {
        ret = SD_STATUS_OK;
    }
    else if (status.STBITERR)
    {
        fputs("SD: Start bit missing on data line\r\n", stdout);
        ret = SD_STATUS_MISSING_START_BIT;
    }
    else if (status.TXUNDERR)
    {
        fputs("SD: FIFO underrun\r\n", stdout);
        ret = SD_STATUS_FIFO_ERROR;
    }
    else if (status.DCRCFAIL)
    {
        fputs("SD: Data CRC failure\r\n", stdout);
        ret = SD_STATUS_DATA_CRC_ERROR;
    }
    else if (status.DTIMEOUT)
    {
        fputs("SD: Data timeout\r\n", stdout);
        ret = SD_STATUS_DATA_TIMEOUT;
    }
    else
    {
        fputs("SD: Unknown SDIO_STA value\r\n", stdout);
        ret = SD_STATUS_LOGICAL_ERROR;
    }

    // Abort the DMA transfer if an error occurred.
    if (ret != SD_STATUS_OK)
    {
        temp_CR.EN                     = 0;
        DMA2.streams[SD_DMA_STREAM].CR = temp_CR;
    }

    // Wait for the DMA controller to shut down.
    while (DMA2.streams[SD_DMA_STREAM].CR.EN)
        ;

    // Write operations use D0 busy/idle signalling; wait for card to go idle.
    sd_wait_d0_high();

    // Wait for DPSM to disable.
    while (SDIO.STA.TXACT)
        ;

    return ret;
}

/**
 * \brief Writes a sector to the SD card.
 *
 * \param[in] sector the sector to write
 *
 * \param[in] data the data to write
 *
 * \return the result of the write attempt
 */
sd_status_t sd_write(uint32_t sector, const void *data)
{
    xSemaphoreTake(sd_mutex, portMAX_DELAY);
    sd_status_t ret = sd_write_impl(sector, data);
    xSemaphoreGive(sd_mutex);
    return ret;
}

static sd_status_t sd_erase_impl(uint32_t sector, size_t count)
{
    // Check if initialized.
    if (!card_state.initialized)
    {
        return SD_STATUS_UNINITIALIZED;
    }

    // Sanity check.
    assert(count != 0U);
    assert(sector + count <= sd_sector_count());

    // Send the commands.
    sd_status_t ret = sd_send_command_r1(
        ERASE_WR_BLK_START_ADDR, card_state.sdhc ? sector : (sector * SD_SECTOR_SIZE),
        STATE_TRAN, false);
    if (ret != SD_STATUS_OK)
    {
        return ret;
    }
    ret = sd_send_command_r1(
        ERASE_WR_BLK_END_ADDR,
        (card_state.sdhc ? (sector + count) : ((sector + count) * SD_SECTOR_SIZE)) - 1U,
        STATE_TRAN, false);
    if (ret != SD_STATUS_OK)
    {
        return ret;
    }
    ret = sd_send_command_r1(ERASE, 0U, STATE_TRAN, false);
    if (ret != SD_STATUS_OK)
    {
        return ret;
    }

    // Erase operations use D0 busy/idle signalling.
    sd_wait_d0_high();

    return SD_STATUS_OK;
}

/**
 * \brief Erases a sequence of sectors on the SD card.
 *
 * \param[in] sector the address of the first sector to erase
 * \param[in] count the number of sectors to erase
 * \return the result of the erase attempt
 */
sd_status_t sd_erase(uint32_t sector, size_t count)
{
    xSemaphoreTake(sd_mutex, portMAX_DELAY);
    sd_status_t ret = sd_erase_impl(sector, count);
    xSemaphoreGive(sd_mutex);
    return ret;
}

/**
 * \brief Returns whether or not the SD card is high-capacity.
 *
 * \retval true an SDHC is inserted
 * \retval false an SDSC is inserted
 */
bool sd_is_hc(void)
{
    return card_state.sdhc;
}

/**
 * \brief Returns the number of sectors on the SD card.
 *
 * \return the number of sectors, or 0 on failure
 */
uint32_t sd_sector_count(void)
{
    return card_state.initialized ? card_state.sector_count : 0;
}
