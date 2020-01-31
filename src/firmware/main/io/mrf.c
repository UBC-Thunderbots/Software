/**
 * \defgroup MRF MRF24J40 functions
 *
 * \brief These functions handle initializing the radio and sending and receiving raw
 * frames.
 *
 * The MRF24J40 radio communicates via SPI.
 * However, the protocol it uses is not very amenable to high-performance operation with
 * an advanced microcontroller, such as the STM32F4. The protocol is defined in terms of
 * register reads and writes, with each register being one byte long. A read or write is a
 * complete SPI transaction, comprising one or two bytes of control data (indicating read
 * or write and address) followed by one byte of data. There is no documented capability
 * to read or write a block of consecutive addresses in a single transaction, even though
 * this would significantly ease access to the frame data buffers. This means that a
 * separate SPI transaction must be carried out for each byte of a received or transmitted
 * frame. Because chip select must be toggled between each of these transactions, firmware
 * must be involved—it is impossible to set up the SPI peripheral to run the sequence of
 * transactions using DMA.
 *
 * As a result of the above limitations, rather than connect the radio to the
 * microcontroller, it is instead connected to the FPGA. The FPGA, with its massive
 * parallelism, can easily issue the multiple SPI transactions in quick succession without
 * compromising the performance of other parts of the system.
 *
 * The FPGA’s MRF subsystem exposes two ways for the microcontroller to operate the radio.
 * In \em direct-access mode, the microcontroller uses ICB transactions to start and
 * finish individual single-register read and write operations, effectively providing a
 * slower version of what the microcontroller would do with the radio directly connected.
 * In \em offload mode, the FPGA communicates with the radio to send and receive frames,
 * packing and unpacking them in internal buffers and allowing a whole frame at a time to
 * be transported over the ICB. At power-up, the subsystem is in direct-access mode. The
 * microcontroller uses the direct access ICB transactions to configure the radio, then
 * instructs the FPGA to enable the offload engines. From that point forward,
 * direct-access mode is no longer used and all communication occurs through the offload
 * engines.
 *
 * The receive offload engine handles frames arriving over the air.
 * As soon as a frame arrives, it is stored in the MRF24J40’s receive buffer and a receive
 * interrupt is asserted. The offload engine observes the interrupt flag and reads the
 * frame from the MRF24J40’s receive buffer into a second receive buffer in the FPGA. Once
 * this transfer is complete, the offload engine asserts an ICB IRQ, indicating that a
 * received frame is ready to be provided to the microcontroller. The microcontroller
 * then, at its leisure, issues an ICB transaction to retrieve the length of the frame,
 * then a second transaction to retrieve the frame. The FPGA will only read out a second
 * received frame from the MRF24J40 once the microcontroller has finished retrieving the
 * first frame. However, the MRF24J40 can receive a second frame as soon as the first
 * frame reaches the FPGA’s internal buffer, so the window for dropped frames is very
 * small and does not depend on microcontroller latency.
 *
 * The transmit offload engine handles frames being sent over the air.
 * When the STM32F4 wishes to send a frame, it issues an ICB command to copy the frame
 * data into a buffer in the FPGA. The FPGA then copies the frame over the slower bus to
 * the MRF24J40’s transmit buffer. Once the frame is fully copied into its final location,
 * the FPGA automatically sets the transmit flag, thus sending the frame. When the
 * MRF24J40 is finished trying to transmit, it asserts an interrupt, which the FPGA
 * observes. At this time, the FPGA automatically reads the transmit status register and
 * asserts an ICB IRQ indicating the fact. The microcontroller can then use another ICB
 * transaction to read the retrieved transmit status register value if desired, and begin
 * sending another frame.
 *
 * Both offload engines are activated simultaneously by a single ICB transaction.
 * Internally, the FPGA contains an arbiter which ensures the offload engines coexist and
 * share the SPI bus. As long as the microcontroller obeys the protocol for talking to
 * each offload engine individually, the system will operate properly.
 *
 * @{
 */

#include "io/mrf.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <event_groups.h>
#include <inttypes.h>
#include <semphr.h>
#include <stdio.h>
#include <task.h>

#include "io/icb.h"
#include "io/leds.h"
#include "io/pins.h"
#include "util/error.h"

/**
 * \brief How long to wait for a received packet before resetting the radio.
 */
#define RECEIVE_TIMEOUT pdMS_TO_TICKS(1000)

typedef enum
{
    MRF_REG_SHORT_RXMCR,
    MRF_REG_SHORT_PANIDL,
    MRF_REG_SHORT_PANIDH,
    MRF_REG_SHORT_SADRL,
    MRF_REG_SHORT_SADRH,
    MRF_REG_SHORT_EADR0,
    MRF_REG_SHORT_EADR1,
    MRF_REG_SHORT_EADR2,
    MRF_REG_SHORT_EADR3,
    MRF_REG_SHORT_EADR4,
    MRF_REG_SHORT_EADR5,
    MRF_REG_SHORT_EADR6,
    MRF_REG_SHORT_EADR7,
    MRF_REG_SHORT_RXFLUSH,
    MRF_REG_SHORT_ORDER = 0x10,
    MRF_REG_SHORT_TXMCR,
    MRF_REG_SHORT_ACKTMOUT,
    MRF_REG_SHORT_ESLOTG1,
    MRF_REG_SHORT_SYMTICKL,
    MRF_REG_SHORT_SYMTICKH,
    MRF_REG_SHORT_PACON0,
    MRF_REG_SHORT_PACON1,
    MRF_REG_SHORT_PACON2,
    MRF_REG_SHORT_TXBCON0 = 0x1A,
    MRF_REG_SHORT_TXNCON,
    MRF_REG_SHORT_TXG1CON,
    MRF_REG_SHORT_TXG2CON,
    MRF_REG_SHORT_ESLOTG23,
    MRF_REG_SHORT_ESLOTG45,
    MRF_REG_SHORT_ESLOTG67,
    MRF_REG_SHORT_TXPEND,
    MRF_REG_SHORT_WAKECON,
    MRF_REG_SHORT_FRMOFFSET,
    MRF_REG_SHORT_TXSTAT,
    MRF_REG_SHORT_TXBCON1,
    MRF_REG_SHORT_GATECLK,
    MRF_REG_SHORT_TXTIME,
    MRF_REG_SHORT_HSYMTMRL,
    MRF_REG_SHORT_HSYMTMRH,
    MRF_REG_SHORT_SOFTRST,
    MRF_REG_SHORT_SECCON0 = 0x2C,
    MRF_REG_SHORT_SECCON1,
    MRF_REG_SHORT_TXSTBL,
    MRF_REG_SHORT_RXSR = 0x30,
    MRF_REG_SHORT_INTSTAT,
    MRF_REG_SHORT_INTCON,
    MRF_REG_SHORT_GPIO,
    MRF_REG_SHORT_TRISGPIO,
    MRF_REG_SHORT_SLPACK,
    MRF_REG_SHORT_RFCTL,
    MRF_REG_SHORT_SECCR2,
    MRF_REG_SHORT_BBREG0,
    MRF_REG_SHORT_BBREG1,
    MRF_REG_SHORT_BBREG2,
    MRF_REG_SHORT_BBREG3,
    MRF_REG_SHORT_BBREG4,
    MRF_REG_SHORT_BBREG6 = 0x3E,
    MRF_REG_SHORT_CCAEDTH,
} mrf_reg_short_t;

typedef enum
{
    MRF_REG_LONG_TXNFIFO    = 0x000,
    MRF_REG_LONG_TXBFIFO    = 0x080,
    MRF_REG_LONG_TXGTS1FIFO = 0x100,
    MRF_REG_LONG_TXGTS2FIFO = 0x180,
    MRF_REG_LONG_RFCON0     = 0x200,
    MRF_REG_LONG_RFCON1,
    MRF_REG_LONG_RFCON2,
    MRF_REG_LONG_RFCON3,
    MRF_REG_LONG_RFCON5 = 0x205,
    MRF_REG_LONG_RFCON6,
    MRF_REG_LONG_RFCON7,
    MRF_REG_LONG_RFCON8,
    MRF_REG_LONG_SLPCAL0,
    MRF_REG_LONG_SLPCAL1,
    MRF_REG_LONG_SLPCAL2,
    MRF_REG_LONG_RFSTATE = 0x20F,
    MRF_REG_LONG_RSSI,
    MRF_REG_LONG_SLPCON0,
    MRF_REG_LONG_SLPCON1   = 0x220,
    MRF_REG_LONG_WAKETIMEL = 0x222,
    MRF_REG_LONG_WAKETIMEH,
    MRF_REG_LONG_REMCNTL,
    MRF_REG_LONG_REMCNTH,
    MRF_REG_LONG_MAINCNT0,
    MRF_REG_LONG_MAINCNT1,
    MRF_REG_LONG_MAINCNT2,
    MRF_REG_LONG_MAINCNT3,
    MRF_REG_LONG_TESTMODE = 0x22F,
    MRF_REG_LONG_ASSOEADR0,
    MRF_REG_LONG_ASSOEADR1,
    MRF_REG_LONG_ASSOEADR2,
    MRF_REG_LONG_ASSOEADR3,
    MRF_REG_LONG_ASSOEADR4,
    MRF_REG_LONG_ASSOEADR5,
    MRF_REG_LONG_ASSOEADR6,
    MRF_REG_LONG_ASSOEADR7,
    MRF_REG_LONG_ASSOSADR0,
    MRF_REG_LONG_ASSOSADR1,
    MRF_REG_LONG_UPNONCE0 = 0x240,
    MRF_REG_LONG_UPNONCE1,
    MRF_REG_LONG_UPNONCE2,
    MRF_REG_LONG_UPNONCE3,
    MRF_REG_LONG_UPNONCE4,
    MRF_REG_LONG_UPNONCE5,
    MRF_REG_LONG_UPNONCE6,
    MRF_REG_LONG_UPNONCE7,
    MRF_REG_LONG_UPNONCE8,
    MRF_REG_LONG_UPNONCE9,
    MRF_REG_LONG_UPNONCE10,
    MRF_REG_LONG_UPNONCE11,
    MRF_REG_LONG_UPNONCE12,
    MRF_REG_LONG_KEYFIFO = 0x280,
    MRF_REG_LONG_RXFIFO  = 0x300,
} mrf_reg_long_t;

typedef enum
{
    RX_EVENT_IRQ    = 0x01,
    RX_EVENT_CANCEL = 0x02,
} rx_event_t;

static mrf_settings_t current_settings;
static SemaphoreHandle_t da_irq_sem, tx_irq_sem, tx_mutex;
static EventGroupHandle_t rx_event_group;
static volatile bool tx_cancelled;

static void da_isr(void)
{
    xSemaphoreGive(da_irq_sem);
}

static void tx_isr(void)
{
    xSemaphoreGive(tx_irq_sem);
}

static void rx_isr(void)
{
    xEventGroupSetBits(rx_event_group, RX_EVENT_IRQ);
}

static void rx_fcs_fail_isr(void)
{
    error_et_fire(ERROR_ET_MRF_FCS);
}

static void reset(void)
{
    static const uint8_t PARAM = 0x00U;
    icb_send(ICB_COMMAND_MRF_DA_SET_AUX, &PARAM, sizeof(PARAM));
}

static void release_reset(void)
{
    static const uint8_t PARAM = 0x01U;
    icb_send(ICB_COMMAND_MRF_DA_SET_AUX, &PARAM, sizeof(PARAM));
}

static uint8_t read_short(uint8_t reg)
{
    static uint8_t param;
    param = reg;
    icb_send(ICB_COMMAND_MRF_DA_READ_SHORT, &param, sizeof(param));
    xSemaphoreTake(da_irq_sem, portMAX_DELAY);
    static uint8_t ret;
    icb_receive(ICB_COMMAND_MRF_DA_GET_DATA, &ret, sizeof(ret));
    return ret;
}

static void write_short(uint8_t reg, uint8_t value)
{
    static uint8_t param[2U];
    param[0U] = reg;
    param[1U] = value;
    icb_send(ICB_COMMAND_MRF_DA_WRITE_SHORT, param, sizeof(param));
    xSemaphoreTake(da_irq_sem, portMAX_DELAY);
}

static __attribute__((unused)) uint8_t read_long(uint16_t reg)
{
    static uint8_t param[2U];
    param[0U] = reg >> 8U;
    param[1U] = reg & 0xFFU;
    icb_send(ICB_COMMAND_MRF_DA_READ_LONG, param, sizeof(param));
    xSemaphoreTake(da_irq_sem, portMAX_DELAY);
    static uint8_t ret;
    icb_receive(ICB_COMMAND_MRF_DA_GET_DATA, &ret, sizeof(ret));
    return ret;
}

static void write_long(uint16_t reg, uint8_t value)
{
    static uint8_t param[3U];
    param[0U] = reg >> 8U;
    param[1U] = reg & 0xFFU;
    param[2U] = value;
    icb_send(ICB_COMMAND_MRF_DA_WRITE_LONG, param, sizeof(param));
    xSemaphoreTake(da_irq_sem, portMAX_DELAY);
}

static void init_radio(void)
{
    static uint8_t int_pin;

    // Reset the chip.
    reset();
    vTaskDelay(1U);
    release_reset();
    vTaskDelay(1U);

    // Check the bus by setting a register and reading it back.
    write_short(MRF_REG_SHORT_EADR0, 0x5A);
    if (read_short(MRF_REG_SHORT_EADR0) != 0x5A)
    {
        fputs("Bus readback test failed.\r\n", stdout);
        return;
    }

    // Check that the interrupt pin is not stuck or disconnected.
    vTaskDelay(pdMS_TO_TICKS(10U));
    icb_receive(ICB_COMMAND_MRF_DA_GET_INT, &int_pin, sizeof(int_pin));
    if (!int_pin)
    {
        fputs("INT pin stuck low.\r\n", stdout);
        return;
    }

    // Write a pile of fixed register values.
    static const struct init_elt_long
    {
        uint16_t address;
        uint8_t data;
    } INIT_ELTS[] = {
        {MRF_REG_SHORT_SOFTRST, 0x07},
        {MRF_REG_SHORT_PACON2, 0x98},
        {MRF_REG_SHORT_TXPEND, 0x7C},
        {MRF_REG_SHORT_TXTIME, 0x38},
        {MRF_REG_SHORT_TXSTBL, 0x95},
        {MRF_REG_LONG_RFCON0, 0x03},
        {MRF_REG_LONG_RFCON1, 0x02},
        {MRF_REG_LONG_RFCON2, 0x80},
        {MRF_REG_LONG_RFCON6, 0x90},
        {MRF_REG_LONG_RFCON7, 0x80},
        {MRF_REG_LONG_RFCON8, 0x10},
        {MRF_REG_LONG_SLPCON0, 0x03},
        {MRF_REG_LONG_SLPCON1, 0x21},
        {MRF_REG_SHORT_RXFLUSH, 0x61},
        {MRF_REG_SHORT_BBREG2, 0xB8},
        // Default threshold for bare chip is 0x60 = -69dB.
        // MRF24J40MB LNA has 20 dB gain → −49 dB at the chip.
        // MRF24J40MD LNA has 13.5 dB gain → −55.5 dB at the chip.
        // Geometric average is -52 dB, or 0xB7.
        {MRF_REG_SHORT_CCAEDTH, 0xB7},
        {MRF_REG_SHORT_BBREG6, 0x40},
    };
    for (size_t i = 0; i < sizeof(INIT_ELTS) / sizeof(*INIT_ELTS); ++i)
    {
        if (INIT_ELTS[i].address >= 0x200)
        {
            write_long(INIT_ELTS[i].address, INIT_ELTS[i].data);
        }
        else
        {
            write_short(INIT_ELTS[i].address, INIT_ELTS[i].data);
        }
    }

    // Re-check interrupt pin after polarity change.
    vTaskDelay(pdMS_TO_TICKS(10U));
    icb_receive(ICB_COMMAND_MRF_DA_GET_INT, &int_pin, sizeof(int_pin));
    if (int_pin)
    {
        fputs("INT pin stuck high.\r\n", stdout);
        return;
    }

    // Initialize per-configuration stuff.
    write_long(MRF_REG_LONG_RFCON0, ((current_settings.channel - 0x0B) << 4) | 0x03);
    write_long(MRF_REG_LONG_RFCON3, 0x18);
    write_short(MRF_REG_SHORT_RFCTL, 0x04);
    write_short(MRF_REG_SHORT_RFCTL, 0x00);
    vTaskDelay(1U);
    if (current_settings.symbol_rate)
    {
        write_short(MRF_REG_SHORT_BBREG0, 0x01);
        write_short(MRF_REG_SHORT_BBREG3, 0x34);
        write_short(MRF_REG_SHORT_BBREG4, 0x5C);
        write_short(MRF_REG_SHORT_SOFTRST, 0x02);
    }
    write_short(MRF_REG_SHORT_PANIDL, current_settings.pan_id);
    write_short(MRF_REG_SHORT_PANIDH, current_settings.pan_id >> 8);
    uint64_t mac_address = current_settings.mac_address;
    for (uint8_t i = 0; i < 8; ++i)
    {
        write_short(MRF_REG_SHORT_EADR0 + i, mac_address);
        mac_address >>= 8;
    }
    write_short(MRF_REG_SHORT_SADRH, current_settings.short_address >> 8);
    write_short(MRF_REG_SHORT_SADRL, current_settings.short_address);

    // Enable the external amplifiers.
    // For MRF24J40MB, no separate power regulator control is needed, but GPIO2:1 need to
    // be configured to control the amplifiers and RF switches. For MRF24J40MD, GPIO2:0
    // are needed for the same purpose.
    write_short(MRF_REG_SHORT_GPIO, 0x00);
    write_long(MRF_REG_LONG_TESTMODE, 0x0F);
    write_short(MRF_REG_SHORT_TRISGPIO, 0x3F);

    // Enable interrupts on receive and transmit complete.
    write_short(MRF_REG_SHORT_INTCON, 0b11110110);

    // Enable the offload engines.
    icb_send(ICB_COMMAND_MRF_OFFLOAD, 0, 0U);
}

/**
 * \brief Initializes the radio.
 *
 * \param[in] settings The settings to use.
 */
void mrf_init(const mrf_settings_t *settings)
{
    // Save parameters.
    current_settings = *settings;

    // Create semaphores.
    static StaticSemaphore_t da_irq_sem_storage, tx_irq_sem_storage, tx_mutex_storage;
    static StaticEventGroup_t rx_event_group_storage;
    da_irq_sem     = xSemaphoreCreateBinaryStatic(&da_irq_sem_storage);
    tx_irq_sem     = xSemaphoreCreateBinaryStatic(&tx_irq_sem_storage);
    rx_event_group = xEventGroupCreateStatic(&rx_event_group_storage);
    tx_mutex       = xSemaphoreCreateMutexStatic(&tx_mutex_storage);
    icb_irq_set_vector(ICB_IRQ_MRF_DA, &da_isr);
    icb_irq_set_vector(ICB_IRQ_MRF_TX, &tx_isr);
    icb_irq_set_vector(ICB_IRQ_MRF_RX, &rx_isr);
    icb_irq_set_vector(ICB_IRQ_MRF_RX_FCS_FAIL, &rx_fcs_fail_isr);

    // Initialize the radio.
    init_radio();
}

/**
 * \brief Shuts down the radio.
 */
void mrf_shutdown(void)
{
    // Disable the offload engines.
    icb_send(ICB_COMMAND_MRF_OFFLOAD_DISABLE, 0, 0U);

    // Put the radio in reset.
    static const uint8_t PARAM = 0b00;  // WAKE = 0, /RESET = 0
    icb_send(ICB_COMMAND_MRF_DA_SET_AUX, &PARAM, sizeof(PARAM));
}

/**
 * \brief Returns the PAN ID.
 *
 * \return the PAN ID
 */
uint16_t mrf_pan_id(void)
{
    return current_settings.pan_id;
}

/**
 * \brief Returns the short address.
 *
 * \return the short address
 */
uint16_t mrf_short_address(void)
{
    return current_settings.short_address;
}

/**
 * \brief Allocates a sequence number for a transmitted frame.
 *
 * \return a fresh sequence number
 */
uint8_t mrf_alloc_seqnum(void)
{
    static uint8_t seqnum = 0x37U;
    return seqnum++;
}

/**
 * \brief Transmits a frame.
 *
 * \param[in] frame the frame to transmit, whose first two bytes must be the header length
 * and total frame length (as required by the MRF24J40)
 *
 * \retval MRF_TX_OK if the frame was sent and acknowledged
 * \retval MRF_TX_NO_ACK if the frame was sent but not acknowledged
 * \retval MRF_TX_CCA_FAIL if the frame was not sent because the channel was too busy
 * \retval MRF_TX_CANCELLED if \ref mrf_transmit_cancel was called
 *
 * \pre The radio must have been initialized.
 */
mrf_tx_result_t mrf_transmit(const void *frame)
{
    // Do not proceed if cancelled.
    if (tx_cancelled)
    {
        return MRF_TX_CANCELLED;
    }

    // Only one task can transmit at a time.
    xSemaphoreTake(tx_mutex, portMAX_DELAY);

    // Send the frame.
    size_t frame_length = ((const uint8_t *)frame)[1U];
    icb_send(ICB_COMMAND_MRF_TX_PUSH, frame, frame_length + 2U);

    // Wait until transmit complete.
    xSemaphoreTake(tx_irq_sem, portMAX_DELAY);

    // Check for cancellation.
    if (tx_cancelled)
    {
        return MRF_TX_CANCELLED;
    }

    // Get transmit status.
    static uint8_t txstat_icb;
    uint8_t txstat;
    icb_receive(ICB_COMMAND_MRF_TX_GET_STATUS, &txstat_icb, sizeof(txstat_icb));
    txstat = txstat_icb;

    // Done transmitting.
    xSemaphoreGive(tx_mutex);

    // Check transmit status.
    if (!(txstat & (1U << 0U)))
    {
        return MRF_TX_OK;
    }
    else if (txstat & (1U << 5U))
    {
        return MRF_TX_CCA_FAIL;
    }
    else
    {
        return MRF_TX_NO_ACK;
    }

    // Blink the status light
    TickType_t last_wake_time = xTaskGetTickCount();
    gpio_set(PIN_LED_STATUS);
    vTaskDelayUntil(&last_wake_time, 1000U);
    gpio_reset(PIN_LED_STATUS);
}

/**
 * \brief Cancels an in-progress or future transmit.
 *
 * \pre This function can only be used once. Once a transmit is cancelled, the radio is
 * unusable.
 *
 * \post Any current, and all subsequent, calls to \ref mrf_transmit will return \ref
 * MRF_TX_CANCELLED. \post No invocation of \ref mrf_transmit will block.
 */
void mrf_transmit_cancel(void)
{
    tx_cancelled = true;
    xSemaphoreGive(tx_irq_sem);
}

/**
 * \brief Receives a frame.
 *
 * \param[out] buffer the buffer into which to store the frame
 *
 * \return the number of bytes stored into the buffer, or 0 if the receive was cancelled
 *
 * \pre The radio must have been initialized.
 *
 * \post The buffer contains the 802.15.4 header, followed by the data payload, followed
 * by the FCS, LQI, and RSSI.
 *
 * \warning This function must only be entered by one caller at a time; it is not
 * thread-safe. It does not make sense to try to use it from multiple threads; there is no
 * control over which frame went to which caller.
 */
size_t mrf_receive(void *buffer)
{
    for (;;)
    {
        // Wait until receive complete or cancelled.
        EventBits_t bits =
            xEventGroupWaitBits(rx_event_group, RX_EVENT_IRQ | RX_EVENT_CANCEL, pdTRUE,
                                pdFALSE, RECEIVE_TIMEOUT);
        if (bits & RX_EVENT_IRQ)
        {
            // No mutex is needed because only one task will take the receive IRQ event
            // bit. A second ICB IRQ is not issued until after ICB_COMMAND_MRF_RX_READ is
            // roughly complete. Therefore, the second task cannot see RX_EVENT_IRQ until
            // the first task has finished receiving and everything is stable again.

            // If a cancellation and a receive IRQ occurred simultaneously, stash the
            // cancellation to take next time.
            if (bits & RX_EVENT_CANCEL)
            {
                xEventGroupSetBits(rx_event_group, RX_EVENT_CANCEL);
            }

            // Get frame length.
            static uint8_t length;
            icb_receive(ICB_COMMAND_MRF_RX_GET_SIZE, &length, sizeof(length));
            assert(length);

            // Copy out the frame.
            icb_receive(ICB_COMMAND_MRF_RX_READ, buffer, length);

            return length;
        }
        else if (bits & RX_EVENT_CANCEL)
        {
            // Cancellation event.
            return 0U;
        }
        else
        {
            // Timeout; reset radio.
            fputs("Receive timeout; reset radio.\r\n", stdout);
            xSemaphoreTake(tx_mutex, portMAX_DELAY);
            mrf_shutdown();
            init_radio();
            xSemaphoreGive(tx_mutex);
        }
    }
}

/**
 * \brief Cancels an in-progress or future receive.
 *
 * \pre Only one cancel can be pending at a time; if this function has been called in the
 * past, \ref mrf_receive must have reported the cancellation by returning zero before
 * calling this function again.
 *
 * \post At some point in the future, \ref mrf_receive will return zero, unless it is
 * never called again. \post No invocation of \ref mrf_receive will block until after the
 * cancellation has been delivered.
 */
void mrf_receive_cancel(void)
{
    xEventGroupSetBits(rx_event_group, RX_EVENT_CANCEL);
}

/**
 * @}
 */
