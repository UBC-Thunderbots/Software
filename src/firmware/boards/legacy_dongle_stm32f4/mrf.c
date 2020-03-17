#include "mrf.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <gpio.h>
#include <nvic.h>
#include <rcc.h>
#include <registers/dma.h>
#include <registers/exti.h>
#include <registers/spi.h>
#include <registers/syscfg.h>
#include <semphr.h>
#include <string.h>
#include <task.h>

#include "radio_config.h"

#define DMA_STREAM_RX 0U
#define DMA_STREAM_TX 3U
#define DMA_CHANNEL 3U

static uint8_t txbuf[3U], rxbuf[3U];
static SemaphoreHandle_t dma_int_sem, bus_mutex;
static void (*mrf_isr)(void);

static inline void sleep_50ns(void)
{
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
}

void mrf_init_once(void)
{
    // Create semaphore and mutex.
    static StaticSemaphore_t dma_int_sem_storage, bus_mutex_storage;
    dma_int_sem = xSemaphoreCreateBinaryStatic(&dma_int_sem_storage);
    bus_mutex   = xSemaphoreCreateMutexStatic(&bus_mutex_storage);
}

void mrf_init(void)
{
    // Set bus to reset state.
    gpio_set(PIN_MRF_CS);
    gpio_reset(PIN_MRF_WAKE);
    gpio_reset(PIN_MRF_NRESET);

    // Reset the module and enable the clock.
    rcc_enable_reset(AHB1, DMA2);
    rcc_enable_reset(APB2, SPI1);

    if (SPI1.CR1.SPE)
    {
        // Wait for SPI module to be idle.
        while (!SPI1.SR.TXE || SPI1.SR.BSY)
        {
            if (SPI1.SR.RXNE)
            {
                (void)SPI1.DR;
            }
        }

        // Disable SPI module.
        SPI1.CR1.SPE = 0;  // Disable module
    }

    // Configure the SPI module.
    {
        SPI_CR2_t tmp = {
            .RXDMAEN = 1,  // Receive DMA enabled.
            .TXDMAEN = 1,  // Transmit DMA enabled.
            .SSOE    = 0,  // Do not output hardware slave select; we will use a GPIO for
                           // this purpose as we need to toggle it frequently.
            .FRF    = 0,   // Motorola frame format.
            .ERRIE  = 0,   // Error interrupt disabled.
            .RXNEIE = 0,   // Receive non-empty interrupt disabled.
            .TXEIE  = 0,   // Transmit empty interrupt disabled.
        };
        SPI1.CR2 = tmp;
    }
    {
        SPI_CR1_t tmp = {
            .CPHA = 0,  // Capture on first clock transition, drive new data on second.
            .CPOL = 0,  // Clock idles low.
            .MSTR = 1,  // Master mode.
            .BR   = 2,  // Transmission speed is 72 MHz (APB2) ÷ 8 = 9 MHz.
            .SPE  = 1,  // SPI module is enabled.
            .LSBFIRST = 0,  // Most significant bit is sent first.
            .SSI = 1,  // Module should assume slave select is high → deasserted → no
                       // other master is using the bus.
            .SSM = 1,  // Module internal slave select logic is controlled by software
                       // (SSI bit).
            .RXONLY   = 0,  // Transmit and receive.
            .DFF      = 0,  // Frames are 8 bits wide.
            .CRCNEXT  = 0,  // Do not transmit a CRC now.
            .CRCEN    = 0,  // CRC calculation disabled.
            .BIDIMODE = 0,  // 2-line bidirectional communication used.
        };
        SPI1.CR1 = tmp;
    }

    // Enable the DMA interrupt.
    portENABLE_HW_INTERRUPT(NVIC_IRQ_DMA2_STREAM0);

    // Configure the external interrupt, but do not actually enable it yet.
    rcc_enable(APB2, SYSCFG);
    SYSCFG.EXTICR[12U / 4U] =
        (SYSCFG.EXTICR[12U / 4U] & ~(0xFU << (12U % 4U * 4U))) | (2U << (12U % 4U * 4U));
    rcc_disable(APB2, SYSCFG);
    EXTI.RTSR |= 1U << 12U;     // TR12 = 1; enable rising edge trigger on EXTI12
    EXTI.FTSR &= ~(1U << 12U);  // TR12 = 0; disable falling edge trigger on EXTI12
    EXTI.IMR |= 1U << 12U;      // MR12 = 1; enable interrupt on EXTI12 trigger
}

void mrf_deinit(void)
{
    // Disable the DMA interrupt.
    portDISABLE_HW_INTERRUPT(NVIC_IRQ_DMA2_STREAM0);

    // Wait for DMA channels to be idle.
    while (DMA2.streams[DMA_STREAM_TX].CR.EN)
        ;
    while (DMA2.streams[DMA_STREAM_RX].CR.EN)
        ;

    // Wait for SPI module to be idle and disable.
    if (SPI1.CR1.SPE)
    {
        // Wait for SPI module to be idle.
        while (!SPI1.SR.TXE || SPI1.SR.BSY)
        {
            if (SPI1.SR.RXNE)
            {
                (void)SPI1.DR;
            }
        }

        // Disable SPI module.
        SPI1.CR1.SPE = 0;  // Disable module
    }

    // Disable the SPI module.
    rcc_disable(APB2, SPI1);
    rcc_disable(AHB1, DMA2);

    // Set bus to reset state.
    gpio_set(PIN_MRF_CS);
    gpio_reset(PIN_MRF_WAKE);
    gpio_reset(PIN_MRF_NRESET);
}

void mrf_enable_interrupt(void (*isr)(void))
{
    mrf_disable_interrupt();
    mrf_isr = isr;
    portENABLE_HW_INTERRUPT(NVIC_IRQ_EXTI15_10);
}

void mrf_disable_interrupt(void)
{
    portDISABLE_HW_INTERRUPT(NVIC_IRQ_EXTI15_10);
    mrf_isr = 0;
}

static void execute_transfer(size_t length)
{
    // Assert chip select.
    gpio_reset(PIN_MRF_CS);
    sleep_50ns();

    // Clear old DMA interrupts.
    {
        _Static_assert(DMA_STREAM_RX == 0U,
                       "LIFCR needs rewriting to the proper stream number!");
        _Static_assert(DMA_STREAM_TX == 3U,
                       "LIFCR needs rewriting to the proper stream number!");
        DMA_LIFCR_t lifcr = {
            .CFEIF0  = 1U,  // Clear FIFO error interrupt flag
            .CDMEIF0 = 1U,  // Clear direct mode error interrupt flag
            .CTEIF0  = 1U,  // Clear transfer error interrupt flag
            .CHTIF0  = 1U,  // Clear half transfer interrupt flag
            .CTCIF0  = 1U,  // Clear transfer complete interrupt flag
            .CFEIF3  = 1U,  // Clear FIFO error interrupt flag
            .CDMEIF3 = 1U,  // Clear direct mode error interrupt flag
            .CTEIF3  = 1U,  // Clear transfer error interrupt flag
            .CHTIF3  = 1U,  // Clear half transfer interrupt flag
            .CTCIF3  = 1U,  // Clear transfer complete interrupt flag
        };
        DMA2.LIFCR = lifcr;
    }

    // Ensure all memory writes have reached memory before enabling DMA.
    __atomic_thread_fence(__ATOMIC_RELEASE);

    // Set up receive DMA.
    {
        DMA_SxFCR_t fcr = {
            .FTH   = DMA_FIFO_THRESHOLD_HALF,  // Threshold
            .DMDIS = 1U,                       // Use the FIFO.
        };
        DMA2.streams[DMA_STREAM_RX].FCR  = fcr;
        DMA2.streams[DMA_STREAM_RX].PAR  = &SPI1.DR;
        DMA2.streams[DMA_STREAM_RX].M0AR = rxbuf;
        DMA2.streams[DMA_STREAM_RX].NDTR = length;
        DMA_SxCR_t scr                   = {
            .EN     = 1U,  // Enable DMA engine
            .TCIE   = 1U,  // Enable transfer complete interrupt
            .PFCTRL = 0U,  // DMA engine controls data length
            .DIR    = DMA_DIR_P2M,
            .CIRC   = 0U,  // No circular buffer mode
            .PINC   = 0U,  // Do not increment peripheral address
            .MINC   = 1U,  // Increment memory address
            .PSIZE  = DMA_DSIZE_BYTE,
            .MSIZE  = DMA_DSIZE_BYTE,
            .PINCOS = 0U,  // No special peripheral address increment mode
            .PL     = 1U,  // Priority 1 (medium)
            .DBM    = 0U,  // No double-buffer mode
            .CT     = 0U,  // Use memory pointer zero
            .PBURST = DMA_BURST_SINGLE,
            .MBURST = DMA_BURST_SINGLE,
            .CHSEL  = DMA_CHANNEL,
        };
        DMA2.streams[DMA_STREAM_RX].CR = scr;
    }

    // Set up transmit DMA.
    {
        DMA_SxFCR_t fcr = {
            .FTH   = DMA_FIFO_THRESHOLD_HALF,  // Threshold
            .DMDIS = 1U,                       // Use the FIFO.
        };
        DMA2.streams[DMA_STREAM_TX].FCR  = fcr;
        DMA2.streams[DMA_STREAM_TX].PAR  = &SPI1.DR;
        DMA2.streams[DMA_STREAM_TX].M0AR = txbuf;
        DMA2.streams[DMA_STREAM_TX].NDTR = length;
        DMA_SxCR_t scr                   = {
            .EN     = 1U,  // Enable DMA engine
            .PFCTRL = 0U,  // DMA engine controls data length
            .DIR    = DMA_DIR_M2P,
            .CIRC   = 0U,  // No circular buffer mode
            .PINC   = 0U,  // Do not increment peripheral address
            .MINC   = 1U,  // Increment memory address
            .PSIZE  = DMA_DSIZE_BYTE,
            .MSIZE  = DMA_DSIZE_BYTE,
            .PINCOS = 0U,  // No special peripheral address increment mode
            .PL     = 1U,  // Priority 1 (medium)
            .DBM    = 0U,  // No double-buffer mode
            .CT     = 0U,  // Use memory pointer zero
            .PBURST = DMA_BURST_SINGLE,
            .MBURST = DMA_BURST_SINGLE,
            .CHSEL  = DMA_CHANNEL,
        };
        DMA2.streams[DMA_STREAM_TX].CR = scr;
    }

    // Wait for transfer complete.
    xSemaphoreTake(dma_int_sem, portMAX_DELAY);

    // Wait for SPI module to be idle.
    while (SPI1.SR.BSY)
        ;

    // Ensure all subsequent memory reads occur after DMA is observed complete.
    __atomic_thread_fence(__ATOMIC_ACQUIRE);

    // Deassert CS.
    sleep_50ns();
    gpio_set(PIN_MRF_CS);
}

uint8_t mrf_read_short(mrf_reg_short_t reg)
{
    // Lock the bus.
    xSemaphoreTake(bus_mutex, portMAX_DELAY);

    txbuf[0U] = reg << 1U;
    txbuf[1U] = 0U;
    execute_transfer(2U);
    uint8_t ret = rxbuf[1U];

    // Unlock the bus.
    xSemaphoreGive(bus_mutex);

    return ret;
}

void mrf_write_short(mrf_reg_short_t reg, uint8_t value)
{
    // Lock the bus.
    xSemaphoreTake(bus_mutex, portMAX_DELAY);

    txbuf[0U] = (reg << 1U) | 0x01U;
    txbuf[1U] = value;
    execute_transfer(2U);

    // Unlock the bus.
    xSemaphoreGive(bus_mutex);
}

uint8_t mrf_read_long(mrf_reg_long_t reg)
{
    // Lock the bus.
    xSemaphoreTake(bus_mutex, portMAX_DELAY);

    txbuf[0U] = (reg >> 3U) | 0x80U;
    txbuf[1U] = reg << 5U;
    txbuf[2U] = 0U;
    execute_transfer(3U);
    uint8_t ret = rxbuf[2U];

    // Unlock the bus.
    xSemaphoreGive(bus_mutex);

    return ret;
}

void mrf_write_long(mrf_reg_long_t reg, uint8_t value)
{
    // Lock the bus.
    xSemaphoreTake(bus_mutex, portMAX_DELAY);

    txbuf[0U] = (reg >> 3U) | 0x80U;
    txbuf[1U] = (reg << 5U) | 0x10U;
    txbuf[2U] = value;
    execute_transfer(3U);

    // Unlock the bus.
    xSemaphoreGive(bus_mutex);
}

void mrf_common_init(void)
{
    mrf_write_short(MRF_REG_SHORT_SOFTRST, 0x07);
    mrf_write_short(MRF_REG_SHORT_PACON2, 0x98);
    mrf_write_short(MRF_REG_SHORT_TXPEND, 0x7C);
    mrf_write_short(MRF_REG_SHORT_TXTIME, 0x38);
    mrf_write_short(MRF_REG_SHORT_TXSTBL, 0x95);
    mrf_write_long(MRF_REG_LONG_RFCON0, 0x03);
    mrf_write_long(MRF_REG_LONG_RFCON1, 0x02);
    mrf_write_long(MRF_REG_LONG_RFCON2, 0x80);
    mrf_write_long(MRF_REG_LONG_RFCON6, 0x90);
    mrf_write_long(MRF_REG_LONG_RFCON7, 0x80);
    mrf_write_long(MRF_REG_LONG_RFCON8, 0x10);
    mrf_write_long(MRF_REG_LONG_SLPCON0, 0x03);
    mrf_write_long(MRF_REG_LONG_SLPCON1, 0x21);
    mrf_write_short(MRF_REG_SHORT_RXFLUSH, 0x61);
    mrf_write_short(MRF_REG_SHORT_BBREG2, 0xB8);
    mrf_write_short(MRF_REG_SHORT_CCAEDTH, 0xDD);
    mrf_write_short(MRF_REG_SHORT_BBREG6, 0x40);
    mrf_write_long(MRF_REG_LONG_RFCON0, ((radio_config.channel - 0x0B) << 4) | 0x03);
    mrf_write_long(MRF_REG_LONG_RFCON3, 0x28);
    mrf_write_short(MRF_REG_SHORT_RFCTL, 0x04);
    mrf_write_short(MRF_REG_SHORT_RFCTL, 0x00);
    vTaskDelay(1U);
    if (radio_config.symbol_rate)
    {
        mrf_write_short(MRF_REG_SHORT_BBREG0, 0x01);
        mrf_write_short(MRF_REG_SHORT_BBREG3, 0x34);
        mrf_write_short(MRF_REG_SHORT_BBREG4, 0x5C);
        mrf_write_short(MRF_REG_SHORT_SOFTRST, 0x02);
    }
    mrf_write_short(MRF_REG_SHORT_PANIDL, radio_config.pan_id);
    mrf_write_short(MRF_REG_SHORT_PANIDH, radio_config.pan_id >> 8);
    mrf_write_short(MRF_REG_SHORT_EADR0, radio_config.mac_address);
    mrf_write_short(MRF_REG_SHORT_EADR1, radio_config.mac_address >> 8);
    mrf_write_short(MRF_REG_SHORT_EADR2, radio_config.mac_address >> 16);
    mrf_write_short(MRF_REG_SHORT_EADR3, radio_config.mac_address >> 24);
    mrf_write_short(MRF_REG_SHORT_EADR4, radio_config.mac_address >> 32);
    mrf_write_short(MRF_REG_SHORT_EADR5, radio_config.mac_address >> 40);
    mrf_write_short(MRF_REG_SHORT_EADR6, radio_config.mac_address >> 48);
    mrf_write_short(MRF_REG_SHORT_EADR7, radio_config.mac_address >> 56);
    mrf_analogue_off();
    mrf_write_short(MRF_REG_SHORT_TRISGPIO, 0x3F);
}

void mrf_analogue_off(void)
{
    mrf_write_short(MRF_REG_SHORT_GPIO, 0x00);
    mrf_write_long(MRF_REG_LONG_TESTMODE, 0x08);
}

void mrf_analogue_rx(void)
{
    mrf_write_short(MRF_REG_SHORT_GPIO, 0x04);
    mrf_write_long(MRF_REG_LONG_TESTMODE, 0x08);
}

void mrf_analogue_txrx(void)
{
    mrf_write_short(MRF_REG_SHORT_GPIO, 0x08);
    mrf_write_long(MRF_REG_LONG_TESTMODE, 0x0F);
}

void dma2_stream0_isr(void)
{
    _Static_assert(DMA_STREAM_RX == 0U,
                   "Function needs rewriting to the proper stream number!");
    DMA_LISR_t lisr   = DMA2.LISR;
    DMA_LIFCR_t lifcr = {
        .CFEIF0  = lisr.FEIF0,
        .CDMEIF0 = lisr.DMEIF0,
        .CTEIF0  = lisr.TEIF0,
        .CHTIF0  = lisr.HTIF0,
        .CTCIF0  = lisr.TCIF0,
    };
    assert(!lisr.DMEIF0 && !lisr.TEIF0);
    DMA2.LIFCR       = lifcr;
    BaseType_t yield = pdFALSE;
    BaseType_t ok    = xSemaphoreGiveFromISR(dma_int_sem, &yield);
    assert(ok);
    if (yield)
    {
        portYIELD_FROM_ISR();
    }

    EXCEPTION_RETURN_BARRIER();
}

void exti10_15_isr(void)
{
    // Clear the interrupt.
    EXTI.PR = 1U << 12U;  // PR12 = 1; clear pending EXTI12 interrupt

    // Call the application handler.
    mrf_isr();

    EXCEPTION_RETURN_BARRIER();
}
