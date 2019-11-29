/**
 * \defgroup ADC Analogue to Digital Converter Functions
 *
 * \brief These functions handle reading the ADCs, converting raw values to voltages, and
 * introducing correction factors to account for inaccuracy.
 *
 * The chip has three ADCs.
 * Some channels are available to only one or two ADCs, while other channels are attached
 * to all three. Our board has the following external analogue inputs from I/O pins on the
 * microcontroller package: \li battery voltage, into a 20 kΩ || 2.2 kΩ divider, on pin 10
 * (PC2 / ADC123_IN12) \li capacitor voltage, into a 2 MΩ || 22 kΩ divider, on pin 11 (PC3
 * / ADC123_IN13) \li break beam sensor voltage, from a divider with phototransistor to
 * ground and 200 Ω to +3.3 V, on pin 24 (PC4 / ADC123_IN14) \li LPS sensor voltage sum,
 * from a MIC7300, on pin 25 (PC5 / ADC123_IN15)
 *
 * We also make use of the following channels, which are provided internally by the
 * STM32F40x: \li temperature sensor, on ADC1_IN16 \li internal voltage reference,
 * nominally 1.21 V, on ADC1_IN17
 *
 * An ADC does not output a measurement of absolute voltage.
 * It outputs, roughly speaking, a number representing the value of an input channel as a
 * fraction of a reference voltage. In the case of the STM32F4, the reference voltage in
 * question is VDD, which we run at nominally 3.3 V. Do not confuse this reference with
 * the “internal voltage reference” mentioned above; the latter is just an ordinary ADC
 * input channel attached to a fixed voltage source. The accuracy of an ADC reading used
 * to measure a voltage therefore depends on three factors: \li the resolution of the ADC,
 * which for the STM32F4 is 12 bits \li the tolerance of components between the signal
 * source and the ADC (e.g. divider resistors) \li the accuracy of the 3.3 V supply An
 * improvement to accuracy of any of these three components improves overall accuracy of
 * the system.
 *
 * Our 3.3 V regulator has a ±1% total tolerance on output voltage.
 * However, line and load regulation specifications, which cover behaviour in the face of
 * transients, are much tighter. We can thus conclude that most of this tolerance is due
 * to a combination of temperature variation and part differences. These factors all have
 * the important characteristic that they vary quite slowly, or not at all, over time for
 * a specific part.
 *
 * The STM32F4’s internal reference voltage source has an even worse tolerance, of nearly
 * ±2.5%. However, nearly all of this tolerance is due to part differences; a mere ±0.2%
 * is due to temperature variation. STMicro includes a calibration constant burnt into ROM
 * which is a reading of the internal voltage reference taken with a known-good VDD value
 * of 3.3 V. This calibration constant can be used to eliminate part differences.
 *
 * This leads to a final sequence of transformations to apply to an analogue input channel
 * to transform it into an actual voltage:
 * -# Measure the internal voltage reference.
 * -# Compute the actual VDD level, working backwards from the measured internal reference
 * value and the calibration constant based on the known-good 3.3 V.
 * -# Measure the desired input channel.
 * -# Use the actual VDD level to convert the raw reading into an actual voltage on the
 * input.
 *
 * In the case of an external input pin, the voltage computed above is reported.
 * In the case of the internal temperature sensor, the algorithm is altered slightly to
 * incorporate further calibration constants that account for part differences in the
 * sensor itself.
 *
 * We do not actually wait for the ADC to sample a channel each time one is requested.
 * Instead, we use the ADC and DMA to continuously sample all channels and store their
 * most recent readings into a memory buffer. All readings are then instantly available.
 *
 * One further wrinkle is sample time.
 * All four of our external input channels require relatively short sample times to
 * achieve good conversion, allowing those channels to be converted very quickly.
 * Unfortunately, the internal voltage reference and temperature sensor require very long
 * sample times (10 µs each). To prevent these channels from negatively impacting the
 * performance of the external channels, we enable two ADCs and their DMA channels
 * simultaneously: \li ADC 1, on DMA controller 2 stream 4 channel 0, reads the internal
 * voltage reference and temperature sensor into one memory buffer \li ADC 2, on DMA
 * controller 2 stream 0 channel 0, reads the external channels into a second memory
 * buffer Fortunately, those channels that require a long sample time (the internal
 * voltage reference and temperature sensor) are precisely those channels that do not
 * change quickly.
 *
 * @{
 */

#include "adc.h"

#include <FreeRTOS.h>
#include <nvic.h>
#include <rcc.h>
#include <registers/adc.h>
#include <registers/dma.h>
#include <stdint.h>
#include <stdio.h>
#include <task.h>

#include "main.h"

#define ADC2_CHANNEL_COUNT 4
#define ADC2_SAMPLE_COUNT 25

#define VREF_CAL_33 (*(const uint16_t *)0x1FFF7A2AU)
#define TEMP_CAL_30 (*(const uint16_t *)0x1FFF7A2CU)
#define TEMP_CAL_110 (*(const uint16_t *)0x1FFF7A2EU)

static uint32_t adc1_raw[2U], adc2_raw[ADC2_SAMPLE_COUNT][ADC2_CHANNEL_COUNT];
static float vdd_from_vref_num;
static float temp_cal_scale, temp_cal_offset;
static float battery_filtered;

/**
 * \brief Configures the ADCs and starts capturing values.
 */
void adc_init(void)
{
    // Precompute values used to convert voltage reference readings into VDD values.
    // Solve for VDD as a function of ADC reading:
    //
    // reading / 4096 × vdd = cal / 4096 × 3.3
    // reading × vdd = cal × 3.3
    // vdd = cal × 3.3 / reading
    //
    // Precompute cal × 3.3, then divide that by reading to get VDD.
    vdd_from_vref_num = 3.3f * VREF_CAL_33;

    // Precompute values used to convert voltage-corrected temperature sensor readings
    // into temperatures. The two calibration values given are raw ADC readings taken
    // at 3.3 V and 30°C and 110°C respectively. Assume linear interpolation and
    // extrapolation based on those two values and solving for temperature as a function
    // of ADC reading:
    //
    // reading = cal30 + (temp - 30) × (cal110 - cal30) / (110 - 30)
    // reading - cal30 = (temp - 30) × (cal110 - cal30) / 80
    // reading × 80 - cal30 × 80 = (temp - 30) × (cal110 - cal30)
    // reading × 80 / (cal110 - cal30) - cal30 × 80 / (cal110 - cal30) = temp - 30
    // temp = reading × 80 / (cal110 - cal30) - cal30 × 80 / (cal110 - cal30) + 30
    //
    // Precompute 80 / (cal110 - cal30) and -cal30 × 80 / (cal110 - cal30) + 30, then
    // multiply reading by the former and add the latter to get temperature.
    temp_cal_scale  = 80.0f / (TEMP_CAL_110 - TEMP_CAL_30);
    temp_cal_offset = -TEMP_CAL_30 * 80.0f / (TEMP_CAL_110 - TEMP_CAL_30) + 30.0f;

    // Enable the modules.
    rcc_enable(APB2, ADC1);
    rcc_enable(APB2, ADC2);
    rcc_reset(APB2, ADC);

    // Clear all interrupts for the DMA streams.
    {
        DMA_LIFCR_t lifcr = {
            .CFEIF2 = 1, .CDMEIF2 = 1, .CTEIF2 = 1, .CHTIF2 = 1, .CTCIF2 = 1};
        DMA2.LIFCR        = lifcr;
        DMA_HIFCR_t hifcr = {
            .CFEIF4 = 1, .CDMEIF4 = 1, .CTEIF4 = 1, .CHTIF4 = 1, .CTCIF4 = 1};
        DMA2.HIFCR = hifcr;
    }

    // Configure DMA 2 stream 2 channel 1 to read ADC 2.
    {
        DMA2.streams[2U].NDTR = sizeof(adc2_raw) / sizeof(uint32_t);
        DMA2.streams[2U].PAR  = &ADC2.DR;
        DMA2.streams[2U].M0AR = adc2_raw;
        DMA_SxFCR_t fcr       = {.DMDIS = 0};  // Direct mode, not using FIFO
        DMA2.streams[2U].FCR  = fcr;
        DMA_SxCR_t scr        = {
            .EN     = 0,                 // Stream not enabled yet
            .PFCTRL = 0,                 // Flow control by DMA controller
            .DIR    = DMA_DIR_P2M,       // Data moved from peripheral to memory
            .CIRC   = 1,                 // Circular buffer mode
            .PINC   = 0,                 // Do not increment peripheral address
            .MINC   = 1,                 // Increment memory address
            .PSIZE  = DMA_DSIZE_WORD,    // Read 32-bit words from peripheral
            .MSIZE  = DMA_DSIZE_WORD,    // Write 32-bit words to memory
            .PL     = 2,                 // Priority 2 (high)
            .DBM    = 0,                 // No double-buffer mode
            .CT     = 0,                 // Using M0AR
            .PBURST = DMA_BURST_SINGLE,  // No burst cycles reading peripheral
            .MBURST = DMA_BURST_SINGLE,  // No burst cycles writing memory
            .CHSEL  = 1U,                // Channel 1
        };
        DMA2.streams[2U].CR = scr;
        scr.EN              = 1;  // Stream enabled now
        DMA2.streams[2U].CR = scr;
    }

    // Configure DMA 2 stream 4 channel 0 to read ADC 1.
    {
        DMA2.streams[4U].NDTR = sizeof(adc1_raw) / sizeof(*adc1_raw);
        DMA2.streams[4U].PAR  = &ADC1.DR;
        DMA2.streams[4U].M0AR = adc1_raw;
        DMA_SxFCR_t fcr       = {.DMDIS = 0};  // Direct mode, not using FIFO
        DMA2.streams[4U].FCR  = fcr;
        DMA_SxCR_t scr        = {
            .EN     = 0,                 // Stream not enabled yet
            .PFCTRL = 0,                 // Flow control by DMA controller
            .DIR    = DMA_DIR_P2M,       // Data moved from peripheral to memory
            .CIRC   = 1,                 // Circular buffer mode
            .PINC   = 0,                 // Do not increment peripheral address
            .MINC   = 1,                 // Increment memory address
            .PSIZE  = DMA_DSIZE_WORD,    // Read 32-bit words from peripheral
            .MSIZE  = DMA_DSIZE_WORD,    // Write 32-bit words to memory
            .PL     = 2,                 // Priority 2 (high)
            .DBM    = 0,                 // No double-buffer mode
            .CT     = 0,                 // Using M0AR
            .PBURST = DMA_BURST_SINGLE,  // No burst cycles reading peripheral
            .MBURST = DMA_BURST_SINGLE,  // No burst cycles writing memory
            .CHSEL  = 0U,                // Channel 0
        };
        DMA2.streams[4U].CR = scr;
        scr.EN              = 1;  // Stream enabled now
        DMA2.streams[4U].CR = scr;
    }

    // Activate the ADCs.
    {
        ADC_CCR_t ccr = {
            .MULTI = 0,  // ADCs operate independently
            .ADCPRE =
                0b01,  // ADC clock (maximum 36 MHz) is APB2 clock (84 MHz) divided by 4
            .VBATE   = 0,  // Disable VBAT channel
            .TSVREFE = 1,  // Enable temperature sensor and voltage reference
        };
        ADC_Common.CCR = ccr;

        ADC_CR1_t cr1 = {
            .EOCIE  = 0,  // Disable end-of-conversion interrupt
            .AWDIE  = 0,  // Disable analogue watchdog interrupt
            .JEOCIE = 0,  // Disable injected end-of-conversion interrupt
            .SCAN   = 1,  // Enable scan mode (convert a sequence of registers instead of
                          // just one)
            .JAUTO  = 0,  // Do not run injected conversion after normal conversion
            .DISCEN = 0,  // Disable discontinuous mode (convert a subset of the sequence
                          // at a time)
            .JAWDEN = 0,  // Disable analogue watchdog on injected channels
            .AWDEN  = 0,  // Disable analogue watchdog on regular channels
            .RES    = 0b00,  // 12-bit resolution
            .OVRIE  = 1,     // Enable data overrun interrupt
        };
        ADC1.CR1 = cr1;
        ADC2.CR1 = cr1;

        ADC_CR2_t cr2 = {
            .ADON = 1,      // Enable ADC
            .CONT = 1,      // Enable continuous conversion (repeat forever instead of
                            // stopping after last channel)
            .DMA  = 1,      // Enable DMA
            .DDS  = 1,      // Do not stop issuing DMA requests when DMA transfer complete
            .EOCS = 0,      // End of conversion is marked after a sequence, not after one
                            // conversion
            .ALIGN    = 0,  // Right-align data in data register
            .JEXTEN   = 0b00,  // Disable external trigger for injected conversion
            .JSWSTART = 0,     // Do not start an injected conversion
            .EXTEN    = 0b00,  // Disable external trigger for regular conversion
            .SWSTART  = 0,     // Do not start a regular conversion
        };
        ADC1.CR2 = cr2;
        ADC2.CR2 = cr2;

        // Sample times are based on ADC clock cycles, which, as described above, are
        // 21 MHz. There is a formula in the datasheet for required number of sample
        // cycles.
        ADC_SMPR1_t adc1_smpr1 = {
            .SMP16 = 0b111,
            .SMP17 = 0b111};  // Both internal sources require at least 10 µs sample time
        ADC1.SMPR1             = adc1_smpr1;
        ADC_SMPR1_t adc2_smpr1 = {
            .SMP12 =
                0b001,  // Battery has effective impedance of 1982 Ω, requires 7 cycles
            .SMP13 = 0b010,  // Capacitor has effective impedance of 21760 Ω, requires 24
                             // cycles
            .SMP14 = 0b001,  // Break beam has effective impedance of ≤ 200 Ω, requires 6
                             // cycles
            .SMP15 =
                0b001,  // LPS has effective impedance of roughly 0 Ω, requires 6 cycles
        };
        ADC2.SMPR1 = adc2_smpr1;

        ADC_SQR1_t adc1_sqr1 = {.L = 1U};  // Two channels in sequence
        ADC1.SQR1            = adc1_sqr1;
        ADC_SQR3_t adc1_sqr3 = {.SQ1 = 16U, .SQ2 = 17U};  // Convert these two channels
        ADC1.SQR3            = adc1_sqr3;

        ADC_SQR1_t adc2_sqr1 = {.L = 3U};  // Four channels in sequence
        ADC2.SQR1            = adc2_sqr1;
        ADC_SQR3_t adc2_sqr3 = {.SQ1 = 12U,
                                .SQ2 = 13U,
                                .SQ3 = 14U,
                                .SQ4 = 15U};  // Convert these four channels
        ADC2.SQR3            = adc2_sqr3;

        cr2.SWSTART = 1;
        ADC1.CR2    = cr2;
        ADC2.CR2    = cr2;
    }

    // Take an interrupt on data overrun. There is no ISR for this, but it
    // should never happen anyway, so crashing is fine.
    portENABLE_HW_INTERRUPT(NVIC_IRQ_ADC);

    // Wait a bit to give the internal channels time to settle and the ADCs time to
    // convert every channel at least once.
    vTaskDelay(1U);

    // Initialize the filtered battery voltage to the snapshot battery voltage.
    battery_filtered = adc_battery_unfiltered();
}

/**
 * \brief Updates time-dependent ADC data.
 *
 * \param[out] record the log record whose \ref breakbeam_diff, \ref battery_voltage, and
 * \ref capacitor_voltage fields will be filled
 */
void adc_tick(log_record_t *record)
{
    float raw      = adc_battery_unfiltered();
    float filtered = (battery_filtered * (199.0f / 200.0f)) + (raw * (1.0f / 200.0f));
    portDISABLE_INTERRUPTS();
    battery_filtered = filtered;
    portENABLE_INTERRUPTS();
    if (filtered < 12.5f)
    {
        printf("Battery critical (%f volts).\r\n", (double)filtered);
        main_shutdown(MAIN_SHUT_MODE_POWER);
    }

    if (record)
    {
        record->tick.battery_voltage   = adc_battery();
        record->tick.capacitor_voltage = adc_capacitor();
    }
}

/**
 * \brief Returns the voltage on the chip’s VDD pin.
 *
 * This voltage is nominally 3.3 V, but varies due to tolerance of the voltage regulator.
 *
 * \return the VDD voltage
 */
float adc_vdd(void)
{
    return vdd_from_vref_num / __atomic_load_n(&adc1_raw[1U], __ATOMIC_RELAXED);
}

/**
 * \brief Returns the temperature of the chip.
 *
 * \return the temperature, in °C
 */
unsigned int adc_temperature(void)
{
    float vdd = adc_vdd();

    // Compute what the reading would be for the current temperature if VDD were
    // exactly 3.3 V, instead of whatever it really is right now.
    float voltage_corrected_reading =
        __atomic_load_n(&adc1_raw[0U], __ATOMIC_RELAXED) / vdd * 3.3f;

    // Convert to a temperature.
    float temp = voltage_corrected_reading * temp_cal_scale + temp_cal_offset;

    // Clamp to zero.
    if (temp < 0.0f)
    {
        temp = 0.0f;
    }
    return (unsigned int)temp;
}

/**
 * \brief Returns the voltage of the battery.
 *
 * The value returned by this function is low-pass filtered.
 *
 * \return the battery voltage, in volts
 */
float adc_battery(void)
{
    // There are no atomic operations for floats, so disable interrupts while reading the
    // variables.
    portDISABLE_INTERRUPTS();
    float ret = battery_filtered;
    portENABLE_INTERRUPTS();
    return ret;
}

/**
 * \brief Returns the instantaneous voltage of the battery.
 *
 * \return the battery voltage, in volts
 */
float adc_battery_unfiltered(void)
{
    float vdd = adc_vdd();
#define DIVIDER_UPPER 20000.0f
#define DIVIDER_LOWER 2200.0f
    return __atomic_load_n(&adc2_raw[0][0U], __ATOMIC_RELAXED) / 4096.0f * vdd /
           DIVIDER_LOWER * (DIVIDER_UPPER + DIVIDER_LOWER);
#undef DIVIDER_UPPER
#undef DIVIDER_LOWER
}

/**
 * \brief Returns the voltage of the capacitor.
 *
 * \return the capacitor voltage, in volts
 */
float adc_capacitor(void)
{
    float vdd = adc_vdd();
#define DIVIDER_UPPER_CAP 200000.0f
#define DIVIDER_UPPER_VDD 100000.0f
#define DIVIDER_LOWER 2200.0f
#define PARALLEL(x, y) (((x) * (y)) / ((x) + (y)))
    float pin_voltage =
        __atomic_load_n(&adc2_raw[0][1U], __ATOMIC_RELAXED) / 4096.0f * vdd;
    float vdd_part = vdd /
                     (DIVIDER_UPPER_VDD + PARALLEL(DIVIDER_LOWER, DIVIDER_UPPER_CAP)) *
                     PARALLEL(DIVIDER_LOWER, DIVIDER_UPPER_CAP);
    float cap_part = pin_voltage - vdd_part;
    return cap_part / PARALLEL(DIVIDER_LOWER, DIVIDER_UPPER_VDD) *
           (DIVIDER_UPPER_CAP + PARALLEL(DIVIDER_LOWER, DIVIDER_UPPER_VDD));
#undef DIVIDER_UPPER_CAP
#undef DIVIDER_UPPER_VDD
#undef DIVIDER_LOWER
#undef PARALLEL
}

/**
 * \brief Returns the voltage at the collector of the break-beam phototransistor.
 *
 * \return the break beam voltage, in volts
 */
float adc_breakbeam(void)
{
    uint32_t sum = 0;
    for (unsigned int i = 0; i != ADC2_SAMPLE_COUNT; ++i)
    {
        sum += __atomic_load_n(&adc2_raw[i][2], __ATOMIC_RELAXED);
    }

    float vdd = adc_vdd();
    return sum / 4096.0f * vdd / ADC2_SAMPLE_COUNT;
}

/**
 * \brief Returns the voltage at the output of the lateral position sensor opamp.
 *
 * \return the LPS voltage, in volts
 */
float adc_lps(void)
{
    uint32_t sum = 0;
    for (unsigned int i = 0; i != ADC2_SAMPLE_COUNT; ++i)
    {
        sum += __atomic_load_n(&adc2_raw[i][3], __ATOMIC_RELAXED);
    }

    float vdd = adc_vdd();
    return sum / 4096.0f * vdd / ADC2_SAMPLE_COUNT;
}

/**
 * @}
 */
