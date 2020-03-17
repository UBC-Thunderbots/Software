#include "estop.h"

#include <FreeRTOS.h>
#include <gpio.h>
#include <nvic.h>
#include <rcc.h>
#include <registers/adc.h>
#include <semphr.h>
#include <stdlib.h>
#include <timers.h>
#include <unused.h>

#include "pins.h"

#define NOT_BROKEN_THRESHOLD 200U

static SemaphoreHandle_t notify_sem;
static unsigned int not_broken_count = NOT_BROKEN_THRESHOLD;
static estop_t value                 = ESTOP_BROKEN;

static void estop_start_sample(void)
{
    ADC1.CR2.SWSTART = 1;  // Start conversion
}

static void estop_finish_sample(void)
{
    uint32_t reading = ADC1.DR.DATA;
    // EStop open (stop) produces nominal 1.086 V
    // EStop closed (run) produces nominal 2.242 V
    //
    // Dividing the space:
    // 0–0.543 → broken
    // 0.543–1.664 → open
    // 1.664–2.771 → closed
    // 2.771–3.3 → broken
    if (reading < (uint32_t)(0.543 / 3.3 * 4096))
    {
        // Broken
        value = ESTOP_BROKEN;
    }
    else if (reading < (uint32_t)(1.664 / 3.3 * 4096))
    {
        // Open
        value = ESTOP_STOP;
    }
    else if (reading < (uint32_t)(2.771 / 3.3 * 4096))
    {
        // Closed
        value = ESTOP_RUN;
    }
    else
    {
        // Broken
        value = ESTOP_BROKEN;
    }
}

static void tick(TimerHandle_t UNUSED(timer))
{
    estop_start_sample();
}

void estop_init(void)
{
    // Initialize the notification source.
    __atomic_store_n(&notify_sem, 0, __ATOMIC_RELAXED);
    __atomic_signal_fence(__ATOMIC_ACQ_REL);

    // Send power to the switch
    gpio_set(PIN_ESTOP_SUPPLY);

    // Configure the ADC.
    // Reset signalling is directed to all three ADCs simultaneously.
    rcc_enable(APB2, ADC1);
    rcc_reset(APB2, ADC);
    {
        ADC_CR1_t tmp = {
            .EOCIE   = 0,  // End of conversion interrupt disabled.
            .AWDIE   = 0,  // Analogue watchdog interrupt disabled.
            .JEOCIE  = 0,  // End of injected conversion interrupt disabled.
            .SCAN    = 0,  // Scan mode disabled.
            .JAUTO   = 0,  // Automatic injected group conversion disabled.
            .DISCEN  = 0,  // Discontinuous mode on regular channels disabled.
            .JDISCEN = 0,  // Discontinuous mode on injected channels disabled.
            .JAWDEN  = 0,  // Analogue watchdog on injected channels disabled.
            .AWDEN   = 0,  // Analogue watchdog on regular channels disabled.
            .RES     = 0,  // 12-bit resolution.
            .OVRIE   = 0,  // Overrun interrupt disabled.
        };
        ADC1.CR1 = tmp;
    }
    {
        ADC_CR2_t tmp = {
            .ADON  = 1,  // Enable ADC.
            .CONT  = 0,  // Single conversion per trigger.
            .DMA   = 0,  // DMA disabled.
            .ALIGN = 0,  // Data right-aligned in buffer.
            .JEXTEN =
                0,  // No external trigger for injected channels to start converting.
            .JSWSTART = 0,  // Do not start converting injected channels now.
            .EXTEN = 0,  // No external trigger for regular channels to start converting.
            .SWSTART = 0,  // Do not start converting regular channels now.
        };
        ADC1.CR2 = tmp;
    }
    vTaskDelay(1U);
    ADC1.SMPR2.SMP9 = 1U;  // Sample time on channel 9 is 15 cycles.
    ADC1.SQR1.L     = 0U;  // Regular conversion sequence has one channel.
    ADC1.SQR3.SQ1   = 9U;  // First regular channel to convert is 9.

    // Take one sample now
    estop_start_sample();
    while (!ADC1.SR.EOC)
    {
        vTaskDelay(1U);
    }
    estop_finish_sample();

    // Enable ADC completion interrupts
    ADC1.CR1.EOCIE = 1;  // Enable interrupt on end of conversion
    portENABLE_HW_INTERRUPT(NVIC_IRQ_ADC);

    // Set up a 50 ms timer to run the ADC.
    static StaticTimer_t timer_storage;
    TimerHandle_t timer = xTimerCreateStatic("estop", 50U / portTICK_PERIOD_MS, pdTRUE, 0,
                                             &tick, &timer_storage);
    if (!xTimerStart(timer, portMAX_DELAY))
    {
        abort();
    }
}

estop_t estop_read(void)
{
    return __atomic_load_n(&value, __ATOMIC_RELAXED);
}

void estop_set_sem(SemaphoreHandle_t sem)
{
    __atomic_signal_fence(__ATOMIC_ACQ_REL);
    __atomic_store_n(&notify_sem, sem, __ATOMIC_RELAXED);
    __atomic_signal_fence(__ATOMIC_ACQ_REL);
}

void adc_isr(void)
{
    estop_t old = value;
    estop_finish_sample();
    if (value == ESTOP_BROKEN)
    {
        not_broken_count = 0U;
    }
    else if (not_broken_count < NOT_BROKEN_THRESHOLD)
    {
        ++not_broken_count;
        value = ESTOP_BROKEN;
    }
    if (value != old && notify_sem)
    {
        BaseType_t yield = pdFALSE;
        // No need to check return value.
        // If semaphore was already given, an update will happen in due time.
        // Resulting estop value will be eventually consistent.
        xSemaphoreGiveFromISR(notify_sem, &yield);
        if (yield)
        {
            portYIELD_FROM_ISR();
        }
    }

    EXCEPTION_RETURN_BARRIER();
}
