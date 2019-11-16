#include "buzzer.h"

#include <FreeRTOS.h>
#include <rcc.h>
#include <registers/timer.h>
#include <timers.h>
#include <unused.h>

/**
 * \brief The number of 100-millisecond periods left before the buzzer should
 * stop sounding.
 */
static unsigned int buzzer_time_left = 0;

/**
 * \brief The buzzer timer task, which disables the buzzer once its time
 * expires.
 */
static void buzzer_timer(TimerHandle_t UNUSED(handle))
{
    unsigned int old_time = buzzer_time_left;
    while (old_time &&
           !__atomic_compare_exchange_n(&buzzer_time_left, &old_time, old_time - 1, false,
                                        __ATOMIC_RELAXED, __ATOMIC_RELAXED))
        ;
    TIM2_5_CCMR2_t tmp = TIM2.CCMR2;
    tmp.O.OC3M = old_time ? 6 : 4;  // Compare output 3 PWM active when count < duty
                                    // cycle, if active, else forced low.
    TIM2.CCMR2 = tmp;
}

void buzzer_init(void)
{
    // Power up the modules
    rcc_enable_reset(APB1, TIM2);

    // Configure timer 2 to drive the buzzer itself in PWM mode
    {
        TIM2_5_CR1_t tmp = {
            .CKD  = 0,  // Timer runs at full clock frequency.
            .ARPE = 0,  // Auto-reload register is not buffered.
            .CMS  = 0,  // Counter counts in one direction.
            .DIR  = 0,  // Counter counts up.
            .OPM  = 0,  // Counter counts forever.
            .URS  = 0,  // Counter overflow, UG bit set, and slave mode update generate an
                        // interrupt.
            .UDIS = 0,  // Updates to control registers are allowed.
            .CEN  = 0,  // Counter is not counting right now.
        };
        TIM2.CR1 = tmp;
    }
    {
        TIM2_5_SMCR_t tmp = {0};  // No external triggers or slave synchronization.
        TIM2.SMCR         = tmp;
    }
    {
        TIM2_5_CCMR2_t tmp = {
            .O =
                {
                    .OC3CE = 0,  // Do not clear compare output 3 on external trigger.
                    .OC3M  = 4,  // Compare output 3 forced low.
                    .CC3S  = 0,  // CC3 channel configured as output.
                },
        };
        TIM2.CCMR2 = tmp;
    }
    {
        TIM2_5_CCER_t tmp = {
            .CC3NP = 0,  // CC3 is an output, so this must be clear.
            .CC3P  = 0,  // CC3 is active high.
            .CC3E  = 1,  // CC3 signal is routed to pin.
        };
        TIM2.CCER = tmp;
    }
    TIM2.CNT     = 0;                        // Clear counter
    TIM2.PSC     = 0;                        // Set prescale 1:1
    TIM2.ARR     = 36000000 * 2 / 2048;      // Set frequency
    TIM2.CCR3    = 36000000 * 2 / 2048 / 2;  // Set duty cycle
    TIM2.CR1.CEN = 1;                        // Counter enabled

    // Start a FreeRTOS timer to count down how long is left before the buzzer
    // turns off.
    static StaticTimer_t timer_storage;
    TimerHandle_t timer = xTimerCreateStatic("buzzer", 100 / portTICK_PERIOD_MS, pdTRUE,
                                             0, &buzzer_timer, &timer_storage);
    xTimerStart(timer, portMAX_DELAY);
}

void buzzer_start(unsigned long millis)
{
    unsigned int ticks    = (unsigned int)((millis + 99) / 100);
    unsigned int old_time = buzzer_time_left;
    while ((ticks > old_time) &&
           !__atomic_compare_exchange_n(&buzzer_time_left, &old_time, ticks, false,
                                        __ATOMIC_RELAXED, __ATOMIC_RELAXED))
        ;
}

void buzzer_stop(void)
{
    __atomic_store_n(&buzzer_time_left, 0, __ATOMIC_RELAXED);
}
