/**
 * \defgroup TICK Tick and Timing Functions
 *
 * \brief These functions handle operations that need to happen at a fixed frequency.
 *
 * There are two classes of ticks.
 *
 * <h1>Normal Ticks</h1>
 *
 * Normal ticks are used for most things that need to happen at a fixed frequency.
 * These ticks occur at 200 Hz.
 * They are implemented in a FreeRTOS task.
 *
 * <h1>Fast Ticks</h1>
 *
 * Fast ticks are used for functions that need to happen very very frequently.
 * Because these operations happen so frequently, it would be very inefficient to put them
 * in a FreeRTOS task. In fact, because they happen at greater than 1 kHz, a FreeRTOS task
 * cannot time them using a standard system tick! Therefore, we run them in a timer ISR
 * instead.
 *
 * The period we choose is 125 µs, equivalent to a frequency is 8 kHz.
 * The following criteria are considered when choosing a period:
 *
 * \li Not too much CPU time must be taken up. A 125 µs interrupt period is equivalent to
 * 21,000 instruction cycles, which, if the ISR is kept small, is very acceptable. \li
 * Functions using analogue inputs must be sure that their inputs have updated between
 * ticks. The ADC has a sample period of less than 4 µs, so this is true. \li External
 * hardware must have had time to settle between ticks. The break beam’s receiver has a
 * rise and fall time of 10 µs, so this is true. \li The interrupt must be fast enough. At
 * 8 m/s, the ball will travel 1 mm in 125 µs, which is acceptable auto-chick latency.
 * \{
 */

#include "tick.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <exception.h>
#include <nvic.h>
#include <rcc.h>
#include <registers/timer.h>
#include <semphr.h>
#include <stack.h>
#include <stdio.h>
#include <task.h>
#include <unused.h>

#include "io/adc.h"
#include "io/breakbeam.h"
#include "io/charger.h"
#include "io/chicker.h"
#include "io/dr.h"
#include "io/dribbler.h"
#include "io/encoder.h"
#include "io/feedback.h"
#include "io/hall.h"
#include "io/leds.h"
#include "io/lps.h"
#include "io/motor.h"
#include "io/receive.h"
#include "io/wheels.h"
#include "main.h"
#include "priority.h"

// Verify that all the timing requirements are set up properly.
_Static_assert(portTICK_PERIOD_MS *CONTROL_LOOP_HZ == 1000U,
               "Tick rate is not equal to control loop period.");

static bool shutdown = false;
static FirmwareWorld_t *world;
static PrimitiveManager_t *primitive_manager;

static void normal_task(void *UNUSED(param))
{
    TickType_t last_wake = xTaskGetTickCount();

    while (!__atomic_load_n(&shutdown, __ATOMIC_RELAXED))
    {
        // Wait one system tick.
        vTaskDelayUntil(&last_wake, 1U);

        // Sanity check: the FPGA must not have experienced a post-configuration CRC
        // failure.
        assert(gpio_get_input(PIN_FPGA_INIT_B));

        // Try to get a data logger record.
        log_record_t *record = log_alloc();
        if (record)
        {
            record->magic = LOG_MAGIC_TICK;
            error_pre_report(ERROR_CONSUMER_LOG, record->tick.errors);
            error_post_report(ERROR_CONSUMER_LOG, true);
        }

        // Run the stuff.
        feedback_tick();
        receive_tick(record);
        adc_tick(record);
        leds_tick();
        breakbeam_tick(record);
        lps_tick();

        if (chicker_auto_fired_test_clear())
        {
            feedback_pend_autokick();
        }

        hall_tick();
        encoder_tick();
        app_primitive_manager_runCurrentPrimitive(primitive_manager, world);
        dr_tick(record);
        wheels_tick(record);
        dribbler_tick(record);
        charger_tick();
        chicker_tick();
        motor_tick();

        // Submit the log record, if we filled one.
        if (record)
        {
            unsigned int current_primitive_index =
                app_primitive_manager_getCurrentPrimitiveIndex(primitive_manager);

            // We can only send integer, so set to a really large value if less then zero,
            // so we can at least indicate that _something_ is wrong
            if (current_primitive_index < 0)
            {
                current_primitive_index = 99;
            }

            // We no longer use message serials, but this is kept so that the message
            // struct remains the same because our older tooling can be extremely brittle
            // to changes
            record->tick.drive_serial    = 0;
            record->tick.primitive       = (uint8_t)current_primitive_index;
            record->tick.idle_cpu_cycles = main_read_clear_idle_cycles();
            log_queue(record);
        }

        // Report progress.
        main_kick_wdt(MAIN_WDT_SOURCE_TICK);
    }

    __atomic_signal_fence(__ATOMIC_ACQUIRE);
    xSemaphoreGive(main_shutdown_sem);
    vTaskSuspend(0);
}

/**
 * \brief Initializes the tick generators.
 *
 * \param[in] _world The world the "high level" firmware can use to interact with the
 *                   outside world
 * \param[in] _primitive_manager The primitive manager, used for running primitives
 */
void tick_init(PrimitiveManager_t *_primitive_manager, FirmwareWorld_t *_world)
{
    primitive_manager = _primitive_manager;
    world             = _world;

    // Configure timer 6 to run the fast ticks.
    rcc_enable_reset(APB1, TIM6);
    TIM6.PSC            = 84000000U / 1000000U;  // One microsecond per timer tick
    TIM6.ARR            = 125U;                  // Timer period, in microseconds
    TIM6.CNT            = 0U;
    TIM_basic_EGR_t egr = {
        .UG = 1,  // Generate an update event to load PSC and ARR
    };
    TIM6.EGR          = egr;
    TIM_basic_SR_t sr = {
        .UIF = 0,  // Clear the interrupt generated by EGR
    };
    TIM6.SR             = sr;
    TIM_basic_CR1_t cr1 = {
        .CEN = 1,  // Counter enabled
    };
    TIM6.CR1              = cr1;
    TIM_basic_DIER_t dier = {
        .UIE = 1,  // Update interrupt enabled
    };
    TIM6.DIER = dier;
    portENABLE_HW_INTERRUPT(NVIC_IRQ_TIM6_DAC);

    // Fork a task to run the normal ticks, and a semaphore to check when it
    // has terminated.
    static StaticTask_t normal_task_tcb;
    STACK_ALLOCATE(normal_task_stack, 4096);
    xTaskCreateStatic(&normal_task, "tick-normal",
                      sizeof(normal_task_stack) / sizeof(*normal_task_stack), 0,
                      PRIO_TASK_NORMAL_TICK, normal_task_stack, &normal_task_tcb);
}

/**
 * \brief Stops the tick generators.
 */
void tick_shutdown(void)
{
    // Instruct the normal tick task to shut down and wait for it to do so.
    __atomic_store_n(&shutdown, true, __ATOMIC_RELAXED);
    xSemaphoreTake(main_shutdown_sem, portMAX_DELAY);

    // Disable timer 6 interrupts.
    portDISABLE_HW_INTERRUPT(NVIC_IRQ_TIM6_DAC);
}

/**
 * \brief Handles timer 6 interrupts.
 *
 * This function should be registered in the interrupt vector table at position 54.
 */
void timer6_isr(void)
{
    // Clear pending interrupt.
    TIM_basic_SR_t sr = {.UIF = 0};
    TIM6.SR           = sr;

    // Run the devices that need to run fast.
    breakbeam_tick_fast();
    chicker_tick_fast();

    // Run lps fast tick
    lps_incr();

    // Report progress.
    main_kick_wdt(MAIN_WDT_SOURCE_HSTICK);

    EXCEPTION_RETURN_BARRIER();
}

/**
 * \}
 */
