/**
 * \defgroup FEEDBACK Feedback Packet Functions
 *
 * \brief These functions relate to sending feedback packets over the radio.
 *
 * \{
 */

#include "io/feedback.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <build_id.h>
#include <math.h>
#include <minmax.h>
#include <stack.h>
#include <string.h>
#include <unused.h>

#include "io/adc.h"
#include "io/breakbeam.h"
#include "io/charger.h"
#include "io/dribbler.h"
#include "io/encoder.h"
#include "io/hall.h"
#include "io/icb.h"
#include "io/lps.h"
#include "io/motor.h"
#include "io/mrf.h"
#include "io/sdcard.h"
#include "io/wheels.h"
#include "main.h"
#include "priority.h"
#include "upgrade/fpga.h"
#include "util/error.h"
#include "util/log.h"

#define HAS_BALL_MIN_PERIOD (10U / portTICK_PERIOD_MS)

typedef enum
{
    EVENT_SEND_NORMAL   = 0x01,
    EVENT_SEND_HAS_BALL = 0x02,
    EVENT_SEND_AUTOKICK = 0x04,
    EVENT_SHUTDOWN      = 0x08,
} event_t;

static unsigned int has_ball_antispam_ticks = 0U;
static bool has_ball_after_antispam         = false;
static bool build_ids_pending               = false;
static TaskHandle_t feedback_task_handle;

static void feedback_task(void *UNUSED(param))
{
    static uint8_t nullary_frame[] = {
        9U,          // Header length
        9U + 1U,     // Total length
        0b01100001,  // Frame control LSB (data frame, no security, no frame pending, ack
                     // request, intra-PAN)
        0b10001000,  // Frame control MSB (16-bit destination address, 16-bit source
                     // address)
        0U,          // [4] Sequence number
        0U,          // [5] PAN ID LSB
        0U,          // [6] PAN ID MSB
        0x00U,       // Destination address LSB
        0x01U,       // Destination address MSB
        0U,          // [9] Source address LSB
        0U,          // [10] Source address MSB
        0U,          // [11] Packet purpose
    };
    uint32_t pending_events = 0;
    for (;;)
    {
        {
            uint32_t new_events;
            xTaskNotifyWait(0, UINT32_MAX, &new_events,
                            pending_events ? 0 : portMAX_DELAY);
            pending_events |= new_events;
        }

        if (pending_events & EVENT_SHUTDOWN)
        {
            pending_events &= ~EVENT_SHUTDOWN;
            break;
        }
        if (pending_events & EVENT_SEND_NORMAL)
        {
            pending_events &= ~EVENT_SEND_NORMAL;
#define PREFIX_LENGTH 2
#define HEADER_LENGTH 9
#define PURPOSE_LENGTH 1
#define BASIC_LENGTH 12
#define EXTENSIONS_MAX_LENGTH 32
            static uint8_t frame[PREFIX_LENGTH + HEADER_LENGTH + PURPOSE_LENGTH +
                                 BASIC_LENGTH + EXTENSIONS_MAX_LENGTH] = {
                HEADER_LENGTH,  // [0] Header length
                0,              // [1] Total length
                0b01100001,  // [2] Frame control LSB (data frame, no security, no frame
                             // pending, ack request, intra-PAN)
                0b10001000,  // [3] Frame control MSB (16-bit destination address, 16-bit
                             // source address)
                0U,          // [4] Sequence number
                0U,          // [5] PAN ID LSB
                0U,          // [6] PAN ID MSB
                0x00U,       // Destination address LSB
                0x01U,       // Destination address MSB
                0U,          // [9] Source address LSB
                0U,          // [10] Source address MSB
                0U,          // [28] LPS value 1
                0U,          // [29] LPS value 2
                0U,          // [30] LPS value 3
                0U,          // [31] LPS value 4
                             // 0U, // [32] LPS value 1
                // 0U, // [33] LPS value 2
                // 0U, // [34] LPS value 3
                // 0U, // [35] LPS value 4
            };

            // Fill header.
            uint8_t *wptr = frame + PREFIX_LENGTH;
            wptr += 2;                          // Frame control
            *wptr++      = mrf_alloc_seqnum();  // Sequence number
            uint16_t u16 = mrf_pan_id();
            *wptr++      = u16;        // PAN ID LSB
            *wptr++      = u16 >> 8U;  // PAN ID MSB
            wptr += 2;                 // Destination address
            u16     = mrf_short_address();
            *wptr++ = u16;        // Source address LSB
            *wptr++ = u16 >> 8U;  // Source address MSB

            // Fill purpose.
            *wptr++ = 0x00;  // Purpose: general robot status update

            // Fill basic section.
            u16           = (uint16_t)(adc_battery() * 1000.0f);
            *wptr++       = u16;        // Battery LSB
            *wptr++       = u16 >> 8U;  // Battery MSB
            u16           = (uint16_t)(adc_capacitor() * 100.0f);
            *wptr++       = u16;        // Capacitor LSB
            *wptr++       = u16 >> 8U;  // Capacitor MSB
            u16           = (uint16_t)(breakbeam_difference() * 1000.0f);
            *wptr++       = u16;        // Breakbeam LSB
            *wptr++       = u16 >> 8U;  // Breakbeam MSB
            u16           = (uint16_t)(adc_temperature() * 100.0f);
            *wptr++       = u16;        // System temperature LSB
            *wptr++       = u16 >> 8U;  // System temperature MSB
            uint8_t flags = 0;
            if (breakbeam_interrupted())
            {
                flags |= 0x80U;
            }
            if (charger_full())
            {
                flags |= 0x40U;
            }
            *wptr++     = flags | log_state();  // Flags and log state
            *wptr++     = log_last_error();     // SD card error
            int16_t i16 = hall_speed(4U);
            *wptr++     = (uint8_t)(uint16_t)i16;            // Dribbler speed LSB
            *wptr++     = (uint8_t)(((uint16_t)i16) >> 8U);  // Dribbler speed MSB
            *wptr++     = (uint8_t)MIN(255, dribbler_temperature());  // Dribbler temp

            // Fill errors extension.
            bool do_errors = error_any_latched(ERROR_CONSUMER_MRF);
            if (do_errors)
            {
                *wptr++ = 0x00;  // Error report extension code.
                error_pre_report(ERROR_CONSUMER_MRF, wptr);
                wptr += ERROR_BYTES;
            }

            // Fill build IDs extension.
            bool do_build_ids = __atomic_load_n(&build_ids_pending, __ATOMIC_RELAXED);
            if (do_build_ids)
            {
                *wptr++      = 0x01;  // Build IDs extension code.
                uint32_t bid = build_id_get();
                memcpy(wptr, &bid, sizeof(bid));
                wptr += sizeof(bid);
                bid = upgrade_fpga_build_id();
                memcpy(wptr, &bid, sizeof(bid));
                wptr += sizeof(bid);
            }

            // Fill LPS data extension.
            {
                *wptr++ = 0x02;  // LPS data extension code.
                lps_values val;
                lps_get_pos(val);
                *wptr++ = (int8_t)(val[0] * 10.0f);
                *wptr++ = (int8_t)(val[1] * 10.0f);
                *wptr++ = (uint8_t)(val[2] * 10.0f);
                *wptr++ = 0;
            }

            // Fill length prefix and transmit.
            frame[1]               = wptr - frame - PREFIX_LENGTH;
            mrf_tx_result_t result = mrf_transmit(frame);
            if (result == MRF_TX_OK)
            {
                // We no longer need to send a has-ball update, because the
                // information it would convey is in the feedback packet.
                pending_events &= ~EVENT_SEND_HAS_BALL;

                // If a build ID was sent, we don’t need to send it any more in
                // future.
                if (do_build_ids)
                {
                    __atomic_store_n(&build_ids_pending, false, __ATOMIC_RELAXED);
                }
            }

            // Clean up errors if this report was delivered intact.
            error_post_report(ERROR_CONSUMER_MRF, result == MRF_TX_OK);
        }
        if (pending_events & EVENT_SEND_HAS_BALL)
        {
            pending_events &= ~EVENT_SEND_HAS_BALL;
            nullary_frame[4U]  = mrf_alloc_seqnum();
            uint16_t u16       = mrf_pan_id();
            nullary_frame[5U]  = u16;
            nullary_frame[6U]  = u16 >> 8U;
            u16                = mrf_short_address();
            nullary_frame[9U]  = u16;
            nullary_frame[10U] = u16 >> 8U;
            nullary_frame[11U] = breakbeam_interrupted() ? 0x04U : 0x05U;
            // No need to check for failure.
            // If the frame is not delivered, the next feedback packet will give the host
            // fully up-to-date information.
            mrf_transmit(nullary_frame);
        }
        if (pending_events & EVENT_SEND_AUTOKICK)
        {
            pending_events &= ~EVENT_SEND_AUTOKICK;
            nullary_frame[4U]  = mrf_alloc_seqnum();
            uint16_t u16       = mrf_pan_id();
            nullary_frame[5U]  = u16;
            nullary_frame[6U]  = u16 >> 8U;
            u16                = mrf_short_address();
            nullary_frame[9U]  = u16;
            nullary_frame[10U] = u16 >> 8U;
            nullary_frame[11U] = 0x01U;
            if (mrf_transmit(nullary_frame) != MRF_TX_OK)
            {
                // Delivery failed.
                // This message absolutely must go through.
                // If not, the host may believe autokick is still armed even though it
                // isn’t! So, try delivering the message again later.
                pending_events |= EVENT_SEND_AUTOKICK;
            }
        }
    }

    xSemaphoreGive(main_shutdown_sem);
    vTaskSuspend(0);
}

/**
 * \brief Initializes the feedback system.
 */
void feedback_init(void)
{
    static StaticTask_t feedback_task_tcb;
    STACK_ALLOCATE(feedback_task_stack, 4096);
    feedback_task_handle =
        xTaskCreateStatic(&feedback_task, "feedback",
                          sizeof(feedback_task_stack) / sizeof(*feedback_task_stack), 0,
                          PRIO_TASK_FEEDBACK, feedback_task_stack, &feedback_task_tcb);
}

/**
 * \brief Shuts down the feedback system.
 */
void feedback_shutdown(void)
{
    xTaskNotify(feedback_task_handle, EVENT_SHUTDOWN, eSetBits);
    mrf_transmit_cancel();
    xSemaphoreTake(main_shutdown_sem, portMAX_DELAY);
}

/**
 * \brief Marks normal feedback as pending.
 *
 * The feedback task will send feedback as soon as possible after this function is called.
 */
void feedback_pend_normal(void)
{
    xTaskNotify(feedback_task_handle, EVENT_SEND_NORMAL, eSetBits);
}

/**
 * \brief Marks has-ball feedback as pending.
 *
 * The feedback task will send a has-ball message as soon as possible after this function
 * is called.
 */
void feedback_pend_has_ball(void)
{
    taskENTER_CRITICAL();
    if (has_ball_antispam_ticks)
    {
        has_ball_after_antispam = true;
    }
    else
    {
        xTaskNotify(feedback_task_handle, EVENT_SEND_HAS_BALL, eSetBits);
        has_ball_antispam_ticks = HAS_BALL_MIN_PERIOD;
    }
    taskEXIT_CRITICAL();
}

/**
 * \brief Marks autokick feedback as pending.
 *
 * The feedback task will send an autokick fired message as soon as possible after this
 * function is called.
 */
void feedback_pend_autokick(void)
{
    xTaskNotify(feedback_task_handle, EVENT_SEND_AUTOKICK, eSetBits);
}

/**
 * \brief Marks build IDs as pending.
 *
 * The feedback task will send the build IDs as part of the next general status
 * update packet.
 */
void feedback_pend_build_ids(void)
{
    __atomic_store_n(&build_ids_pending, true, __ATOMIC_RELAXED);
}

/**
 * \brief Ticks the feedback module.
 */
void feedback_tick(void)
{
    taskENTER_CRITICAL();
    if (has_ball_antispam_ticks)
    {
        --has_ball_antispam_ticks;
    }
    if (!has_ball_antispam_ticks && has_ball_after_antispam)
    {
        xTaskNotify(feedback_task_handle, EVENT_SEND_HAS_BALL, eSetBits);
        has_ball_after_antispam = false;
    }
    taskEXIT_CRITICAL();
}

/**
 * \}
 */
