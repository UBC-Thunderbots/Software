#include "promiscuous.h"

#include <FreeRTOS.h>
#include <errno.h>
#include <minmax.h>
#include <queue.h>
#include <rcc.h>
#include <registers/exti.h>
#include <registers/timer.h>
#include <semphr.h>
#include <stack.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <unused.h>
#include <usb.h>

#include "constants.h"
#include "crc.h"
#include "enabled.h"
#include "led.h"
#include "mrf.h"
#include "radio_config.h"

typedef struct
{
    uint8_t length;
    uint8_t
        data[1U + 1U + 6U + 127U];  // Flags + channel + timestamp + data (RX FIFO is 128
                                    // bytes long, of which 1 is used for frame length)
} packet_t;

/**
 * \brief The possible task notification bits understood by the radio task.
 */
enum radio_event_t
{
    /**
     * \brief The task should start doing work.
     */
    RADIO_EVENT_START = 0x01,

    /**
     * \brief The task should stop doing work.
     */
    RADIO_EVENT_STOP = 0x02,

    /**
     * \brief A rising edge was detected on the MRF24J40 interrupt pin.
     */
    RADIO_EVENT_INTERRUPT = 0x04,
};

#define NUM_PACKETS 64U

static QueueHandle_t free_queue, receive_queue;
static TaskHandle_t radio_task_handle, usb_task_handle;
static uint16_t promisc_flags;

static void mrf_int_isr(void)
{
    // Give the semaphore.
    BaseType_t yield = pdFALSE;
    xTaskNotifyFromISR(radio_task_handle, RADIO_EVENT_INTERRUPT, eSetBits, &yield);
    if (yield)
    {
        portYIELD_FROM_ISR();
    }
}

static uint64_t promiscuous_get_timestamp(void)
{
    uint32_t high1, high2;
    uint16_t low;

    do
    {
        high1 = TIM5.CNT;
        low   = TIM4.CNT;
        high2 = TIM5.CNT;
    } while (high1 != high2);

    return (((uint64_t)high1) << 16) | low;
}

static void radio_task(void *UNUSED(param))
{
    uint32_t pending_events = 0;
    for (;;)
    {
        // Wait to be instructed to start doing work.
        while (!(pending_events & RADIO_EVENT_START))
        {
            uint32_t new_events;
            xTaskNotifyWait(0, UINT32_MAX, &new_events, portMAX_DELAY);
            pending_events |= new_events;
        }
        pending_events &= ~RADIO_EVENT_START;

        // Initialize the radio.
        mrf_init();
        vTaskDelay(1U);
        mrf_release_reset();
        vTaskDelay(1U);
        mrf_common_init();
        while (mrf_get_interrupt())
            ;

        // Enable external interrupt on MRF INT rising edge.
        mrf_enable_interrupt(&mrf_int_isr);

        // Notify on_enter that initialization is finished.
        xSemaphoreGive(enabled_mode_change_sem);

        // Run the main operation.
        bool packet_dropped = false;
        while (!(pending_events & RADIO_EVENT_STOP))
        {
            // Process interrupts until the interrupt pin goes low.
            while (mrf_get_interrupt())
            {
                // Check outstanding interrupts.
                uint8_t intstat = mrf_read_short(MRF_REG_SHORT_INTSTAT);
                if (intstat & (1U << 3U))
                {
                    // RXIF = 1; packet received.
                    // Take a timestamp before doing anything else.
                    uint64_t stamp = promiscuous_get_timestamp();
                    mrf_write_short(MRF_REG_SHORT_BBREG1,
                                    0x04U);  // RXDECINV = 1; invert receiver symbol sign
                                             // to prevent further packet reception.
                    uint8_t rxfifo_frame_length = mrf_read_long(
                        MRF_REG_LONG_RXFIFO);  // Need to read this here even if no packet
                                               // buffer because this read also re-enables
                                               // the receiver.

                    // Sanitize the frame length to avoid buffer overruns.
                    rxfifo_frame_length = MIN(rxfifo_frame_length,
                                              128U /* RX FIFO size */ - 1U /* Length */ -
                                                  1U /* LQI */ - 1U /* RSSI */);
                    // Commented out because -Werror is on
                    // #warning proper packet filtering when radio filters do not match
                    // capture flags exactly

                    // Allocate a packet.
                    packet_t *packet;
                    if (xQueueReceive(free_queue, &packet, 0U))
                    {
                        uint8_t *wptr  = packet->data;
                        *wptr++        = packet_dropped ? 0x01U : 0x00U;
                        packet_dropped = false;
                        *wptr++        = radio_config.channel;
                        for (unsigned int i = 0; i != 6; ++i)
                        {
                            *wptr++ = (uint8_t)(stamp & 0xFF);
                            stamp >>= 8;
                        }
                        {
                            uint8_t *base = wptr;
                            for (unsigned int tries = 0; tries != 4; ++tries)
                            {
                                wptr         = base;
                                uint16_t crc = 0;
                                for (size_t i = 0U; i < rxfifo_frame_length; ++i)
                                {
                                    uint8_t byte = mrf_read_long(
                                        MRF_REG_LONG_RXFIFO + 1U /* Frame length */ + i);
                                    *wptr++ = byte;
                                    crc     = crc_update(crc, byte);
                                }
                                for (size_t i = 0; i < 2U /* LQI + RSSI */; ++i)
                                {
                                    *wptr++ = mrf_read_long(MRF_REG_LONG_RXFIFO +
                                                            1U /* Frame length */ +
                                                            rxfifo_frame_length + i);
                                }
                                if (crc == 0)
                                {
                                    break;
                                }
                            }
                        }
                        packet->length = wptr - packet->data;
                        xQueueSend(receive_queue, &packet, portMAX_DELAY);
                    }
                    else
                    {
                        packet_dropped = true;
                    }
                    mrf_write_short(MRF_REG_SHORT_BBREG1,
                                    0x00U);  // RXDECINV = 0; stop inverting receiver and
                                             // allow further reception

                    // Blink receive LED.
                    led_blink(LED_RX);
                }
            }
            pending_events &= ~RADIO_EVENT_INTERRUPT;

            // Wait for an event to occur.
            uint32_t new_events;
            xTaskNotifyWait(0, UINT32_MAX, &new_events, portMAX_DELAY);
            pending_events |= new_events;
        }
        pending_events &= ~RADIO_EVENT_STOP;

        // Disable the external interrupt on MRF INT.
        mrf_disable_interrupt();

        // Reset the radio.
        mrf_deinit();

        // Done.
        xSemaphoreGive(enabled_mode_change_sem);
    }
}

static void usb_task(void *UNUSED(param))
{
    for (;;)
    {
        // Wait to be instructed to start doing work.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Run.
        for (;;)
        {
            // Get a received packet.
            packet_t *packet;
            xQueueReceive(receive_queue, &packet, portMAX_DELAY);
            if (packet)
            {
                // Send the packet over USB.
                bool ok        = uep_write(0x81U, packet->data, packet->length, true);
                int error_code = errno;
                xQueueSend(free_queue, &packet, portMAX_DELAY);
                if (!ok && error_code == ECONNRESET)
                {
                    // Shutdown signal.
                    break;
                }
                // In case of EPIPE (endpoint halt status), drop packets on the floor
                // until the host changes its mind.
            }
            else
            {
                // Pushing a null pointer into the queue signals shutdown.
                break;
            }
        }

        xSemaphoreGive(enabled_mode_change_sem);
    }
}

void promiscuous_init(void)
{
    // Create IPC objects.
    static StaticQueue_t free_queue_storage, receive_queue_storage;
    static uint8_t free_queue_buffer[NUM_PACKETS * sizeof(packet_t *)],
        receive_queue_buffer[(NUM_PACKETS + 1) * sizeof(packet_t *)];
    free_queue = xQueueCreateStatic(NUM_PACKETS, sizeof(packet_t *), free_queue_buffer,
                                    &free_queue_storage);
    receive_queue =
        xQueueCreateStatic(NUM_PACKETS + 1U /* Signalling NULL */, sizeof(packet_t *),
                           receive_queue_buffer, &receive_queue_storage);

    // Allocate packet buffers.
    static packet_t packets[NUM_PACKETS];
    for (unsigned int i = 0U; i < NUM_PACKETS; ++i)
    {
        packet_t *packet = &packets[i];
        xQueueSend(free_queue, &packet, portMAX_DELAY);
    }

    // Create tasks.
    static StaticTask_t radio_task_storage, usb_task_storage;
    STACK_ALLOCATE(radio_task_stack, 4096);
    STACK_ALLOCATE(usb_task_stack, 4096);
    radio_task_handle = xTaskCreateStatic(
        &radio_task, "prom_radio", sizeof(radio_task_stack) / sizeof(*radio_task_stack),
        0, 6, radio_task_stack, &radio_task_storage);
    usb_task_handle = xTaskCreateStatic(&usb_task, "prom_usb",
                                        sizeof(usb_task_stack) / sizeof(*usb_task_stack),
                                        0, 5, usb_task_stack, &usb_task_storage);
}

void promiscuous_on_enter(void)
{
    // Enable timer 4 as a 16-bit upcounter with update events causing trigger
    // outputs, which will serve as the low ⅓ of a 48-bit timestamp counter,
    // and timer 5 as a 32-bit upcounter clocked by its trigger, which will
    // serve as the high ⅔ of the 48-bit counter.
    rcc_enable_reset(APB1, TIM4);
    rcc_enable_reset(APB1, TIM5);
    {
        TIM2_5_CR1_t cr1      = {.URS = 1};
        TIM2_5_EGR_t egr      = {.UG = 1};
        TIM2_5_CR2_t t4_cr2   = {.MMS = 2};
        TIM2_5_SMCR_t t5_smcr = {.TS = 2, .SMS = 7};

        TIM4.CR1 = cr1;
        TIM4.CR2 = t4_cr2;
        TIM4.CNT = 0;
        TIM4.PSC = 71;
        TIM4.ARR = 0xFFFF;
        TIM4.EGR = egr;

        TIM5.CR1  = cr1;
        TIM5.SMCR = t5_smcr;
        TIM5.CNT  = 0;
        TIM5.PSC  = 0;
        TIM5.ARR  = 0xFFFFFFFF;
        TIM5.EGR  = egr;

        cr1.CEN  = 1;
        TIM5.CR1 = cr1;
        TIM4.CR1 = cr1;
    }

    // Clear flags.
    promisc_flags = 0U;

    // Tell the tasks to start doing work.
    xTaskNotifyGive(radio_task_handle);
    xTaskNotifyGive(usb_task_handle);

    // Wait until the task has finished initializing the radio.
    // We must do this because we need to prevent SET PROMISCUOUS FLAGS from arriving and
    // poking things during initialization. During initialization, this could be avoided
    // by the task taking the bus mutex. However, that would leave a race where SET
    // PROMISCUOUS FLAGS arrives before the task gets around to taking said mutex. This
    // would be safe as far as mutual exclusion on the bus is concerned. However, it would
    // have a different problem: the promiscuous flags would be destroyed by the
    // initialization routine.
    //
    // One might think this could be avoided by taking the mutex here and passing
    // ownership of it into the task. However, FreeRTOS semaphores can be given and taken
    // by different tasks, but mutexes cannot. A mutex must always be given by the same
    // task that takes it. So, we can’t do that. Instead, we implement this
    // initialization-waiter mechanism.
    xSemaphoreTake(enabled_mode_change_sem, portMAX_DELAY);
}

void promiscuous_on_exit(void)
{
    // Shut down tasks.
    xTaskNotify(radio_task_handle, RADIO_EVENT_STOP, eSetBits);
    static packet_t *const null_packet = 0;
    xQueueSend(receive_queue, &null_packet, portMAX_DELAY);
    xSemaphoreTake(enabled_mode_change_sem, portMAX_DELAY);
    xSemaphoreTake(enabled_mode_change_sem, portMAX_DELAY);

    // Flush receive queue.
    packet_t *packet;
    while (xQueueReceive(receive_queue, &packet, 0))
    {
        xQueueSend(free_queue, &packet, 0);
    }

    // Turn off receive LED.
    led_off(LED_RX);

    // Stop the timestamp counter.
    rcc_disable(APB1, TIM4);
    rcc_disable(APB1, TIM5);
}

bool promiscuous_control_handler(const usb_setup_packet_t *pkt)
{
    if (pkt->bmRequestType.direction &&
        pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
        pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
        pkt->bRequest == CONTROL_REQUEST_GET_PROMISCUOUS_FLAGS && !pkt->wValue)
    {
        uep0_data_write(&promisc_flags, sizeof(promisc_flags));
        return true;
    }
    else if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
             pkt->bmRequestType.type == USB_CTYPE_VENDOR &&
             pkt->bRequest == CONTROL_REQUEST_SET_PROMISCUOUS_FLAGS && !pkt->wLength)
    {
        // This request must have value using only bits 0 through 7.
        if (pkt->wValue & 0xFF00U)
        {
            return false;
        }

        // Check if the application actually wants *ANY* packets.
        if (pkt->wValue & 0xF0U)
        {
            // Sanity check: if flag 0 (acknowledge) is set and one of bits 4 through 7
            // (accept some type of frame) is set…
            if ((pkt->wValue & 0x0001U) && (pkt->wValue & 0xF0U))
            {
                // … either exactly one or all three of bits 4 through 6 must be set to 1…
                if ((pkt->wValue & 0x70U) != 0x10U && (pkt->wValue & 0x70U) != 0x20U &&
                    (pkt->wValue & 0x70U) != 0x40U && (pkt->wValue & 0x70U) != 0x70U)
                {
                    return false;
                }
                // … either none or all of bits 1, 2, and 7 must be set to 1…
                if ((pkt->wValue & 0x86U) != 0x00U && (pkt->wValue & 0x86U) != 0x86U)
                {
                    return false;
                }
                // … and if bit 7 is set to 1, bits 4 through 6 must also all be set to 1
                if ((pkt->wValue & 0x80U) && (pkt->wValue & 0x70U) != 0x70U)
                {
                    return false;
                }
            }
            // This set of flags is acceptable; save.
            promisc_flags = pkt->wValue;
            // Disable all packet reception.
            mrf_write_short(MRF_REG_SHORT_BBREG1, 0x04U);
            // Install the new flags.
            mrf_write_short(MRF_REG_SHORT_RXMCR,
                            ((pkt->wValue & (1U << 0U)) ? 0U : (1U << 5U)) |
                                ((pkt->wValue & (1U << 3U)) ? (1U << 1U) : 0U) |
                                ((pkt->wValue & 0x86U) ? (1U << 0U) : 0U));
            if ((pkt->wValue & 0xF0U) == 0x10U)
            {
                mrf_write_short(MRF_REG_SHORT_RXFLUSH, 0x64U);
            }
            else if ((pkt->wValue & 0xF0U) == 0x20U)
            {
                mrf_write_short(MRF_REG_SHORT_RXFLUSH, 0x68U);
            }
            else if ((pkt->wValue & 0xF0U) == 0x40U)
            {
                mrf_write_short(MRF_REG_SHORT_RXFLUSH, 0x62U);
            }
            else
            {
                mrf_write_short(MRF_REG_SHORT_RXFLUSH, 0x60U);
            }
            // Set analogue path appropriately based on whether ACKs are being generated
            // and whether any packets are desired.
            if (pkt->wValue & 0x01U)
            {
                mrf_analogue_txrx();
            }
            else
            {
                mrf_analogue_rx();
            }
            // Re-enable packet reception.
            mrf_write_short(MRF_REG_SHORT_BBREG1, 0x00U);
            // Enable interrupt on receive.
            mrf_write_short(MRF_REG_SHORT_INTCON, 0xF7U);
            // Turn on receive LED to indicate capture is enabled.
            led_on(LED_RX);
        }
        else
        {
            // Shut down the radio.
            mrf_write_short(MRF_REG_SHORT_RXMCR, 0x20U);
            mrf_write_short(MRF_REG_SHORT_BBREG1, 0x04U);
            mrf_write_short(MRF_REG_SHORT_INTCON, 0xFFU);
            mrf_analogue_off();
            // Turn off receive LED to indicate capture is disabled.
            led_off(LED_RX);
        }

        return true;
    }

    return false;
}
