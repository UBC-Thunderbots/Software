/**
 * \defgroup CDCACM Communications device class abstract control model functions
 *
 * \brief These functions allow sending data over an emulated serial port.
 *
 * CDC ACM functionality exists within a USB configuration.
 * Within that configuration, the functionality uses two USB interfaces (grouped under an
 * interface association descriptor) and three endpoints (two IN and one OUT). The
 * application must include an instance of structure \ref cdcacm_descriptors_t in its
 * configuration descriptor. This structure should be initialized by setting its value to
 * the result of a call to \ref CDCACM_DESCRIPTORS_INIT. The device descriptor’s class,
 * subclass, and protocol numbers must be set to indicate the use of interface association
 * descriptors.
 *
 * At runtime, an application begins by calling \ref cdcacm_init at startup, and must do
 * so only once. Thereafter, the application must call \ref cdcacm_start when entering the
 * relevant configuration, and \ref cdcacm_stop when exiting it, from the USB stack
 * internal task. The application may call \ref cdcacm_write at any time once \ref
 * cdcacm_init has returned, from any task, regardless of whether CDC ACM is active or
 * whether any other task is in any of the functions.
 *
 * @{
 */

#include <cdcacm.h>
#include <errno.h>
#include <minmax.h>
#include <semphr.h>
#include <stack.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <task.h>
#include <unused.h>

#include "firmware/boards/shared/legacy_freertos/include/FreeRTOS.h"

/**
 * \cond INTERNAL
 */

/**
 * \brief The size of the data buffer.
 */
#define CDCACM_BUFFER_SIZE 4096U

/**
 * \brief The possible task notification bits understood by the CDC ACM task.
 */
enum cdcacm_event_t
{
    /**
     * \brief The task should start doing work.
     */
    CDCACM_EVENT_START = 0x01,

    /**
     * \brief The task should stop doing work.
     */
    CDCACM_EVENT_STOP = 0x02,

    /**
     * \brief New data is available in the buffer.
     */
    CDCACM_EVENT_NEW_DATA = 0x04,
};

/**
 * \brief The buffer that holds data before it travels over USB.
 */
static uint8_t cdcacm_buffer[CDCACM_BUFFER_SIZE];

/**
 * \brief The index in the buffer of the next byte that the task will send over USB.
 */
static size_t cdcacm_rptr = 0;

/**
 * \brief The index in the buffer of the next byte that a caller will write into.
 */
static size_t cdcacm_wptr = 0;

/**
 * \brief A mutex protecting the module from multiple simultaneous writers.
 */
static SemaphoreHandle_t cdcacm_writer_mutex;

/**
 * \brief A semaphore used to notify that the task has quiesced.
 */
static SemaphoreHandle_t cdcacm_shutdown_sem;

/**
 * \brief The handle of the task operating on the USB endpoint.
 */
static TaskHandle_t cdcacm_task_handle;

_Static_assert(
    INCLUDE_vTaskSuspend == 1,
    "vTaskSuspend must be included, because otherwise mutex taking can time out!");

/**
 * \endcond
 */



/**
 * \cond INTERNAL
 * \brief The CDC ACM task.
 */
static void cdcacm_task(void *param)
{
    unsigned int endpoint   = (unsigned int)param;
    uint32_t pending_events = 0;
    for (;;)
    {
        // Wait to be instructed to start doing work.
        while (!(pending_events & CDCACM_EVENT_START))
        {
            uint32_t new_events;
            xTaskNotifyWait(0, UINT32_MAX, &new_events, portMAX_DELAY);
            pending_events |= new_events;
        }
        pending_events &= ~CDCACM_EVENT_START;

        // Run until stopped.
        while (!(pending_events & CDCACM_EVENT_STOP))
        {
            // If we have data to send, send it.
            if (pending_events & CDCACM_EVENT_NEW_DATA)
            {
                // Keep going until we have no more data.
                for (;;)
                {
                    // Copy the read and write pointers to locals.
                    size_t rptr =
                        cdcacm_rptr;  // Non-atomic because only modified by cdcacm_task.
                    size_t wptr = __atomic_load_n(
                        &cdcacm_wptr,
                        __ATOMIC_RELAXED);  // Atomic because modified by cdcacm_write.
                    if (wptr == rptr)
                    {
                        break;
                    }
                    __atomic_signal_fence(__ATOMIC_ACQUIRE);

                    // Send as much as we can in one contiguous block.
                    size_t to_send;
                    if (rptr < wptr)
                    {
                        to_send = wptr - rptr;
                    }
                    else
                    {
                        to_send = CDCACM_BUFFER_SIZE - rptr;
                    }
                    if (uep_write(endpoint, &cdcacm_buffer[rptr], to_send, true))
                    {
                        // Data sent successfully; consume from buffer.
                        rptr += to_send;
                        if (rptr == CDCACM_BUFFER_SIZE)
                        {
                            rptr = 0U;
                        }
                    }
                    else
                    {
                        // Data transmission failed; check why.
                        if (errno == EPIPE)
                        {
                            // Endpoint halted; wait for halt status to end.
                            if (uep_halt_wait(endpoint))
                            {
                                // Endpoint no longer halted; discard any data written
                                // while halted.
                                __atomic_signal_fence(__ATOMIC_RELEASE);
                                __atomic_store_n(
                                    &cdcacm_rptr,
                                    __atomic_load_n(&cdcacm_wptr, __ATOMIC_RELAXED),
                                    __ATOMIC_RELAXED);
                            }
                            else
                            {
                                // Endpoint deactivated while waiting for halt status;
                                // fall out and observe shutdown event on next outer-loop
                                // iteration.
                                break;
                            }
                        }
                        else if (errno == ECONNRESET)
                        {
                            // Endpoint deactivated while writing; fall out and observe
                            // shutdown event on next outer-loop iteration.
                            break;
                        }
                        else
                        {
                            // Unknown reason.
                            abort();
                        }
                    }
                    __atomic_store_n(&cdcacm_rptr, rptr, __ATOMIC_RELAXED);
                }
            }
            pending_events &= ~CDCACM_EVENT_NEW_DATA;

            // Wait for something to happen.
            uint32_t new_events;
            xTaskNotifyWait(0, UINT32_MAX, &new_events, portMAX_DELAY);
            pending_events |= new_events;
        }
        pending_events &= ~CDCACM_EVENT_STOP;

        // Notify the other task.
        xSemaphoreGive(cdcacm_shutdown_sem);
    }
}
/**
 * \endcond
 */

/**
 * \brief Initializes the CDC ACM module.
 *
 * This function must be called and must return before any other CDC ACM functions are
 * called.
 *
 * \param[in] in_data_ep_num the endpoint number of the IN data endpoint
 *
 * \param[in] task_priority the priority of the CDC ACM task
 */
void cdcacm_init(unsigned int in_data_ep_num, unsigned int task_priority)
{
    static StaticSemaphore_t writer_mutex_storage, shutdown_sem_storage;
    cdcacm_writer_mutex = xSemaphoreCreateMutexStatic(&writer_mutex_storage);
    cdcacm_shutdown_sem = xSemaphoreCreateBinaryStatic(&shutdown_sem_storage);
    static StaticTask_t cdcacm_task_tcb;
    STACK_ALLOCATE(cdcacm_task_stack, 4096);
    cdcacm_task_handle = xTaskCreateStatic(
        &cdcacm_task, "cdcacm", sizeof(cdcacm_task_stack) / sizeof(*cdcacm_task_stack),
        (void *)(in_data_ep_num | 0x80), task_priority, cdcacm_task_stack,
        &cdcacm_task_tcb);
}

/**
 * \brief Activates the CDC ACM module.
 *
 * This function must be called on the USB stack internal task when the containing
 * configuration is entered. A suitable way to do this might be to use this function as
 * the \c on_enter callback of a \c udev_alternate_setting_info_t structure.
 */
void cdcacm_start(void)
{
    xTaskNotify(cdcacm_task_handle, CDCACM_EVENT_START, eSetBits);
}

/**
 * \brief Deactivates the CDC ACM module.
 *
 * This function must be called on the USB stack internal task when the containing
 * configuration is exited. A suitable way to do this might be to use this function as the
 * \c on_exit callback of a \c udev_alternate_setting_info_t structure.
 */
void cdcacm_stop(void)
{
    xTaskNotify(cdcacm_task_handle, CDCACM_EVENT_STOP, eSetBits);
    xSemaphoreTake(cdcacm_shutdown_sem, portMAX_DELAY);
}

/**
 * \brief Writes data to the serial port.
 *
 * This function may be called whether or not the CDC ACM is currently enabled.
 * Data is queued in a buffer and transmitted in the CDC ACM task, so this function
 * returns quickly. Data is lost if the buffer becomes full (due to the host not running
 * any transactions), the endpoint is halted, or the CDC ACM is disabled.
 *
 * \param[in] data the data to write
 *
 * \param[in] length the number of bytes to write
 */
void cdcacm_write(const void *data, size_t length)
{
    xSemaphoreTake(cdcacm_writer_mutex, portMAX_DELAY);

    const uint8_t *dptr = data;

    for (;;)
    {
        // If there is no more data to read, then we are done.
        if (!length)
        {
            break;
        }

        // Copy the read and write pointers to locals.
        size_t rptr = __atomic_load_n(
            &cdcacm_rptr, __ATOMIC_RELAXED);  // Atomic because modified by cdcacm_task.
        size_t wptr = cdcacm_wptr;  // Non-atomic because only modified by cdcacm_write,
                                    // against which we hold a mutex.
        __atomic_signal_fence(__ATOMIC_ACQUIRE);

        // If the buffer is full, then we are done.
        if (wptr + 1U == rptr || (wptr + 1U == CDCACM_BUFFER_SIZE && !rptr))
        {
            break;
        }

        // Copy some data into the buffer.
        size_t space;
        if (wptr < rptr)
        {
            space = rptr - wptr - 1U;
        }
        else
        {
            space = CDCACM_BUFFER_SIZE - wptr;
        }
        size_t to_copy = MIN(space, length);
        memcpy(&cdcacm_buffer[wptr], dptr, to_copy);

        // Advance.
        dptr += to_copy;
        length -= to_copy;
        wptr += to_copy;
        if (wptr == CDCACM_BUFFER_SIZE)
        {
            wptr = 0U;
        }
        __atomic_signal_fence(__ATOMIC_RELEASE);
        __atomic_store_n(&cdcacm_wptr, wptr, __ATOMIC_RELAXED);
    }

    // Notify the task.
    xTaskNotify(cdcacm_task_handle, CDCACM_EVENT_NEW_DATA, eSetBits);

    xSemaphoreGive(cdcacm_writer_mutex);
}

/**
 * @}
 */
