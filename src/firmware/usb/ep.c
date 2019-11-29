/**
 * \defgroup UEP Nonzero endpoint handling
 *
 * These functions allow the application to communicate over a nonzero
 * endpoint. None of the function in this section may be called from the USB
 * stack internal task unless otherwise indicated.
 *
 * \{
 */
#include <FreeRTOS.h>
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <minmax.h>
#include <registers/otg_fs.h>
#include <stdlib.h>
#include <task.h>
#include <usb.h>

#include "internal.h"

/**
 * \cond INTERNAL
 */
/**
 * \brief The configuration and status data for the endpoints.
 */
uep_ep_t uep_eps[UEP_MAX_ENDPOINT * 2U];

/**
 * \brief The maximum number of packets in an OUT physical transfer.
 *
 * This value is determined by the width of the \c PKTCNT field in the \c TSIZ register.
 */
#define OUT_PXFR_MAX_PACKETS 0x3FFU

/**
 * \brief The maximum number of bytes in an OUT physical transfer.
 *
 * This value is determined by the width of the \c XFRSIZ field in the \c TSIZ register.
 */
#define OUT_PXFR_MAX_BYTES 0x7FFFFU

/**
 * \brief The maximum number of packets in an IN physical transfer.
 *
 * This value is determined by the width of the \c PKTCNT field in the \c TSIZ register.
 */
#define IN_PXFR_MAX_PACKETS 0x3FFU

/**
 * \brief The maximum number of bytes in an IN physical transfer.
 *
 * This value should be determined by the width of the \c XFRSIZ field in the
 * \c TSIZ register. However, what appears to be a hardware bug means that
 * transfers occasionally execute at the wrong size if bit 11 or above of the
 * \c XFRSIZ field is nonzero!
 */
#define IN_PXFR_MAX_BYTES 0x7FFFFU
/**
 * \endcond
 */



/**
 * \cond INTERNAL
 */
/**
 * \brief Queues an event on an endpoint’s event queue.
 *
 * \param[in] ep the endpoint address
 * \param[in] event the event to queue
 * \pre This function must be called only from task code.
 */
void uep_queue_event(unsigned int ep, uint8_t event)
{
    uep_ep_t *ctx  = &uep_eps[UEP_IDX(ep)];
    BaseType_t ret = xQueueSend(ctx->event_queue, &event, 0U);
    assert(ret == pdTRUE);
    if (ctx->async_cb)
    {
        ctx->async_cb(ep, 0);
    }
}

/**
 * \brief Queues an event on an endpoint’s event queue.
 *
 * \param[in] ep the endpoint address
 * \param[in] event the event to queue
 * \param[out] yield whether the operation woke up a task that requires a
 * context switch on return
 * \pre This function must be called only from the USB interrupt service
 * routine.
 */
void uep_queue_event_from_isr(unsigned int ep, uint8_t event, BaseType_t *yield)
{
    uep_ep_t *ctx  = &uep_eps[UEP_IDX(ep)];
    BaseType_t ret = xQueueSendFromISR(ctx->event_queue, &event, yield);
    assert(ret == pdTRUE);
    if (ctx->async_cb)
    {
        ctx->async_cb(ep, yield);
    }
}

/**
 * \brief Disables a running endpoint.
 *
 * \param[in] ep the endpoint address
 * \pre The caller must hold the endpoint mutex.
 * \pre The endpoint must be in UEP_STATE_RUNNING.
 * \post The endpoint is in UEP_STATE_IDLE.
 * \post The endpoint mutex is still held.
 */
static void uep_disable(unsigned int ep)
{
    uep_ep_t *ctx = &uep_eps[UEP_IDX(ep)];

    // Sanity check.
    assert(ctx->state == UEP_STATE_RUNNING);

    if (UEP_DIR(ep))
    {
        // Disabling an IN endpoint.
        taskENTER_CRITICAL();
        // Prevent the ISR from pushing more data into this endpoint.
        OTG_FS.DIEPEMPMSK &= ~(1U << UEP_NUM(ep));
        // Kick off the disable operation. Whether the transfer completes or
        // not, the SNAK=1 will cause the hardware to set NAKSTS=1 and
        // INEPNE=1, the latter of which will cause an interrupt; the ISR will
        // observe INEPNE=1, clear it, and, if EPENA=1, set EPDIS=1.
        {
            OTG_FS_DIEPCTLx_t diepctl             = OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL;
            diepctl.EPENA                         = 0;
            diepctl.EPDIS                         = 0;
            diepctl.SNAK                          = 1;
            OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL = diepctl;
        }
        taskEXIT_CRITICAL();
        // Wait for the endpoint to be disabled.
        xSemaphoreTake(ctx->disabled_sem, portMAX_DELAY);
        // Flush the transmit FIFO.
        udev_flush_tx_fifo(ep);
    }
    else
    {
        // Disabling an OUT endpoint.
        xSemaphoreTake(udev_gonak_mutex, portMAX_DELAY);
        // Tell the ISR that it should disable this endpoint when GONAKEFF
        // occurs.
        udev_gonak_disable_ep = ep;
        // Kick off the process by achieving GONAK.
        taskENTER_CRITICAL();
        __atomic_signal_fence(__ATOMIC_RELEASE);
        OTG_FS.DCTL.SGONAK = 1;
        taskEXIT_CRITICAL();
        // Wait for the endpoint to be disabled.
        xSemaphoreTake(ctx->disabled_sem, portMAX_DELAY);
        xSemaphoreGive(udev_gonak_mutex);
    }

    // By the time we get here, we know that not only is there no transfer
    // running and EPENA=0 but also that an event has been queued which will
    // wake up the task previously running a transfer. The event might be XFRC
    // or might be EPDISD, but which one it is doesn’t matter.

    // Transfer has now stopped.
    ctx->state = UEP_STATE_IDLE;
}



/**
 * \brief Starts the next physical transfer on an endpoint.
 *
 * \param[in] ep the endpoint address
 * \retval true the transfer was started
 * \retval false the transfer was not started due to the endpoint
 * state, and \c errno has been set appropriately
 * \pre The caller must hold the endpoint mutex.
 * \pre The endpoint must not be in UEP_STATE_HALTED_WAITING.
 * \pre The endpoint must be disabled (\c EPENA=0).
 * \pre The transfer metadata must be initialized.
 * \post The endpoint mutex is still held.
 */
static bool uep_start_pxfr(unsigned int ep)
{
    uep_ep_t *ctx = &uep_eps[UEP_IDX(ep)];

    bool ret;
    switch (ctx->state)
    {
        case UEP_STATE_INACTIVE:
            // Endpoint asynchronously deactivated. Report the situation to the
            // application.
            errno = ECONNRESET;
            ret   = false;
            break;

        case UEP_STATE_HALTED:
        case UEP_STATE_CLEAR_HALT_PENDING:
            // Endpoint asynchronously halted. Report the situation to the
            // application.
            errno = EPIPE;
            ret   = false;
            break;

        case UEP_STATE_RUNNING:
            // Start the next physical transfer.
            if (UEP_DIR(ep))
            {
                // IN transfer.
                size_t max_packet = OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL.MPSIZ;
                size_t pxfr_bytes = ctx->bytes_left;
                pxfr_bytes = MIN(pxfr_bytes, IN_PXFR_MAX_BYTES / max_packet * max_packet);
                pxfr_bytes = MIN(pxfr_bytes, IN_PXFR_MAX_PACKETS * max_packet);
                OTG_FS_DIEPTSIZx_t tsiz;
                if (pxfr_bytes)
                {
                    tsiz.PKTCNT = (pxfr_bytes + max_packet - 1U) / max_packet,
                    tsiz.XFRSIZ = pxfr_bytes;
                }
                else
                {
                    tsiz.PKTCNT    = 1U;
                    tsiz.XFRSIZ    = 0U;
                    ctx->flags.zlp = false;
                }
                OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPTSIZ = tsiz;
                OTG_FS_DIEPCTLx_t ctl = OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL;
                ctl.EPENA             = 1;
                ctl.EPDIS             = 0;
                ctl.CNAK              = 1;
                __atomic_signal_fence(
                    __ATOMIC_ACQ_REL);  // Prevent writes to ctx from being sunk below,
                                        // nor the write to DIEPCTL from being hoisted
                                        // above, this point.
                OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL = ctl;
                if (pxfr_bytes)
                {
                    taskENTER_CRITICAL();
                    OTG_FS.DIEPEMPMSK |= 1U << UEP_NUM(ep);
                    taskEXIT_CRITICAL();
                }
            }
            else
            {
                // OUT transfer.
                size_t max_packet   = OTG_FS.DOEP[UEP_NUM(ep) - 1U].DOEPCTL.MPSIZ;
                size_t pxfr_packets = (ctx->bytes_left + max_packet - 1U) / max_packet;
                pxfr_packets        = MIN(pxfr_packets, OUT_PXFR_MAX_PACKETS);
                pxfr_packets        = MIN(pxfr_packets, OUT_PXFR_MAX_BYTES / max_packet);
                OTG_FS_DOEPTSIZx_t tsiz = {
                    .PKTCNT = pxfr_packets,
                    .XFRSIZ = pxfr_packets * max_packet,
                };
                OTG_FS.DOEP[UEP_NUM(ep) - 1U].DOEPTSIZ = tsiz;
                OTG_FS_DOEPCTLx_t ctl = OTG_FS.DOEP[UEP_NUM(ep) - 1U].DOEPCTL;
                ctl.EPENA             = 1;
                ctl.EPDIS             = 0;
                ctl.CNAK              = 1;
                __atomic_signal_fence(
                    __ATOMIC_ACQ_REL);  // Prevent writes to ctx from being sunk below,
                                        // nor the write to DOEPCTL from being hoisted
                                        // above, this point.
                OTG_FS.DOEP[UEP_NUM(ep) - 1U].DOEPCTL = ctl;
            }
            ret = true;
            break;

        default:
            // If UEP_STATE_IDLE, logic error: higher-level start-transfer
            // function should have set UEP_STATE_RUNNING. If
            // UEP_STATE_HALTED_WAITING, logic error: application tried to
            // start two operations on an endpoint at the same time.
            abort();
    }

    return ret;
}

/**
 * \brief Synchronously runs a logical transfer.
 *
 * \param[in] ep the endpoint address
 * \retval true the transfer finished normally
 * \retval false the transfer was not started or was aborted due to the
 * endpoint state, and \c errno has been set appropriately
 * \pre The endpoint must not be in UEP_STATE_RUNNING or
 * UEP_STATE_HALTED_WAITING.
 * \pre The transfer parameters must be initialized.
 */
static bool uep_transfer(unsigned int ep)
{
    uep_ep_t *ctx = &uep_eps[UEP_IDX(ep)];

    // Grab max packet size.
    size_t max_packet;
    if (UEP_DIR(ep))
    {
        max_packet = OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL.MPSIZ;
    }
    else
    {
        max_packet = OTG_FS.DOEP[UEP_NUM(ep) - 1U].DOEPCTL.MPSIZ;
    }

    // Change from idle state, if there, to running.
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    assert(ctx->state != UEP_STATE_RUNNING && ctx->state != UEP_STATE_HALTED_WAITING);
    if (ctx->state == UEP_STATE_IDLE)
    {
        ctx->state = UEP_STATE_RUNNING;
    }
    xSemaphoreGive(ctx->mutex);

    bool done, ret = true;
    do
    {
        if (UEP_DIR(ep))
        {
            // Done when all bytes are sent and a ZLP, if necessary, has also
            // been sent.
            done = !ctx->bytes_left && !ctx->flags.zlp;
        }
        else
        {
            // Done when all bytes are received, a ZLP is received or a short
            // packet is received.
            done = !ctx->bytes_left || ctx->flags.zlp ||
                   ((ctx->transfer.out.wptr - ctx->transfer.out.buffer) % max_packet);
        }

        // If an overflow occurs, stop immediately.
        if (ctx->flags.overflow)
        {
            done  = true;
            ret   = false;
            errno = EOVERFLOW;
        }

        if (!done)
        {
            // Try to start a physical transfer.
            xSemaphoreTake(ctx->mutex, portMAX_DELAY);
            ret = uep_start_pxfr(ep);
            xSemaphoreGive(ctx->mutex);

            if (ret)
            {
                // Started physical transfer OK; wait for event.
                uint8_t event;
                xQueueReceive(ctx->event_queue, &event, portMAX_DELAY);
                switch (event)
                {
                    case UEP_EVENT_XFRC:
                        // Physical transfer done; loop around and start next.
                        break;

                    case UEP_EVENT_EPDISD:
                        // Endpoint asynchronously disabled; report.
                        ret  = false;
                        done = true;
                        xSemaphoreTake(ctx->mutex, portMAX_DELAY);
                        errno = ctx->state == UEP_STATE_INACTIVE ? ECONNRESET : EPIPE;
                        xSemaphoreGive(ctx->mutex);
                        break;

                    default:
                        // These should never happen during a transfer.
                        abort();
                }
            }
            else
            {
                // Starting physical transfer failed; return now.
                done = true;
            }
        }
    } while (!done);

    // Change state from running, if there, back to idle.
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    if (ctx->state == UEP_STATE_RUNNING)
    {
        ctx->state = UEP_STATE_IDLE;
    }
    xSemaphoreGive(ctx->mutex);

    return ret;
}
/**
 * \endcond
 */

/**
 * \brief Reads a block of data from an OUT endpoint.
 *
 * This function blocks until the requested amount of data is received, a short
 * transaction is received, or an error is detected.
 *
 * \param[in] ep the endpoint address to read from, from 0x01 to \ref
 * UEP_MAX_ENDPOINT
 * \param[out] buffer the buffer into which to store the received data
 * \param[in] max_length the maximum number of bytes to receive
 * \param[out] length the actual number of bytes received (which is valid and
 * may be nonzero even if an error occurs)
 * \retval true if the read completed successfully
 * \retval false if an error occurred before or during the read (despite which
 * some data may have been received)
 * \exception EPIPE the endpoint was halted, either when the function was first
 * called or while the transfer was occurring
 * \exception ECONNRESET the endpoint was inactive, either when the function
 * was first called or while the transfer was occurring
 * \exception EOVERFLOW \p max_length is not a multiple of the endpoint maximum
 * packet size and the last transaction did not fit in the buffer
 * \pre The endpoint must be enabled in the current configuration and in the
 * current alternate setting of any relevant interfaces.
 */
bool uep_read(unsigned int ep, void *buffer, size_t max_length, size_t *length)
{
    // Sanity check.
    assert(!UEP_DIR(ep));
    assert(1 <= UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);
    assert(buffer);
    assert(max_length);

    // Set up transfer parameters.
    uep_ep_t *ctx            = &uep_eps[UEP_IDX(ep)];
    ctx->flags.zlp           = false;
    ctx->flags.overflow      = false;
    ctx->transfer.out.buffer = buffer;
    ctx->transfer.out.wptr   = buffer;
    ctx->bytes_left          = max_length;
    ctx->async_cb            = 0;

    // Run the transfer.
    bool ret = uep_transfer(ep);

    // Determine number of bytes received.
    *length = ctx->transfer.out.wptr - ctx->transfer.out.buffer;

    // Clear transfer parameters.
    ctx->transfer.out.buffer = 0;
    ctx->transfer.out.wptr   = 0;
    ctx->bytes_left          = 0;
    ctx->async_cb            = 0;

    // Done.
    return ret;
}



/**
 * \brief Writes a block of data to an IN endpoint.
 *
 * This function blocks until the data has been delivered or an error is
 * detected.
 *
 * \param[in] ep the endpoint to write to, from 0x81 to <code>\ref
 * UEP_MAX_ENDPOINT | 0x80</code>
 * \param[in] data the data to send
 * \param[in] length the number of bytes to send
 * \param[in] zlp \c true to add a zero-length packet if \p length is a
 * multiple of the endpoint maximum packet size, or \c false to omit the
 * zero-length packet
 * \retval true if the write completed successfully
 * \retval false if an error occurred before or during the write (despite which
 * some data may have been sent)
 * \exception EPIPE the endpoint was halted, either when the function was first
 * called or while the transfer was occurring
 * \exception ECONNRESET the endpoint was deactivated, either when the function
 * was first called or while the transfer was occurring
 * \pre The endpoint must be enabled in the current configuration and in the
 * current alternate setting of any relevant interfaces.
 */
bool uep_write(unsigned int ep, const void *data, size_t length, bool zlp)
{
    // Sanity check.
    assert(UEP_DIR(ep));
    assert(1 <= UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);
    assert(!!data == !!length);
    assert(length || zlp);

    // Grab max packet size.
    size_t max_packet = OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL.MPSIZ;

    // Set up transfer parameters.
    uep_ep_t *ctx         = &uep_eps[UEP_IDX(ep)];
    ctx->flags.zlp        = zlp && !(length % max_packet);
    ctx->flags.overflow   = false;
    ctx->transfer.in.data = data;
    ctx->transfer.in.rptr = data;
    ctx->bytes_left       = length;
    ctx->async_cb         = 0;

    // Run transfer.
    bool ret = uep_transfer(ep);

    // Clear transfer parameters.
    ctx->transfer.in.data = 0;
    ctx->transfer.in.rptr = 0;
    ctx->bytes_left       = 0;
    ctx->async_cb         = 0;

    // Done.
    return ret;
}



/**
 * \brief Waits for halt status to clear on a halted endpoint.
 *
 * This function returns when the endpoint is not halted. If the endpoint is
 * not halted when called, this function returns immediately.
 *
 * \param[in] ep the endpoint on which to wait, from 0x01 to \ref
 * UEP_MAX_ENDPOINT or 0x81 to <code>\ref UEP_MAX_ENDPOINT | 0x80</code>
 * \retval true if endpoint halt status has been cleared and the endpoint is
 * now working normally
 * \retval false if an error occurred
 * \exception ECONNRESET the endpoint was deactivated, either when the function
 * was first called or while waiting
 * \pre The endpoint must be enabled in the current configuration and in the
 * current alternate setting of any relevant interfaces.
 */
bool uep_halt_wait(unsigned int ep)
{
    assert(UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);

    uep_ep_t *ctx = &uep_eps[UEP_IDX(ep)];

    // Decide whether to return immediately or wait.
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    bool ret, wait;
    switch (ctx->state)
    {
        case UEP_STATE_INACTIVE:
            // Endpoint is not halted; return immediately.
            errno = ECONNRESET;
            ret   = false;
            wait  = false;
            break;

        case UEP_STATE_IDLE:
            // Endpoint is not halted; return immediately.
            ret  = true;
            wait = false;
            break;

        case UEP_STATE_CLEAR_HALT_PENDING:
            // Prior halt status acknowledged; return immediately.
            ctx->state = UEP_STATE_IDLE;
            ret        = true;
            wait       = false;
            break;

        case UEP_STATE_HALTED:
            // Endpoint is halted; wait.
            ctx->state = UEP_STATE_HALTED_WAITING;
            wait       = true;
            break;

        default:
            // If UEP_STATE_RUNNING or UEP_STATE_HALTED_WAITING, logic error:
            // application tried to start two operations on an endpoint at the
            // same time.
            abort();
    }
    xSemaphoreGive(ctx->mutex);

    if (wait)
    {
        uint8_t event;
        xQueueReceive(ctx->event_queue, &event, portMAX_DELAY);
        switch (event)
        {
            case UEP_EVENT_DEACTIVATED_WHILE_HALTED:
                // Endpoint deactivated.
                errno = ECONNRESET;
                ret   = false;
                break;

            case UEP_EVENT_HALT_CLEARED:
                // Halt status cleared. Let the application go on and
                // operate the endpoint.
                ret = true;
                break;

            default:
                // If UEP_EVENT_XFRC or UEP_EVENT_EPDISD, logic error: no
                // transfer should have been in progress.
                abort();
        }
    }

    return ret;
}



/**
 * \cond INTERNAL
 */
/**
 * \brief Finishes an asynchronous read or write on an endpoint.
 *
 * This function returns immediately, whether or not the transfer was finished,
 * reporting the operation’s current status. This function does not compute the
 * number of bytes transferred nor clear the transfer parameters; the caller
 * must do that if this function returns anything other than \c false with \c
 * errno set to \c EINPROGRESS. This function also does not modify the endpoint
 * state variable.
 *
 * \param[in] ep the endpoint to check
 * \retval true if the transfer completed successfully
 * \retval false if an error occurred while the transfer was in progress
 * (despite which some data may have been transferred), if the write has not
 * yet finished
 * \exception EPIPE the endpoint was halted while the read was in progress
 * \exception ECONNRESET the endpoint was disabled while the read was in progress
 * \exception EINPROGRESS the operation has not yet finished
 * \pre A prior asynchronous read or write must have been started and not yet
 * reported completed on this endpoint.
 */
static bool uep_async_finish(unsigned int ep)
{
    // Grab max packet size if this is an OUT endpoint (not needed for IN
    // endpoints).
    size_t max_packet_out =
        UEP_DIR(ep) ? 0U : OTG_FS.DOEP[UEP_NUM(ep) - 1U].DOEPCTL.MPSIZ;

    // Check for event.
    uep_ep_t *ctx = &uep_eps[UEP_IDX(ep)];
    uint8_t event;
    bool ret, done;
    if (xQueueReceive(ctx->event_queue, &event, 0) == pdTRUE)
    {
        // Event received.
        switch (event)
        {
            case UEP_EVENT_XFRC:
                // Physical transfer done.
                if (UEP_DIR(ep))
                {
                    // Done when all bytes are sent and a ZLP, if necessary,
                    // has also been sent.
                    done = !ctx->bytes_left && !ctx->flags.zlp;
                }
                else
                {
                    // Done when all bytes are received, a ZLP is received or a
                    // short packet is received.
                    done = !ctx->bytes_left || ctx->flags.zlp ||
                           ((ctx->transfer.out.wptr - ctx->transfer.out.buffer) %
                            max_packet_out);
                }
                if (done)
                {
                    // Logical transfer is done.
                    ret = true;
                }
                else
                {
                    // Need another physical transfer to do the rest of the
                    // bytes.
                    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
                    ret = uep_start_pxfr(ep);
                    xSemaphoreGive(ctx->mutex);
                    if (ret)
                    {
                        // Next physical transfer started. Not a successful
                        // transfer-finish; transfer is still in progress.
                        errno = EINPROGRESS;
                        ret   = false;
                    }
                    else
                    {
                        // Transfer aborted; use return value and errno from
                        // uep_start_pxfr.
                    }
                }
                break;

            case UEP_EVENT_EPDISD:
                // Endpoint asynchronously disabled; report.
                ret = false;
                xSemaphoreTake(ctx->mutex, portMAX_DELAY);
                errno = ctx->state == UEP_STATE_INACTIVE ? ECONNRESET : EPIPE;
                xSemaphoreGive(ctx->mutex);
                break;

            default:
                // These should never happen during a transfer.
                abort();
        }
    }
    else
    {
        // No event; there is nothing to report.
        errno = EINPROGRESS;
        ret   = false;
    }

    return ret;
}
/**
 * \endcond
 */

/**
 * \brief Starts an asynchronous read on an endpoint.
 *
 * This function returns immediately, allowing the read to continue in the
 * background. The memory referred to by the \p buffer pointer \em must remain
 * valid until the asynchronous read operation completes (successfully or
 * otherwise)!
 *
 * \param[in] ep the endpoint to read from, from 0x01 to \ref UEP_MAX_ENDPOINT
 * \param[out] buffer the buffer into which to store the received data
 * \param[in] max_length the maximum number of bytes to receive
 * \param[in] cb a callback to invoke when \ref uep_async_read_finish must be
 * called (whose invocation does not necessarily imply the transfer is
 * complete)
 * \retval true if the read started successfully
 * \retval false if an error occurred before the read started (in which case no
 * data has been received)
 * \exception EPIPE the endpoint was halted at the time of call
 * \exception ECONNRESET the endpoint was disabled at the time of call
 * \pre The endpoint must be enabled in the current configuration and in the
 * current alternate setting of any relevant interfaces.
 */
bool uep_async_read_start(unsigned int ep, void *buffer, size_t max_length,
                          uep_async_cb_t cb)
{
    // Sanity check.
    assert(!UEP_DIR(ep));
    assert(1 <= UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);
    assert(buffer);
    assert(max_length);

    // Set up transfer parameters.
    uep_ep_t *ctx            = &uep_eps[UEP_IDX(ep)];
    ctx->flags.zlp           = false;
    ctx->flags.overflow      = false;
    ctx->transfer.out.buffer = buffer;
    ctx->transfer.out.wptr   = buffer;
    ctx->bytes_left          = max_length;
    ctx->async_cb            = cb;

    // Change endpoint state and start first physical transfer.
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    if (ctx->state == UEP_STATE_IDLE)
    {
        ctx->state = UEP_STATE_RUNNING;
    }
    bool ret = uep_start_pxfr(ep);

    // On failure, clear transfer parameters and reset endpoint state.
    if (!ret)
    {
        ctx->transfer.out.buffer = 0;
        ctx->transfer.out.wptr   = 0;
        ctx->bytes_left          = 0;
        ctx->async_cb            = 0;
        if (ctx->state == UEP_STATE_RUNNING)
        {
            ctx->state = UEP_STATE_IDLE;
        }
    }
    xSemaphoreGive(ctx->mutex);

    // Done.
    return ret;
}

/**
 * \brief Finishes an asynchronous read on an endpoint.
 *
 * This function returns immediately, whether or not the read was finished,
 * reporting the operation’s current status.
 *
 * \param[in] ep the endpoint being read from, from 0x01 to \ref
 * UEP_MAX_ENDPOINT
 * \param[out] length the number of bytes actually received
 * \retval true if the read completed successfully
 * \retval false if an error occurred while the read was in progress (despite
 * which some data may have been received) or if the read has not yet finished
 * \exception EPIPE the endpoint was halted while the read was in progress
 * \exception ECONNRESET the endpoint was disabled while the read was in
 * progress
 * \exception EOVERFLOW \p max_length was not a multiple of the endpoint
 * maximum packet size and the last transaction did not fit in the buffer
 * \exception EINPROGRESS the operation has not yet finished
 * \pre A prior asynchronous read must have been started and not yet reported
 * completed on this endpoint.
 */
bool uep_async_read_finish(unsigned int ep, size_t *length)
{
    // Sanity check.
    assert(!UEP_DIR(ep));
    assert(1 <= UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);

    // Try to finish the transfer.
    bool ret = uep_async_finish(ep);
    if (ret || (!ret && errno != EINPROGRESS))
    {
        // Transfer finished, successfully or otherwise; compute length, clear
        // transfer parameters and reset endpoint state.
        uep_ep_t *ctx            = &uep_eps[UEP_IDX(ep)];
        *length                  = ctx->transfer.out.wptr - ctx->transfer.out.buffer;
        ctx->transfer.out.buffer = 0;
        ctx->transfer.out.wptr   = 0;
        ctx->bytes_left          = 0;
        ctx->async_cb            = 0;
        xSemaphoreTake(ctx->mutex, portMAX_DELAY);
        if (ctx->state == UEP_STATE_RUNNING)
        {
            ctx->state = UEP_STATE_IDLE;
        }
        xSemaphoreGive(ctx->mutex);
    }

    return ret;
}



/**
 * \brief Starts an asynchronous write on an endpoint.
 *
 * This function returns immediately, allowing the write to continue in the
 * background. The memory referred to by the \p data pointer \em must remain
 * valid until the asynchronous write operation completes (successfully or
 * otherwise)!
 *
 * \param[in] ep the endpoint to write to, from 0x81 to <code>\ref
 * UEP_MAX_ENDPOINT | 0x80</code>
 * \param[in] data the data to send
 * \param[in] length the number of bytes to send
 * \param[in] zlp \c true to add a zero-length packet if \p length is a
 * multiple of the endpoint maximum packet size, or \c false to omit the
 * zero-length packet
 * \param[in] cb a callback to invoke when \ref uep_async_write_finish must be
 * called (whose invocation does not necessarily imply the transfer is
 * complete)
 * \retval true if the write started successfully
 * \retval false if an error occurred before the write started (in which case
 * no data has been sent)
 * \exception EPIPE the endpoint was halted at the time of call
 * \exception ECONNRESET the endpoint was disabled at the time of call
 * \pre The endpoint must be enabled in the current configuration and in the
 * current alternate setting of any relevant interfaces.
 */
bool uep_async_write_start(unsigned int ep, const void *data, size_t length, bool zlp,
                           uep_async_cb_t cb)
{
    // Sanity check.
    assert(UEP_DIR(ep));
    assert(1 <= UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);
    assert(!!data == !!length);
    assert(length || zlp);

    // Grab max packet size.
    size_t max_packet = OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL.MPSIZ;

    // Set up transfer parameters.
    uep_ep_t *ctx         = &uep_eps[UEP_IDX(ep)];
    ctx->flags.zlp        = zlp && !(length % max_packet);
    ctx->flags.overflow   = false;
    ctx->transfer.in.data = data;
    ctx->transfer.in.rptr = data;
    ctx->bytes_left       = length;
    ctx->async_cb         = cb;

    // Change endpoint state and start first physical transfer.
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    if (ctx->state == UEP_STATE_IDLE)
    {
        ctx->state = UEP_STATE_RUNNING;
    }
    bool ret = uep_start_pxfr(ep);

    // On failure, clear transfer parameters.
    if (!ret)
    {
        ctx->transfer.in.data = 0;
        ctx->transfer.in.rptr = 0;
        ctx->bytes_left       = 0;
        ctx->async_cb         = 0;
        if (ctx->state == UEP_STATE_RUNNING)
        {
            ctx->state = UEP_STATE_IDLE;
        }
    }
    xSemaphoreGive(ctx->mutex);

    // Done.
    return ret;
}

/**
 * \brief Finishes an asynchronous write on an endpoint.
 *
 * This function returns immediately, whether or not the write was finished,
 * reporting the operation’s current status.
 *
 * \param[in] ep the endpoint being read from, from 0x81 to <code>\ref
 * UEP_MAX_ENDPOINT | 0x80</code>
 * \retval true if the write completed successfully
 * \retval false if an error occurred while the write was in progress (despite
 * which some data may have been sent) or if the write has not yet finished
 * \exception EPIPE the endpoint was halted while the read was in progress
 * \exception ECONNRESET the endpoint was disabled while the read was in progress
 * \exception EINPROGRESS the operation has not yet finished
 * \pre A prior asynchronous write must have been started and not yet reported
 * completed on this endpoint.
 */
bool uep_async_write_finish(unsigned int ep)
{
    // Sanity check.
    assert(UEP_DIR(ep));
    assert(1 <= UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);

    // Try to finish the transfer.
    bool ret = uep_async_finish(ep);
    if (ret || (!ret && errno != EINPROGRESS))
    {
        // Transfer finished, successfully or otherwise; clear transfer
        // parameters and reset endpoint state.
        uep_ep_t *ctx         = &uep_eps[UEP_IDX(ep)];
        ctx->transfer.in.data = 0;
        ctx->transfer.in.rptr = 0;
        ctx->bytes_left       = 0;
        ctx->async_cb         = 0;
        xSemaphoreTake(ctx->mutex, portMAX_DELAY);
        if (ctx->state == UEP_STATE_RUNNING)
        {
            ctx->state = UEP_STATE_IDLE;
        }
        xSemaphoreGive(ctx->mutex);
    }

    return ret;
}



/**
 * \brief Starts waiting for halt status to clear on a halted endpoint.
 *
 * This function returns immediately, allowing the wait to continue in the
 * background. If the endpoint is not halted, this function succeeds and starts
 * a wait which immediately finishes.
 *
 * \param[in] ep the endpoint on which to wait, from 0x01 to \ref
 * UEP_MAX_ENDPOINT or 0x81 to <code>\ref UEP_MAX_ENDPOINT | 0x80</code>
 * \param[in] cb a callback to invoke when \ref uep_async_halt_wait_finish must
 * be called (whose invocation does not necessarily imply the wait is complete)
 * \retval true if the wait started successfully
 * \retval false if an error occurred before the wait started or if the
 * endpoint was not halted at the time of call
 * \exception ECONNRESET the endpoint was disabled at the time of call
 * \exception EINVAL the endpoint was not halted at the time of call
 * \pre The endpoint must be enabled in the current configuration and in the
 * current alternate setting of any relevant interfaces.
 */
bool uep_async_halt_wait_start(unsigned int ep, uep_async_cb_t cb)
{
    // Sanity check.
    assert(1 <= UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);

    uep_ep_t *ctx = &uep_eps[UEP_IDX(ep)];

    // Decide whether to start a wait or return immediately.
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    bool ret;
    switch (ctx->state)
    {
        case UEP_STATE_INACTIVE:
            // Endpoint inactive; return immediately.
            errno = ECONNRESET;
            ret   = false;
            break;

        case UEP_STATE_IDLE:
            // Endpoint is not halted; return immediately.
            errno = EINVAL;
            ret   = false;
            break;

        case UEP_STATE_CLEAR_HALT_PENDING:
            // Prior halt status acknowledged; return immediately.
            ctx->state = UEP_STATE_IDLE;
            errno      = EINVAL;
            ret        = false;
            break;

        case UEP_STATE_HALTED:
            // Endpoint is halted; start waiting.
            ctx->state    = UEP_STATE_HALTED_WAITING;
            ctx->async_cb = cb;
            ret           = true;
            break;

        default:
            // If UEP_STATE_RUNNING or UEP_STATE_HALTED_WAITING, logic error:
            // application tried to start two operations on an endpoint at the
            // same time.
            abort();
    }
    xSemaphoreGive(ctx->mutex);

    return ret;
}

/**
 * \brief Finishes waiting for halt status to clear on a halted endpoint.
 *
 * This function returns immediately, whether or not the wait was finished,
 * reporting the operation’s current status.
 *
 * \param[in] ep the endpoint being waited on, from 0x01 to \ref
 * UEP_MAX_ENDPOINT or 0x81 to <code>\ref UEP_MAX_ENDPOINT | 0x80</code>
 * \retval true if the endpoint halt status has been cleared and the endpoint
 * is now working normally
 * \retval false if an error occurred while the wait was in progress or if the
 * endpoint is still halted
 * \exception ECONNRESET the endpoint was disabled while the wait was in
 * progress
 * \exception EINPROGRESS the operation has not yet finished
 * \pre A prior asynchronous halt wait must have been started and not yet
 * reported completed on this endpoint.
 */
bool uep_async_halt_wait_finish(unsigned int ep)
{
    // Sanity check.
    assert(1 <= UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);

    // Check for event.
    uep_ep_t *ctx = &uep_eps[UEP_IDX(ep)];
    uint8_t event;
    bool ret;
    if (xQueueReceive(ctx->event_queue, &event, 0) == pdTRUE)
    {
        // Event received.
        switch (event)
        {
            case UEP_EVENT_DEACTIVATED_WHILE_HALTED:
                // Endpoint deactivated.
                errno         = ECONNRESET;
                ret           = false;
                ctx->async_cb = 0;
                break;

            case UEP_EVENT_HALT_CLEARED:
                // Halt status cleared.
                ret           = true;
                ctx->async_cb = 0;
                break;

            default:
                // If UEP_EVENT_XFRC or UEP_EVENT_EPDISD, logic error: no
                // transfer should have been in progress.
                abort();
        }
    }
    else
    {
        // No event; there is nothing to report.
        errno = EINPROGRESS;
        ret   = false;
    }

    return ret;
}



/**
 * \cond INTERNAL
 */
/**
 * \brief Sets the halt feature on an endpoint.
 *
 * \param[in] ep the endpoint address to halt
 * \param[in] cb the callback to invoke if halt status actually changes from
 * not halted to halted, or null to not invoke any callback in this case
 * \retval true if the endpoint is now halted
 * \retval false if the endpoint is inactive in the current configuration
 */
bool uep_halt_with_cb(unsigned int ep, void (*cb)(void))
{
    // Sanity check.
    assert(1 <= UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);
    uep_ep_t *ctx = &uep_eps[UEP_IDX(ep)];

    // Take the mutex.
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);

    // Do the state-specific handling.
    bool ret, runcb;
    switch (ctx->state)
    {
        case UEP_STATE_INACTIVE:
            // Cannot set halt feature on an inactive endpoint.
            ret   = false;
            runcb = false;
            break;

        case UEP_STATE_IDLE:
            // Set halt feature.
            ctx->state = UEP_STATE_HALTED;
            ret        = true;
            runcb      = true;
            break;

        case UEP_STATE_RUNNING:
            // Stop transfer first, then set halt feature.
            uep_disable(ep);
            ctx->state = UEP_STATE_HALTED;
            ret        = true;
            runcb      = true;
            break;

        case UEP_STATE_HALTED:
        case UEP_STATE_HALTED_WAITING:
            // Already halted, so nothing to do.
            ret   = true;
            runcb = false;
            break;

        case UEP_STATE_CLEAR_HALT_PENDING:
            // Go back from pending to halted.
            ctx->state = UEP_STATE_HALTED;
            ret        = true;
            runcb      = false;
            break;

        default:
            // State contains illegal value.
            abort();
    }

    // On success, get the endpoint returning STALL handshakes.
    if (ret)
    {
        if (UEP_DIR(ep))
        {
            OTG_FS_DIEPCTLx_t diepctl             = OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL;
            diepctl.EPENA                         = 0;
            diepctl.EPDIS                         = 0;
            diepctl.STALL                         = 1;
            OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL = diepctl;
        }
        else
        {
            OTG_FS_DOEPCTLx_t doepctl             = OTG_FS.DOEP[UEP_NUM(ep) - 1U].DOEPCTL;
            doepctl.EPENA                         = 0;
            doepctl.EPDIS                         = 0;
            doepctl.STALL                         = 1;
            OTG_FS.DOEP[UEP_NUM(ep) - 1U].DOEPCTL = doepctl;
        }
    }

    // Release the mutex.
    xSemaphoreGive(ctx->mutex);

    // Run the callback, if provided and should do so.
    if (cb && runcb)
    {
        cb();
    }

    return ret;
}
/**
 * \endcond
 */

/**
 * \brief Sets the halt feature on an endpoint.
 *
 * The application cannot clear the halt feature once set. The halt feature can
 * only be cleared by the host via a control transfer on endpoint zero.
 *
 * \param[in] ep the endpoint to halt, from 0x01 to \ref UEP_MAX_ENDPOINT or
 * 0x81 to <code>\ref UEP_MAX_ENDPOINT | 0x80</code>
 * \retval true if the endpoint is now halted
 * \retval false if the endpoint is inactive in the current configuration
 */
bool uep_halt(unsigned int ep)
{
    return uep_halt_with_cb(ep, 0);
}

/**
 * \cond INTERNAL
 */
/**
 * \brief Clears the halt feature on an endpoint.
 *
 * \param[in] ep the endpoint address
 * \param[in] cancb the callback to invoke to check whether it is acceptable to
 * clear halt status
 * \param[in] cb the callback to invoke if halt status is cleared
 * \retval true if the endpoint is now not halted
 * \retval false if the endpoint is inactive or the halt feature could not be
 * cleared
 * \pre This function must be invoked on the stack internal task.
 */
bool uep_clear_halt(unsigned int ep, bool (*cancb)(void), void (*cb)(void))
{
    // Sanity check.
    assert(1 <= UEP_NUM(ep) && UEP_NUM(ep) <= UEP_MAX_ENDPOINT);
    uep_ep_t *ctx = &uep_eps[UEP_IDX(ep)];

    // Take the mutex.
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);

    // Do the state-specific handling.
    bool ret, docb = false;
    switch (ctx->state)
    {
        case UEP_STATE_INACTIVE:
            // Cannot clear halt feature on an inactive endpoint.
            ret = false;
            break;

        case UEP_STATE_IDLE:
        case UEP_STATE_RUNNING:
            // Not actually clearing the halt feature, but the request should
            // still succeed.
            ret = true;
            break;

        case UEP_STATE_HALTED:
            if (!cancb || cancb())
            {
                // No check callback or check callback succeeded; halt feature
                // can be cleared. No process is waiting, so we must pend the
                // clear so that another task will observe the halt status at
                // least once and acknowledge it before we go back to idle
                // (otherwise halt status could be set and immediately cleared
                // and the task handling endpoint transfers not even notice,
                // whereas it really ought to do something application-specific
                // in response to that scenario).
                ctx->state = UEP_STATE_CLEAR_HALT_PENDING;
                docb       = true;
                ret        = true;
            }
            else
            {
                // Check callback denies clearing halt status.
                ret = false;
            }
            break;

        case UEP_STATE_HALTED_WAITING:
            if (!cancb || cancb())
            {
                // No check callback or check callback succeeded; halt feature
                // can be cleared. A task is waiting for the halt to clear (and
                // by being in that waiting state has acknowledged the setting
                // of the halt feature), so notify the task and go to idle
                // state.
                ctx->state = UEP_STATE_IDLE;
                docb       = true;
                uep_queue_event(ep, UEP_EVENT_HALT_CLEARED);
                ret = true;
            }
            else
            {
                // Check callback denies clearing halt status.
                ret = false;
            }
            break;

        case UEP_STATE_CLEAR_HALT_PENDING:
            // Do nothing, but successfully. Clearing halt status repeatedly
            // should not fail.
            ret = true;
            break;

        default:
            // State contains illegal value.
            abort();
    }

    // On success, get the endpoint to stop sending STALLs and reset its data
    // toggle. According to USB 2.0 section 9.4.5, even if the halt feature was
    // not actually set, the CLEAR FEATURE request should reset data toggle.
    if (ret)
    {
        if (UEP_DIR(ep))
        {
            OTG_FS_DIEPCTLx_t diepctl             = OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL;
            diepctl.EPENA                         = 0;
            diepctl.EPDIS                         = 0;
            diepctl.STALL                         = 0;
            OTG_FS.DIEP[UEP_NUM(ep) - 1U].DIEPCTL = diepctl;
        }
        else
        {
            OTG_FS_DOEPCTLx_t doepctl             = OTG_FS.DOEP[UEP_NUM(ep) - 1U].DOEPCTL;
            doepctl.EPENA                         = 0;
            doepctl.EPDIS                         = 0;
            doepctl.STALL                         = 0;
            OTG_FS.DOEP[UEP_NUM(ep) - 1U].DOEPCTL = doepctl;
        }
    }

    // Invoke the callback if we appropriate; note that this happens after
    // stopping the STALL handshakes.
    if (docb && cb)
    {
        cb();
    }

    // Release the mutex.
    xSemaphoreGive(ctx->mutex);

    return ret;
}
/**
 * \endcond
 */



/**
 * \cond INTERNAL
 */
/**
 * \brief Activates an endpoint.
 *
 * \param[in] edesc the endpoint descriptor describing the endpoint to activate
 * \param[in] interface the interface number to which the endpoint belongs, or
 * UINT_MAX if none
 * \pre This function must be invoked on the stack internal task.
 * \pre The endpoint must be in UEP_STATE_INACTIVE.
 * \post The endpoint is in UEP_STATE_IDLE.
 */
void uep_activate(const usb_endpoint_descriptor_t *edesc, unsigned int interface)
{
    // Sanity check.
    assert(UEP_NUM(edesc->bEndpointAddress));
    assert(UEP_NUM(edesc->bEndpointAddress) <= UEP_MAX_ENDPOINT);
    uep_ep_t *ctx = &uep_eps[UEP_IDX(edesc->bEndpointAddress)];
    assert(ctx->state == UEP_STATE_INACTIVE);

    // Set up state block.
    ctx->state     = UEP_STATE_IDLE;
    ctx->interface = interface;
    ctx->async_cb  = 0;

    // Enable the endpoint hardware.
    if (UEP_DIR(edesc->bEndpointAddress))
    {
        // The amount of space available for this FIFO is MAX{16,
        // transmit_fifo_words[N]} words. We cannot use more than that amount,
        // because other FIFOs lie beyond that space. However, we can elect to
        // use a smaller FIFO if it would help. Normally, a larger FIFO helps
        // reduce interrupts; multiple packets can be pushed at once, and an
        // interrupt is taken only once half of them have been transmitted.
        // However, the hardware requires that, no matter how large the FIFO
        // is, only seven packets may be present at any time. Fortunately, the
        // half-empty interrupt is inhibited if all seven packets are present,
        // even if the FIFO is less than half full, avoiding an interrupt
        // storm. As the FIFO grows beyond seven max-packets, however,
        // interrupt frequency increases. This is because fewer packets have to
        // be transmitted in order for the FIFO to become half empty. In the
        // limit, once the FIFO reaches 12 packets in size, every packet
        // transmitted brings the FIFO from seven to six packets, making it
        // half empty and causing an interrupt. Absolute minimization of
        // interrupts occurs when the FIFO is seven packets long, so we try to
        // do that. However, we must still obey the other hardware rule, namely
        // that the FIFO must be at least 16 words long. That means we cannot
        // quite achieve seven packets for a max packet size of eight bytes or
        // smaller; however, we do the best we can. All of the above is
        // completely irrelevant if the interrupt minimization flag is set,
        // because then the interrupt fires when the FIFO is completely empty.
        size_t max_packet_words = (edesc->wMaxPacketSize + 3U) / 4U;
        size_t alloc_words =
            MAX(16U, uep0_current_configuration
                         ->transmit_fifo_words[UEP_NUM(edesc->bEndpointAddress) - 1U]);
        size_t fifo_words;
        if (udev_info->flags.minimize_interrupts)
        {
            fifo_words = alloc_words;
        }
        else
        {
            fifo_words = MAX(16U, MIN(alloc_words, max_packet_words * 7U));
        }
        OTG_FS.DIEPTXF[UEP_NUM(edesc->bEndpointAddress) - 1U].INEPTXFD = fifo_words;

        // The FIFO must be large enough to contain at least one packet.
        assert(fifo_words >= max_packet_words);

        // Flush transmit FIFO.
        udev_flush_tx_fifo(edesc->bEndpointAddress);

        // Activate endpoint.
        OTG_FS_DIEPCTLx_t ctl = {
            .SD0PID_SEVNFRM = 1,
            .TXFNUM         = UEP_NUM(edesc->bEndpointAddress),
            .EPTYP          = edesc->bmAttributes.type,
            .USBAEP         = 1,
            .MPSIZ          = edesc->wMaxPacketSize,
        };
        OTG_FS.DIEP[UEP_NUM(edesc->bEndpointAddress) - 1U].DIEPCTL = ctl;
    }
    else
    {
        // Activate endpoint.
        assert(!OTG_FS.DOEP[UEP_NUM(edesc->bEndpointAddress) - 1U].DOEPCTL.USBAEP);
        OTG_FS_DOEPCTLx_t ctl = {
            .SD0PID_SEVENFRM = 1,
            .EPTYP           = edesc->bmAttributes.type,
            .USBAEP          = 1,
            .MPSIZ           = edesc->wMaxPacketSize,
        };
        OTG_FS.DOEP[UEP_NUM(edesc->bEndpointAddress) - 1U].DOEPCTL = ctl;
    }
}

/**
 * \brief Deactivates an endpoint.
 *
 * \param[in] edesc the endpoint descriptor describing the endpoint to
 * deactivate
 * \pre This function must be invoked on the stack internal task.
 * \pre The endpoint must be in something other than UEP_STATE_INACTIVE.
 */
void uep_deactivate(const usb_endpoint_descriptor_t *edesc)
{
    // Sanity check.
    assert(UEP_NUM(edesc->bEndpointAddress) &&
           UEP_NUM(edesc->bEndpointAddress) <= UEP_MAX_ENDPOINT);
    uep_ep_t *ctx = &uep_eps[UEP_IDX(edesc->bEndpointAddress)];

    // Take the semaphore.
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);

    // Based on state, do needed special handling.
    switch (ctx->state)
    {
        case UEP_STATE_INACTIVE:
            // This should never happen.
            abort();

        case UEP_STATE_IDLE:
        case UEP_STATE_HALTED:
        case UEP_STATE_CLEAR_HALT_PENDING:
            // We do not need to do anything in these states.
            break;

        case UEP_STATE_RUNNING:
            // Stop the running transfer before deactivating the endpoint.
            uep_disable(edesc->bEndpointAddress);
            break;

        case UEP_STATE_HALTED_WAITING:
            // Wake up the waiting task before deactivating the endpoint.
            uep_queue_event(edesc->bEndpointAddress, UEP_EVENT_DEACTIVATED_WHILE_HALTED);
            break;
    }

    // Clear the state block.
    ctx->state     = UEP_STATE_INACTIVE;
    ctx->interface = UINT_MAX;
    ctx->async_cb  = 0;

    // Disable the endpoint hardware.
    if (UEP_DIR(edesc->bEndpointAddress))
    {
        OTG_FS_DIEPCTLx_t ctl                                      = {0};
        OTG_FS.DIEP[UEP_NUM(edesc->bEndpointAddress) - 1U].DIEPCTL = ctl;
    }
    else
    {
        OTG_FS_DOEPCTLx_t ctl                                      = {0};
        OTG_FS.DOEP[UEP_NUM(edesc->bEndpointAddress) - 1U].DOEPCTL = ctl;
    }

    // Release the semaphore.
    xSemaphoreGive(ctx->mutex);
}
/**
 * \endcond
 */

/**
 * \}
 */
