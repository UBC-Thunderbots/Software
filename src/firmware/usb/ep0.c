/**
 * \defgroup UEP0 Endpoint zero handling
 *
 * These functions are used to communicate over endpoint zero, the control endpoint.
 * All functions in this section must only be called from the USB stack internal task.
 *
 * \{
 */
#include <FreeRTOS.h>
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <minmax.h>
#include <registers/otg_fs.h>
#include <stdint.h>
#include <string.h>
#include <task.h>
#include <usb.h>

#include "internal.h"

/**
 * \cond INTERNAL
 */
/**
 * \brief The most recent setup-stage-related event that happened on endpoint
 * zero.
 */
uep0_setup_event_t uep0_setup_event = UEP0_SETUP_EVENT_NONE;

/**
 * \brief A pointer to the next location where an OUT transaction to endpoint
 * zero will be written by the ISR.
 */
void *uep0_out_wptr = 0;

/**
 * \brief The length of the OUT data.
 *
 * While the endpoint is enabled, before the transaction is received, this is
 * the maximum number of bytes the ISR is allowed to write into the buffer.
 * After the transaction has been taken, this is the size of the received
 * packet, which may be larger or smaller than the original value.
 */
size_t uep0_out_length = 0U;

/**
 * \brief A bitmask of transfers on endpoint zero that have completed.
 *
 * Bit 0 is for an OUT transfer while bit 1 is for an IN transfer.
 */
unsigned int uep0_xfrc = 0;

/**
 * \brief The currently active configuration.
 */
const udev_config_info_t *uep0_current_configuration = 0;

/**
 * \brief The currently active alternate settings.
 */
uint8_t *uep0_alternate_settings = 0;

/**
 * \brief A double buffer for holding SETUP transactions.
 *
 * Index \ref uep0_setup_packet_wptr will be written into by the ISR whenever a
 * SETUP transaction arrives.
 */
static usb_setup_packet_t uep0_setup_packets[2];

/**
 * \brief The index of the buffer which will receive incoming SETUP
 * transactions.
 *
 * The interrupt service routine uses this index to write SETUP transactions
 * into \ref udev_setup_packets. When the setup stage complete event arrives at
 * the stack internal task, it atomically inverts this index to ensure the
 * SETUP transaction is not overwritten while the control transfer is under
 * way.
 */
static unsigned int uep0_setup_packet_wptr = 0U;

/**
 * \brief The SETUP transaction of the current control transfer.
 *
 * This variable is null if no control transfer is under way.
 */
static const usb_setup_packet_t *uep0_setup_packet = 0;

/**
 * \brief The function that will be invoked after the status stage of the current control
 * transfer.
 */
static void (*uep0_poststatus)(void) = 0;

/**
 * \brief Whether endpoint zero is functionally halted.
 */
static bool uep0_halted = false;
/**
 * \endcond
 */



/**
 * \brief Reads the OUT data stage of a control transfer.
 *
 * This function blocks until the data stage finishes or an error is detected.
 *
 * \param[out] buffer the buffer into which to store the received data, which
 * must be large enough to accommodate the length specified in the SETUP packet
 *
 * \retval true if the read completed successfully
 * \retval false if an error occurred before or during the read
 *
 * \exception ECONNRESET another SETUP packet was received since this control
 * transfer started
 * \exception EPIPE the endpoint was disabled, either when the function was
 * first called or while the transfer was occurring, due to USB reset
 * signalling, cable unplug, or a call to \ref udev_detach
 * \exception EOVERFLOW the host attempted to send more data than it specified
 * in the SETUP packet
 */
bool uep0_data_read(void *buffer)
{
    // Sanity check.
    assert(!uep0_setup_packet->bmRequestType.direction);
    assert(uep0_setup_packet->wLength);
    assert(buffer);

    // Run the data stage.
    uep0_out_wptr     = buffer;
    size_t max_packet = udev_info->device_descriptor.bMaxPacketSize0;
    size_t left       = uep0_setup_packet->wLength;
    while (left)
    {
        // Check for USB reset, cable unplug, \ref udev_detach, or new SETUP
        // packet before enabling the endpoint.
        if (__atomic_load_n(&udev_state_change, __ATOMIC_RELAXED) !=
            UDEV_STATE_CHANGE_NONE)
        {
            errno = EPIPE;
            return false;
        }
        if (__atomic_load_n(&uep0_setup_event, __ATOMIC_RELAXED) != UEP0_SETUP_EVENT_NONE)
        {
            errno = ECONNRESET;
            return false;
        }

        // Enable the endpoint for one packet.
        size_t this     = left > max_packet ? max_packet : left;
        uep0_out_length = this;
        // Prevent sinking writes to uep0_out_{wptr,length} below this line.
        __atomic_signal_fence(__ATOMIC_RELEASE);
        {
            OTG_FS_DOEPTSIZ0_t tsiz = {.PKTCNT = 1, .XFRSIZ = max_packet};
            OTG_FS.DOEPTSIZ0        = tsiz;
            OTG_FS_DOEPCTL0_t ctl   = {.EPENA = 1, .CNAK = 1};
            OTG_FS.DOEPCTL0         = ctl;
        }

        // Wait for transfer complete, a new setup-stage event, or a state
        // change.
        unsigned int xfrc;
        while (!(xfrc = __atomic_exchange_n(&uep0_xfrc, 0, __ATOMIC_RELAXED)))
        {
            bool abort = false;
            if (__atomic_load_n(&udev_state_change, __ATOMIC_RELAXED) !=
                UDEV_STATE_CHANGE_NONE)
            {
                // A USB reset, cable unplug, or \ref udev_detach happened.
                abort = true;
                errno = EPIPE;
            }
            if (__atomic_load_n(&uep0_setup_event, __ATOMIC_RELAXED) !=
                UEP0_SETUP_EVENT_NONE)
            {
                // A new SETUP packet was received.
                abort = true;
                errno = ECONNRESET;
            }
            if (abort)
            {
                // Because of the event that occurred, we must abort the
                // transfer that is currently running.
                //
                // In the case of a new SETUP packet arriving, the hardware
                // disables the endpoint automatically as soon as it sees the
                // token, which is awesome. However, there is still a tiny race
                // condition: we could write EPENA=1 just a moment after the
                // SETUP token arrives, thus starting a transfer which we think
                // is for the old control transfer but which will actually see
                // data for the new control transfer.
                //
                // Also, USB reset signalling will not disable the endpoint (it
                // will clear USBAEP for all nonzero endpoints which also
                // inhibits traffic, but endpoint zero’s USBAEP is hardwired to
                // 1). So, we still need to do some work in firmware to be
                // absolutely certain no traffic will be accepted at all.
                //
                // No matter what we do, this is still slightly racy. If a
                // SETUP token arrives a moment before we write EPENA=1, there
                // is still a small window of time after we write EPENA=1 and
                // before we get in here to stop things where some traffic
                // could flow. This race is impossible to fix AFAICT. However,
                // it should be hard to trigger ever, and impossible to trigger
                // unless the host is doing something rather non-standard like
                // abandoning a control transfer in the middle for no good
                // reason almost immediately after starting it.
                //
                // We can’t actually disable endpoint zero under application
                // request (the EPDIS bit is not implemented for endpoint
                // zero). So, do the best we can: endpoint local NAK status.
                OTG_FS_DOEPCTL0_t ctl = {.SNAK = 1};
                OTG_FS.DOEPCTL0       = ctl;
                // Normally, when disabling a non-zero OUT endpoint, we first
                // achieve global OUT NAK (which fires an interrupt), then
                // disable the endpoint using EPDIS (which fires a second
                // interrupt), thus allowing us to block until the endpoint is
                // fully disabled. We can’t use EPDIS due to it not existing
                // for endpoint zero, and local NAK status for an OUT endpoint
                // doesn’t fire an interrupt, so we have no choice but to spin
                // rather than block. This is likely to take much less than one
                // FreeRTOS timeslice, so it would be counterproductive to
                // sleep.
                while (!OTG_FS.DOEPCTL0.NAKSTS)
                    ;
                // Prevent the write to uep0_xfrc from being hoisted above
                // here.
                __atomic_signal_fence(__ATOMIC_ACQUIRE);
                // While we were waiting to achieve local NAK status, it’s
                // possible the transfer might have completed. If that
                // happened, just throw away the transfer complete notification
                // from the ISR. We don’t care about the data, but we must
                // clear the flag to maintain the invariant that the flag is
                // false whenever no transfer is being done.
                __atomic_store_n(&uep0_xfrc, 0, __ATOMIC_RELAXED);
                // Report to the caller.
                return false;
            }
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        // Sanity check: only OUT endpoint zero is being activated, so IN
        // endpoint zero must never report transfer complete.
        assert(!(xfrc & 2));

        // A packet was shipped. In this case, the ISR sets uep0_out_length to
        // the length of the packet actually received, while copying the data
        // into the buffer and advancing the pointer.
        if (uep0_out_length > this)
        {
            errno = EOVERFLOW;
            return false;
        }
        else
        {
            left -= uep0_out_length;
            uep0_out_wptr = ((char *)uep0_out_wptr) + uep0_out_length;
        }
    }

    // Transfer finished OK.
    return true;
}

/**
 * \brief Writes the IN data stage of a control transfer.
 *
 * This function blocks until the data stage finishes or an error is detected.
 *
 * \param[in] data the data to send
 * \param[in] length the maximum number of bytes to send, which is permitted to
 * be more or less than the length requested in the SETUP packet (in the former
 * case, the data is truncated)
 *
 * \retval true if the write completed successfully
 * \retval false if an error occurred before or during the write
 *
 * \exception ECONNRESET another SETUP packet was received since this control
 * transfer started
 * \exception EPIPE the endpoint was disabled, either when the function was
 * first called or while the transfer was occurring, due to USB reset
 * signalling, cable unplug, or a call to \ref udev_detach
 */
bool uep0_data_write(const void *data, size_t length)
{
    // We consider writing more data than requested to be a successful
    // operation where the data actually delivered is truncated. For
    // uniformity, this policy should apply even when zero bytes are requested.
    // Strictly speaking when wLength=0 a transfer is treated more like an OUT
    // transfer than an IN transfer (the status stage is IN). If an application
    // cares, it can always check the SETUP packet in more detail. However, in
    // the common case where bRequest fully identifies a request, this clause
    // prevents the application from having to special-case wLength=0.
    if (!uep0_setup_packet->wLength)
    {
        return true;
    }

    // Sanity check.
    assert(uep0_setup_packet->bmRequestType.direction);
    assert(data || !length);

    // Clamp length to amount requested by host.
    if (length > uep0_setup_packet->wLength)
    {
        length = uep0_setup_packet->wLength;
    }

    // Decide whether we will send a ZLP.
    bool need_zlp = (length != uep0_setup_packet->wLength) &&
                    !(length & (udev_info->device_descriptor.bMaxPacketSize0 - 1U));

    // It could occur that the device sends the last data packet of the data
    // stage, but the ACK handshake returned is corrupted. Then the device will
    // still think it’s in the data stage, but the host will think it’s
    // advanced to the status stage. This will result in an attempt to send an
    // OUT transaction while the OUT endpoint is disabled. We fix this by
    // starting the status stage now.
    {
        uep0_out_wptr   = 0;
        uep0_out_length = 0;
        // Prevent sinking writes to uep0_out_{wptr,length} below this line.
        __atomic_signal_fence(__ATOMIC_RELEASE);
        OTG_FS_DOEPTSIZ0_t tsiz = {
            .PKTCNT = 1, .XFRSIZ = udev_info->device_descriptor.bMaxPacketSize0};
        OTG_FS.DOEPTSIZ0      = tsiz;
        OTG_FS_DOEPCTL0_t ctl = {.EPENA = 1, .CNAK = 1};
        OTG_FS.DOEPCTL0       = ctl;
    }

    // DIEPTSIZ0 provides a 2-bit PKTCNT field and a 7-bit XFRSIZ field. That
    // means we can only ship blocks of up to min{3 packets, 127 bytes} at a
    // time. For a 64-byte max packet size, that means one packet at a time.
    // For anything smaller, than means three packets at a time. Because these
    // blocks are so small, rather than having the ISR copy data into the
    // transmit FIFO, we just do it directly on enabling the endpoint. The
    // endpoint 0 transmit FIFO is always long enough for this to be safe.
    const uint8_t *dptr = data;
    while (length || need_zlp)
    {
        // Check for USB reset, cable unplug, \ref udev_detach, new SETUP
        // packet, or status stage complete before enabling the endpoint.
        if (__atomic_load_n(&udev_state_change, __ATOMIC_RELAXED) !=
            UDEV_STATE_CHANGE_NONE)
        {
            errno = EPIPE;
            return false;
        }
        if (__atomic_load_n(&uep0_setup_event, __ATOMIC_RELAXED) != UEP0_SETUP_EVENT_NONE)
        {
            errno = ECONNRESET;
            return false;
        }
        unsigned int xfrc = __atomic_exchange_n(&uep0_xfrc, 0, __ATOMIC_RELAXED);
        if (xfrc & 1)
        {
            return true;
        }

        // Sanity check, we did not start an IN transfer so it cannot have finished.
        assert(!(xfrc & 2));

        // Start a block.
        size_t to_push;
        if (length)
        {
            OTG_FS_DIEPTSIZ0_t tsiz;
            tsiz.PKTCNT = udev_info->device_descriptor.bMaxPacketSize0 == 64U ? 1U : 3U;
            tsiz.XFRSIZ = tsiz.PKTCNT * udev_info->device_descriptor.bMaxPacketSize0;
            if (tsiz.XFRSIZ > length)
            {
                tsiz.XFRSIZ = length;
            }
            tsiz.PKTCNT =
                (tsiz.XFRSIZ + udev_info->device_descriptor.bMaxPacketSize0 - 1U) /
                udev_info->device_descriptor.bMaxPacketSize0;
            OTG_FS.DIEPTSIZ0 = tsiz;
            to_push          = tsiz.XFRSIZ;
        }
        else
        {
            OTG_FS_DIEPTSIZ0_t tsiz = {.PKTCNT = 1, .XFRSIZ = 0};
            OTG_FS.DIEPTSIZ0        = tsiz;
            to_push                 = 0U;
            need_zlp                = false;
        }
        OTG_FS_DIEPCTL0_t ctl = {
            .EPENA = 1, .CNAK = 1, .TXFNUM = 0, .MPSIZ = OTG_FS.DIEPCTL0.MPSIZ};
        OTG_FS.DIEPCTL0 = ctl;
        length -= to_push;

        // The USB engine very strictly requires that all data for a whole
        // packet be pushed into the FIFO at once. This rule even applies
        // ACROSS DIFFERENT FIFOS. Pushing part of a packet, then going and
        // doing some work on a completely different FIFO, then finishing the
        // original packet makes the USB engine fall over and die. For most
        // endpoints, we start (potentially) very large physical transfers and
        // then let the ISR push packets—packet pushing is serialized simply by
        // being in the ISR. For endpoint zero, the small size of PKTCNT/XFRSIZ
        // means large physical transfers are impossible. For a small physical
        // transfer, we save the overhead of getting into the ISR to push data
        // by pushing it immediately when the endpoint is enabled, right here.
        // However, we must ensure the ISR doesn’t fire during this time and go
        // off and push data on some other endpoint. So, we must do this inside
        // a critical section.
        taskENTER_CRITICAL();
        udev_tx_copy_in(&OTG_FS_FIFO[0][0], dptr, to_push);
        dptr += to_push;
        taskEXIT_CRITICAL();

        // Wait for transfer complete, a new setup-stage event, or a state
        // change.
        while (!(xfrc = __atomic_exchange_n(&uep0_xfrc, 0, __ATOMIC_RELAXED)))
        {
            bool abort = false;
            if (__atomic_load_n(&udev_state_change, __ATOMIC_RELAXED) !=
                UDEV_STATE_CHANGE_NONE)
            {
                // A USB reset, cable unplug, or \ref udev_detach happened.
                abort = true;
                errno = EPIPE;
            }
            if (__atomic_load_n(&uep0_setup_event, __ATOMIC_RELAXED) !=
                UEP0_SETUP_EVENT_NONE)
            {
                // A new SETUP packet was received.
                abort = true;
                errno = ECONNRESET;
            }
            if (abort)
            {
                // See the long comment in uep0_data_read for why this.
                OTG_FS_DOEPCTL0_t octl = {.SNAK = 1};
                OTG_FS.DOEPCTL0        = octl;
                OTG_FS_DIEPCTL0_t ictl = {.SNAK = 1};
                OTG_FS.DIEPCTL0        = ictl;
                while (!OTG_FS.DOEPCTL0.NAKSTS)
                    ;
                while (!OTG_FS.DIEPCTL0.NAKSTS)
                    ;
                udev_flush_tx_fifo(0x80);
                // Prevent the write to uep0_xfrc from being hoisted above
                // here.
                __atomic_signal_fence(__ATOMIC_ACQUIRE);
                __atomic_store_n(&uep0_xfrc, 0, __ATOMIC_RELAXED);
                // Report to the caller.
                return false;
            }
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        if (xfrc & 1)
        {
            // OUT status stage complete.
            if (!(xfrc & 2))
            {
                // IN data stage not complete when status stage finished. Abort
                // endpoint. See long comment in uep0_data_read for why done
                // this way.
                OTG_FS_DIEPCTL0_t ictl = {.SNAK = 1};
                OTG_FS.DIEPCTL0        = ictl;
                while (!OTG_FS.DIEPCTL0.NAKSTS)
                    ;
                udev_flush_tx_fifo(0x80);
                // Prevent the write to uep0_xfrc from being hoisted above
                // here.
                __atomic_signal_fence(__ATOMIC_ACQUIRE);
                __atomic_store_n(&uep0_xfrc, 0, __ATOMIC_RELAXED);
                return true;
            }
            else
            {
                // IN data stage finished as well as OUT status stage.
                return true;
            }
        }
    }

    // Transfer finished OK.
    return true;
}

/**
 * \brief Sets a callback to be called after the status stage completes.
 *
 * \param[in] cb the callback
 */
void uep0_set_poststatus(void (*cb)(void))
{
    uep0_poststatus = cb;
}

/**
 * \cond INTERNAL
 */
/**
 * \brief Looks up a buffer into which the next arriving SETUP packet should be
 * written.
 *
 * \return the buffer
 */
void *uep0_writable_setup_packet(void)
{
    return &uep0_setup_packets[uep0_setup_packet_wptr];
}

/**
 * \brief Runs the status stage for a successful control transfer.
 *
 * This function returns once the status stage has finished.
 * This function is responsible for calling the poststatus callback, if one is registered.
 */
static void uep0_status_stage(void)
{
    if (!uep0_setup_packet->bmRequestType.direction || !uep0_setup_packet->wLength)
    {
        // Data stage was OUT or not present, so status stage will be IN.
        OTG_FS_DIEPTSIZ0_t tsiz = {.PKTCNT = 1, .XFRSIZ = 0};
        OTG_FS.DIEPTSIZ0        = tsiz;
        OTG_FS_DIEPCTL0_t ctl   = {
            .EPENA = 1, .CNAK = 1, .TXFNUM = 0, .MPSIZ = OTG_FS.DIEPCTL0.MPSIZ};
        OTG_FS.DIEPCTL0 = ctl;

        // Wait until transfer complete.
        unsigned int xfrc;
        while (!(xfrc = __atomic_exchange_n(&uep0_xfrc, 0, __ATOMIC_RELAXED)))
        {
            bool abort = false;
            if (__atomic_load_n(&udev_state_change, __ATOMIC_RELAXED) !=
                UDEV_STATE_CHANGE_NONE)
            {
                // A USB reset, cable unplug, or \ref udev_detach happened.
                abort = true;
            }
            if (__atomic_load_n(&uep0_setup_event, __ATOMIC_RELAXED) !=
                UEP0_SETUP_EVENT_NONE)
            {
                // A new SETUP packet was received.
                abort = true;
            }
            if (abort)
            {
                // See the long comment in uep0_data_read for why this.
                OTG_FS_DIEPCTL0_t ictl = {.SNAK = 1};
                OTG_FS.DIEPCTL0        = ictl;
                while (!OTG_FS.DIEPCTL0.NAKSTS)
                    ;
                // Prevent the write to uep0_xfrc from being hoisted above
                // here.
                __atomic_signal_fence(__ATOMIC_ACQUIRE);
                __atomic_store_n(&uep0_xfrc, 0, __ATOMIC_RELAXED);
                break;
            }
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        // Sanity check.
        if (xfrc)
        {
            assert(xfrc == 2);
        }
    }
    else
    {
        // Data stage was IN so status stage will be OUT. OUT status stages are
        // started in uep0_data_write, so no need to do it here. However, we
        // should still wait until the transfer finishes. Wait until transfer
        // complete.
        unsigned int xfrc;
        while (!(xfrc = __atomic_exchange_n(&uep0_xfrc, 0, __ATOMIC_RELAXED)))
        {
            bool abort = false;
            if (__atomic_load_n(&udev_state_change, __ATOMIC_RELAXED) !=
                UDEV_STATE_CHANGE_NONE)
            {
                // A USB reset, cable unplug, or \ref udev_detach happened.
                abort = true;
            }
            if (__atomic_load_n(&uep0_setup_event, __ATOMIC_RELAXED) !=
                UEP0_SETUP_EVENT_NONE)
            {
                // A new SETUP packet was received.
                abort = true;
            }
            if (abort)
            {
                // See the long comment in uep0_data_read for why this.
                OTG_FS_DOEPCTL0_t octl = {.SNAK = 1};
                OTG_FS.DOEPCTL0        = octl;
                while (!OTG_FS.DOEPCTL0.NAKSTS)
                    ;
                // Prevent the write to uep0_xfrc from being hoisted above
                // here.
                __atomic_signal_fence(__ATOMIC_ACQUIRE);
                __atomic_store_n(&uep0_xfrc, 0, __ATOMIC_RELAXED);
                break;
            }
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        // Sanity check.
        if (xfrc)
        {
            assert(xfrc == 1);
        }
    }

    if (uep0_poststatus)
    {
        uep0_poststatus();
        uep0_poststatus = 0;
    }
}

/**
 * \brief Enters the current configuration.
 *
 * This function activates all nonzero endpoints.
 * It then invokes the enter callback for the configuration and then the interfaces and
 * alternate settings.
 */
static void uep0_enter_configuration(void)
{
    if (uep0_current_configuration)
    {
        // Configure all the nonzero transmit FIFOs.
        OTG_FS_DIEPTXF0_t txf0 = OTG_FS.DIEPTXF0;
        unsigned int fifo_used = txf0.TX0FSA + txf0.TX0FD;
        for (unsigned int ep = 1U; ep <= UEP_MAX_ENDPOINT; ++ep)
        {
            OTG_FS_DIEPTXFx_t txf = {
                .INEPTXFD =
                    MAX(16U, uep0_current_configuration->transmit_fifo_words[ep - 1U]),
                .INEPTXSA = fifo_used};
            OTG_FS.DIEPTXF[ep - 1U] = txf;
            fifo_used += txf.INEPTXFD;
        }
        assert(fifo_used <= 320U);

        // Scan the descriptor block and activate on all the endpoints.
        const uint8_t *base = (const uint8_t *)uep0_current_configuration->descriptors;
        const uint8_t *dptr = base;
        unsigned int endpoints_to_skip      = 0U;
        unsigned int endpoints_in_interface = 0U;
        unsigned int last_interface         = UINT_MAX;
        while (dptr - base < uep0_current_configuration->descriptors->wTotalLength)
        {
            if (dptr[1U] == USB_DTYPE_ENDPOINT)
            {
                if (endpoints_to_skip)
                {
                    --endpoints_to_skip;
                }
                else
                {
                    uep_activate((const usb_endpoint_descriptor_t *)dptr, last_interface);
                    if (endpoints_in_interface)
                    {
                        --endpoints_in_interface;
                        if (!endpoints_in_interface)
                        {
                            last_interface = UINT_MAX;
                        }
                    }
                }
            }
            else if (dptr[1U] == USB_DTYPE_INTERFACE)
            {
                const usb_interface_descriptor_t *idesc =
                    (const usb_interface_descriptor_t *)dptr;
                if (idesc->bAlternateSetting)
                {
                    // This descriptor describes a non-default alternate setting for an
                    // interface. Any endpoint descriptors falling under this interface
                    // descriptor only apply when that non-default alternate setting is
                    // selected. We should ignore all those endpoint descriptors now
                    // because only alternate setting zero of each interface is selected.
                    endpoints_to_skip = idesc->bNumEndpoints;
                }
                else
                {
                    // The next specified number of endpoints will be associated with this
                    // interface.
                    endpoints_in_interface = idesc->bNumEndpoints;
                    last_interface         = idesc->bInterfaceNumber;
                }
            }
            dptr += dptr[0U];
        }

        // Now run the configuration entry callback.
        if (uep0_current_configuration->on_enter)
        {
            uep0_current_configuration->on_enter();
        }

        // Now allocate the alternate settings array, initialize all interfaces to
        // alternate setting zero, and run the alternate setting entry callbacks.
        uep0_alternate_settings =
            malloc(sizeof(*uep0_alternate_settings) *
                   uep0_current_configuration->descriptors->bNumInterfaces);
        for (unsigned int intf = 0;
             intf < uep0_current_configuration->descriptors->bNumInterfaces; ++intf)
        {
            uep0_alternate_settings[intf] = 0U;
            const udev_interface_info_t *iinfo =
                uep0_current_configuration->interfaces[intf];
            assert(!iinfo->alternate_settings[0U].can_enter);
            if (iinfo && iinfo->alternate_settings[0U].on_enter)
            {
                iinfo->alternate_settings[0U].on_enter();
            }
        }
    }
}

/**
 * \brief Exits the current configuration.
 *
 * This function deactivates all nonzero endpoints.
 * It then invokes the exit callbacks for all current alternate settings and interfaces
 * and for the configuration itself.
 */
static void uep0_exit_configuration(void)
{
    if (uep0_current_configuration)
    {
        // First order of business, scan the descriptor block and deactivate all
        // endpoints.
        const uint8_t *base = (const uint8_t *)uep0_current_configuration->descriptors;
        const uint8_t *dptr = base;
        unsigned int endpoints_to_skip = 0U;
        while (dptr - base < uep0_current_configuration->descriptors->wTotalLength)
        {
            if (dptr[1U] == USB_DTYPE_ENDPOINT)
            {
                if (endpoints_to_skip)
                {
                    --endpoints_to_skip;
                }
                else
                {
                    const usb_endpoint_descriptor_t *edesc =
                        (const usb_endpoint_descriptor_t *)dptr;
                    uep_deactivate(edesc);
                }
            }
            else if (dptr[1U] == USB_DTYPE_INTERFACE)
            {
                const usb_interface_descriptor_t *idesc =
                    (const usb_interface_descriptor_t *)dptr;
                if (idesc->bAlternateSetting !=
                    uep0_alternate_settings[idesc->bInterfaceNumber])
                {
                    // This descriptor describes an alternate setting that is not
                    // currently selected. Any endpoint descriptors falling under this
                    // interface descriptor only apply when that alternate setting is
                    // selected. We should therefore ignore all those endpoint
                    // descriptors.
                    endpoints_to_skip = idesc->bNumEndpoints;
                }
            }
            dptr += dptr[0U];
        }

        // Now run the alternate setting exit callbacks.
        for (unsigned int intf = 0;
             intf < uep0_current_configuration->descriptors->bNumInterfaces; ++intf)
        {
            uint8_t setting = uep0_alternate_settings[intf];
            const udev_interface_info_t *iinfo =
                uep0_current_configuration->interfaces[intf];
            if (iinfo && iinfo->alternate_settings[setting].on_exit)
            {
                iinfo->alternate_settings[setting].on_exit();
            }
        }

        // Next, run the configuration exit callback.
        if (uep0_current_configuration->on_exit)
        {
            uep0_current_configuration->on_exit();
        }

        // Clean up.
        free(uep0_alternate_settings);
        uep0_alternate_settings    = 0;
        uep0_current_configuration = 0;
    }
}

/**
 * \brief Activates the endpoints for an interface alternate setting.
 */
static void uep0_altsetting_activate_endpoints(
    unsigned int interface, const usb_interface_descriptor_t *interface_descriptor)
{
    const uint8_t *dptr    = (const uint8_t *)interface_descriptor;
    unsigned int endpoints = interface_descriptor->bNumEndpoints;
    while (endpoints)
    {
        if (dptr[1U] == USB_DTYPE_ENDPOINT)
        {
            const usb_endpoint_descriptor_t *edesc =
                (const usb_endpoint_descriptor_t *)dptr;
            uep_activate(edesc, interface);
            --endpoints;
        }
        dptr += dptr[0U];
    }
}

/**
 * \brief Deactivates the endpoints for an interface alternate setting.
 */
static void uep0_altsetting_deactivate_endpoints(
    const usb_interface_descriptor_t *interface_descriptor)
{
    const uint8_t *dptr    = (const uint8_t *)interface_descriptor;
    unsigned int endpoints = interface_descriptor->bNumEndpoints;
    while (endpoints)
    {
        if (dptr[1U] == USB_DTYPE_ENDPOINT)
        {
            const usb_endpoint_descriptor_t *edesc =
                (const usb_endpoint_descriptor_t *)dptr;
            uep_deactivate(edesc);
            --endpoints;
        }
        dptr += dptr[0U];
    }
}

/**
 * \brief Handles control transfers when no other handler is interested.
 *
 * \param[in] pkt the payload of the SETUP transaction starting the transfer
 *
 * \retval true if the handler accepted the transfer
 * \retval false if the handler could not handle the transfer
 */
static bool uep0_default_handler(const usb_setup_packet_t *pkt)
{
    static const uint16_t ZERO16 = 0U;
    if (pkt->bmRequestType.type == USB_CTYPE_STANDARD)
    {
        if (pkt->bmRequestType.recipient == USB_RECIPIENT_DEVICE)
        {
            if (pkt->bRequest == USB_CREQ_GET_CONFIGURATION &&
                pkt->bmRequestType.direction == 1 && !pkt->wValue && !pkt->wIndex &&
                pkt->wLength == 1U && !uep0_halted)
            {
                uint8_t conf =
                    uep0_current_configuration
                        ? uep0_current_configuration->descriptors->bConfigurationValue
                        : 0U;
                uep0_data_write(&conf, sizeof(conf));
                return true;
            }
            else if (pkt->bRequest == USB_CREQ_GET_DESCRIPTOR &&
                     pkt->bmRequestType.direction == 1 && !uep0_halted)
            {
                const void *descriptor = 0;
                size_t length          = 0;
                switch (pkt->wValue >> 8U)
                {
                    case USB_DTYPE_DEVICE:
                        if ((pkt->wValue & 0xFFU) == 0)
                        {
                            descriptor = &udev_info->device_descriptor;
                        }
                        break;

                    case USB_DTYPE_CONFIGURATION:
                        if ((pkt->wValue & 0xFFU) <
                            udev_info->device_descriptor.bNumConfigurations)
                        {
                            descriptor = udev_info->configurations[pkt->wValue & 0xFFU]
                                             ->descriptors;
                            length = ((const usb_configuration_descriptor_t *)descriptor)
                                         ->wTotalLength;
                        }
                        break;

                    case USB_DTYPE_STRING:
                        if (!(pkt->wValue & 0xFFU))
                        {
                            descriptor = udev_info->string_zero_descriptor;
                        }
                        else if ((pkt->wValue & 0xFFU) <= udev_info->string_count)
                        {
                            for (const udev_language_info_t *lang =
                                     udev_info->language_table;
                                 lang->strings && !descriptor; ++lang)
                            {
                                if (lang->id == pkt->wIndex)
                                {
                                    descriptor =
                                        lang->strings[(pkt->wValue & 0xFFU) - 1U];
                                }
                            }
                        }
                        break;
                }

                if (descriptor)
                {
                    if (!length)
                    {
                        length = *(const uint8_t *)descriptor;
                    }
                    uep0_data_write(descriptor, length);
                    return true;
                }
            }
            else if (pkt->bRequest == USB_CREQ_GET_STATUS &&
                     pkt->bmRequestType.direction == 1 && !pkt->wValue && !pkt->wIndex &&
                     pkt->wLength == 2U)
            {
                uint16_t status = 0U;
                if (udev_info->flags.self_powered)
                {
                    status |= 1U;
                }
                uep0_data_write(&status, sizeof(status));
                return true;
            }
            else if (pkt->bRequest == USB_CREQ_SET_ADDRESS && pkt->wValue <= 127 &&
                     !pkt->wIndex && !pkt->wLength && !uep0_halted)
            {
                OTG_FS.DCFG.DAD = pkt->wValue;
                return true;
            }
            else if (pkt->bRequest == USB_CREQ_SET_CONFIGURATION && !pkt->wIndex &&
                     !pkt->wLength && !uep0_halted)
            {
                const udev_config_info_t *new_config = 0;
                if (pkt->wValue)
                {
                    for (unsigned int i = 0;
                         !new_config &&
                         i < udev_info->device_descriptor.bNumConfigurations;
                         ++i)
                    {
                        if (udev_info->configurations[i]
                                ->descriptors->bConfigurationValue == pkt->wValue)
                        {
                            new_config = udev_info->configurations[i];
                        }
                    }
                    if (!new_config)
                    {
                        return false;
                    }
                }

                if (new_config && new_config->can_enter && !new_config->can_enter())
                {
                    return false;
                }

                uep0_exit_configuration();
                uep0_current_configuration = new_config;
                uep0_enter_configuration();

                return true;
            }
        }
        else if (pkt->bmRequestType.recipient == USB_RECIPIENT_INTERFACE &&
                 uep0_current_configuration &&
                 pkt->wIndex < uep0_current_configuration->descriptors->bNumInterfaces)
        {
            if (pkt->bRequest == USB_CREQ_GET_INTERFACE && !pkt->wValue &&
                pkt->wLength == 1U && !uep0_halted)
            {
                uep0_data_write(&uep0_alternate_settings[pkt->wIndex], 1U);
                return true;
            }
            else if (pkt->bRequest == USB_CREQ_GET_STATUS && !pkt->wValue &&
                     pkt->wLength == 2U)
            {
                uep0_data_write(&ZERO16, sizeof(ZERO16));
                return true;
            }
            else if (pkt->bRequest == USB_CREQ_SET_INTERFACE && !pkt->wLength &&
                     !uep0_halted)
            {
                unsigned int interface = pkt->wIndex;
                unsigned int new_as    = pkt->wValue;
                const usb_interface_descriptor_t *newidesc =
                    uutil_find_interface_descriptor(
                        uep0_current_configuration->descriptors, interface, new_as);
                if (newidesc)
                {
                    const udev_interface_info_t *iinfo =
                        uep0_current_configuration->interfaces[interface];
                    unsigned int old_as = uep0_alternate_settings[interface];
                    const usb_interface_descriptor_t *oldidesc =
                        uutil_find_interface_descriptor(
                            uep0_current_configuration->descriptors, interface, old_as);
                    assert(oldidesc);
                    if (iinfo && iinfo->alternate_settings[new_as].can_enter &&
                        !iinfo->alternate_settings[new_as].can_enter())
                    {
                        return false;
                    }
                    uep0_altsetting_deactivate_endpoints(oldidesc);
                    if (iinfo && iinfo->alternate_settings[old_as].on_exit)
                    {
                        iinfo->alternate_settings[old_as].on_exit();
                    }
                    uep0_alternate_settings[interface] = new_as;
                    uep0_altsetting_activate_endpoints(interface, newidesc);
                    if (iinfo && iinfo->alternate_settings[new_as].on_enter)
                    {
                        iinfo->alternate_settings[new_as].on_enter();
                    }
                    return true;
                }
            }
        }
        else if (pkt->bmRequestType.recipient == USB_RECIPIENT_ENDPOINT &&
                 UEP_NUM(pkt->wIndex) <= UEP_MAX_ENDPOINT)
        {
            if (pkt->bRequest == USB_CREQ_CLEAR_FEATURE && !pkt->wLength)
            {
                if (pkt->wValue == USB_FEATURE_ENDPOINT_HALT)
                {
                    if (!UEP_NUM(pkt->wIndex))
                    {
                        // Endpoint zero always allows halt feature to be
                        // cleared.
                        uep0_halted = false;
                        return true;
                    }
                    else
                    {
                        const udev_endpoint_info_t *einfo =
                            uutil_find_endpoint_info(pkt->wIndex);
                        if (einfo)
                        {
                            return uep_clear_halt(pkt->wIndex, einfo->can_clear_halt,
                                                  einfo->on_clear_halt);
                        }
                    }
                }
            }
            else if (pkt->bRequest == USB_CREQ_GET_STATUS && !pkt->wValue &&
                     pkt->wLength == 2U)
            {
                if (!UEP_NUM(pkt->wIndex))
                {
                    uint16_t status = uep0_halted ? 1U : 0U;
                    uep0_data_write(&status, sizeof(status));
                    return true;
                }
                else
                {
                    uep_ep_t *ctx = &uep_eps[UEP_IDX(pkt->wIndex)];
                    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
                    uep_state_t state = ctx->state;
                    xSemaphoreGive(ctx->mutex);
                    bool exists, halted;
                    switch (state)
                    {
                        case UEP_STATE_INACTIVE:
                            exists = false;
                            break;

                        case UEP_STATE_IDLE:
                        case UEP_STATE_RUNNING:
                            exists = true;
                            halted = false;
                            break;

                        case UEP_STATE_HALTED:
                        case UEP_STATE_HALTED_WAITING:
                            exists = true;
                            halted = true;
                            break;

                        case UEP_STATE_CLEAR_HALT_PENDING:
                            exists = true;
                            halted = false;
                            break;

                        default:
                            abort();
                    }
                    if (exists)
                    {
                        uint16_t status = halted ? 1U : 0U;
                        uep0_data_write(&status, sizeof(status));
                        return true;
                    }
                }
            }
            else if (pkt->bRequest == USB_CREQ_SET_FEATURE && !pkt->wLength)
            {
                if (pkt->wValue == USB_FEATURE_ENDPOINT_HALT)
                {
                    if (!UEP_NUM(pkt->wIndex))
                    {
                        uep0_halted = true;
                        return true;
                    }
                    else
                    {
                        const udev_endpoint_info_t *einfo =
                            uutil_find_endpoint_info(pkt->wIndex);
                        if (einfo)
                        {
                            uep_halt_with_cb(pkt->wIndex, einfo->on_commanded_halt);
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}

/**
 * \brief Handles a control transfer.
 *
 * This function must be invoked after the setup stage finishes. It dispatches
 * the control transfer to whichever handler is appropriate and runs the data
 * and status stages before returning. It also returns if a device state change
 * occurs or another SETUP transaction arrives.
 */
void uep0_run_transfer(void)
{
    // Setup transaction complete.
    if (OTG_FS.DIEPCTL0.EPENA || OTG_FS.DOEPCTL0.EPENA)
    {
        // We hit the tiny endpoint-disabling race condition described in the
        // abort branch of uep0_data_read. We have four options:
        // 1. Try to run this transfer.
        // 2. NAK the transfer forever.
        // 3. Issue protocol stall for this transfer.
        // 4. Set functional stall (endpoint halt) for endpoint zero.
        // Option 1 is impossible: with the endpoint enabled as it is, we can’t
        // possibly avoid letting old stuff bleed over into this transfer.
        //
        // Option 2 is possible but seems potentially problematic since it may
        // hang up host software for a long time before it times out.
        //
        // Option 3 doesn’t make a lot of sense because the request itself is
        // probably inherently acceptable, and to issue protocol stall would be
        // to claim otherwise.
        //
        // Option 4 is all we have left, so do it.
        uep0_halted           = true;
        OTG_FS.DOEPCTL0.STALL = 1;
        OTG_FS.DIEPCTL0.STALL = 1;
    }
    else
    {
        // Grab the SETUP packet.
        uep0_setup_packet = &uep0_setup_packets[__atomic_fetch_xor(
            &uep0_setup_packet_wptr, 1U, __ATOMIC_RELAXED)];
        __atomic_signal_fence(
            __ATOMIC_ACQUIRE);  // Prevent operations from being hoisted above the
                                // udev_setup_packet_wptr fetch and XOR.

        bool handled;
        if (uep0_halted)
        {
            // Endpoint is halted. When endpoint zero is halted, only ClearFeature,
            // SetFeature, and GetStatus requests are acceptable. No user handlers
            // are invoked, only the default handler.
            handled = uep0_default_handler(uep0_setup_packet);
        }
        else
        {
            // Go through all available handlers and see who wants this transfer.
            handled                   = false;
            usb_recipient_t recipient = uep0_setup_packet->bmRequestType.recipient;
            if ((recipient == USB_RECIPIENT_INTERFACE ||
                 recipient == USB_RECIPIENT_ENDPOINT) &&
                uep0_current_configuration)
            {
                unsigned int interface = UINT_MAX;
                if (recipient == USB_RECIPIENT_INTERFACE)
                {
                    interface = uep0_setup_packet->wIndex;
                }
                else if (recipient == USB_RECIPIENT_ENDPOINT &&
                         UEP_NUM(uep0_setup_packet->wIndex) <= UEP_MAX_ENDPOINT)
                {
                    interface = uep_eps[UEP_IDX(uep0_setup_packet->wIndex)].interface;
                }
                if (interface < uep0_current_configuration->descriptors->bNumInterfaces)
                {
                    const udev_interface_info_t *iinfo =
                        uep0_current_configuration->interfaces[interface];
                    if (iinfo)
                    {
                        const udev_alternate_setting_info_t *asinfo =
                            &iinfo
                                 ->alternate_settings[uep0_alternate_settings[interface]];
                        handled = handled || (asinfo->control_handler &&
                                              asinfo->control_handler(uep0_setup_packet));
                        handled = handled || (iinfo->control_handler &&
                                              iinfo->control_handler(uep0_setup_packet));
                    }
                }
            }
            handled = handled ||
                      (uep0_current_configuration &&
                       uep0_current_configuration->control_handler &&
                       uep0_current_configuration->control_handler(uep0_setup_packet));
            handled = handled || (udev_info->control_handler &&
                                  udev_info->control_handler(uep0_setup_packet));
            handled = handled || uep0_default_handler(uep0_setup_packet);
        }

        if (__atomic_load_n(&udev_state_change, __ATOMIC_RELAXED) ==
                UDEV_STATE_CHANGE_NONE &&
            __atomic_load_n(&uep0_setup_event, __ATOMIC_RELAXED) == UEP0_SETUP_EVENT_NONE)
        {
            if (handled)
            {
                // The request was accepted and the request handler will have
                // handled any necessary data stage.
                uep0_status_stage();
            }
            else
            {
                // Stall the endpoints and wait for another SETUP packet.
                OTG_FS.DOEPCTL0.STALL = 1;
                OTG_FS.DIEPCTL0.STALL = 1;
            }
        }

        uep0_setup_packet = 0;
    }
}

/**
 * \brief Handles a detach, reset, or cable unplug event.
 */
void uep0_handle_reset(void)
{
    uep0_exit_configuration();
    uep0_halted = false;
}
/**
 * \endcond
 */

/**
 * \}
 */
