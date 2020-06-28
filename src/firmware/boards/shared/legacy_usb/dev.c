/**
 * \defgroup UDEV Device handling functions
 *
 * \brief These functions handle device-wide state and configuration.
 *
 * An application begins by calling \ref udev_init.
 * This initializes the USB stack and registers the application’s configuration tables.
 * Once the stack is initialized, \ref udev_attach must be called to attach the D+ pull-up
 * resistor and enumerate on the bus to the host.
 *
 * If the application wishes to detach from the bus to simulate a cable unplug without
 * actually unplugging, \ref udev_detach can be called. Once the device is detached, it
 * can be reattached with \ref udev_attach. The device’s configuration tables can also be
 * changed by calling \ref udev_init while detached (it must not be called while
 * attached).
 *
 * Finally, the function \ref udev_isr must be registered in entry 67 of the hardware
 * interrupt vector table to handle OTG_FS interrupts.
 *
 * @{
 */
#include <FreeRTOS.h>
#include <assert.h>
#include <exception.h>
#include <limits.h>
#include <minmax.h>
#include <nvic.h>
#include <rcc.h>
#include <registers/otg_fs.h>
#include <semphr.h>
#include <stdbool.h>
#include <string.h>
#include <task.h>
#include <unused.h>
#include <usb.h>

#include "internal.h"

/**
 * \cond INTERNAL
 */

/**
 * \brief The task handle of the stack internal task.
 */
static TaskHandle_t udev_task_handle;

/**
 * \brief The current state of the device.
 *
 * This field is only updated by the stack internal task.
 */
static udev_state_t udev_state = UDEV_STATE_UNINITIALIZED;

/**
 * \brief The most recent change of state that occurred.
 */
udev_state_change_t udev_state_change = UDEV_STATE_CHANGE_NONE;

/**
 * \brief The semaphore used to notify an application task that a requested
 * attach or detach operation is complete.
 */
static SemaphoreHandle_t udev_attach_detach_sem;

/**
 * \brief Whether an attach or detach operation is requested to give \ref
 * udev_attach_detach_sem.
 */
static bool udev_attach_detach_sem_needed;

/**
 * \brief The registered configuration table.
 */
const udev_info_t *udev_info;

/**
 * \brief A mutex that must be held by any task that wishes to take global OUT
 * NAK.
 */
SemaphoreHandle_t udev_gonak_mutex = 0;

/**
 * \brief The endpoint number of the OUT endpoint that should be disabled when
 * global OUT NAK status becomes effective.
 */
unsigned int udev_gonak_disable_ep;

_Static_assert(
    INCLUDE_vTaskSuspend == 1,
    "vTaskSuspend must be included, because otherwise mutex taking can time out!");

/**
 * \endcond
 */



/**
 * \cond INTERNAL
 * \brief Handles \ref UDEV_STATE_DETACHED.
 *
 * This function waits until \ref udev_attach is called by another task, then
 * updates the current state and returns.
 */
static void udev_task_detached(void)
{
    while (udev_state == UDEV_STATE_DETACHED)
    {
        switch (__atomic_exchange_n(&udev_state_change, UDEV_STATE_CHANGE_NONE,
                                    __ATOMIC_RELAXED))
        {
            case UDEV_STATE_CHANGE_NONE:
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                break;

            case UDEV_STATE_CHANGE_ATTACH:
                udev_state = UDEV_STATE_ATTACHED;
                break;

            default:
                abort();
        }
    }
}

/**
 * \cond INTERNAL
 * \brief Handles \ref UDEV_STATE_ATTACHED.
 *
 * This function waits until \ref udev_detach is called before returning. While
 * waiting, it also handles session end and reset signalling interrupts as well
 * as traffic on endpoint zero.
 */
static void udev_task_attached(void)
{
    // Configure endpoint zero maximum packet size.
    {
        OTG_FS_DIEPCTL0_t ctl0 = {.USBAEP = 1};
        switch (udev_info->device_descriptor.bMaxPacketSize0)
        {
            case 8:
                ctl0.MPSIZ = 3;
                break;
            case 16:
                ctl0.MPSIZ = 2;
                break;
            case 32:
                ctl0.MPSIZ = 1;
                break;
            case 64:
                ctl0.MPSIZ = 0;
                break;
            default:
                abort();
        }
        OTG_FS.DIEPCTL0 = ctl0;
    }
    {
        OTG_FS_DOEPCTL0_t ctl0 = {.USBAEP = 1};
        OTG_FS.DOEPCTL0        = ctl0;
    }

    // Set local NAK on all OUT endpoints.
    {
        OTG_FS_DOEPCTL0_t ctl0 = {.SNAK = 1};
        OTG_FS.DOEPCTL0        = ctl0;
    }
    for (unsigned int i = 0U; i < UEP_MAX_ENDPOINT; ++i)
    {
        OTG_FS_DOEPCTLx_t ctl  = {.SNAK = 1};
        OTG_FS.DOEP[i].DOEPCTL = ctl;
    }

    // Enable interrupts.
    {
        OTG_FS_GINTMSK_t gintmsk = {
            .OEPINT = 1, .IEPINT = 1, .USBRST = 1, .RXFLVLM = 1, .OTGINT = 1};
        OTG_FS.GINTMSK = gintmsk;
    }

    // Wait for detach.
    while (udev_state == UDEV_STATE_ATTACHED)
    {
        udev_state_change_t state_change = __atomic_exchange_n(
            &udev_state_change, UDEV_STATE_CHANGE_NONE, __ATOMIC_RELAXED);
        switch (state_change)
        {
            case UDEV_STATE_CHANGE_NONE:
                switch (__atomic_exchange_n(&uep0_setup_event, UEP0_SETUP_EVENT_NONE,
                                            __ATOMIC_RELAXED))
                {
                    case UEP0_SETUP_EVENT_NONE:
                        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                        break;

                    case UEP0_SETUP_EVENT_STARTED:
                        // Ignore this event; wait for setup stage to finish.
                        break;

                    case UEP0_SETUP_EVENT_FINISHED:
                        // Run the control transfer.
                        uep0_run_transfer();
                        break;
                }
                break;

            case UDEV_STATE_CHANGE_DETACH:
                udev_state = UDEV_STATE_DETACHED;
                uep0_handle_reset();
                break;

            case UDEV_STATE_CHANGE_SEDET:
            case UDEV_STATE_CHANGE_RESET:
                // Exit any configuration we might previously have been in.
                uep0_handle_reset();

                // Initialize and flush receive and endpoint zero transmit FIFOs.
                assert(udev_info->receive_fifo_words >= 16U);
                {
                    OTG_FS_GRXFSIZ_t rxfsiz = {.RXFD = udev_info->receive_fifo_words};
                    OTG_FS.GRXFSIZ          = rxfsiz;
                }
                {
                    // See uep0_data_write for why these values.
                    OTG_FS_DIEPTXF0_t txf0 = {.TX0FSA = udev_info->receive_fifo_words};
                    switch (udev_info->device_descriptor.bMaxPacketSize0)
                    {
                        case 8U:
                            txf0.TX0FD = 16U;
                            break;
                        case 16U:
                            txf0.TX0FD = 16U;
                            break;
                        case 32U:
                            txf0.TX0FD = 24U;
                            break;
                        case 64U:
                            txf0.TX0FD = 16U;
                            break;
                    }
                    OTG_FS.DIEPTXF0 = txf0;
                }
                udev_flush_rx_fifo();
                udev_flush_tx_fifo(0x80U);

                // Set local NAK on all OUT endpoints.
                {
                    OTG_FS_DOEPCTL0_t ctl0 = {.SNAK = 1};
                    OTG_FS.DOEPCTL0        = ctl0;
                }
                for (unsigned int i = 0U; i < UEP_MAX_ENDPOINT; ++i)
                {
                    OTG_FS_DOEPCTLx_t ctl  = {.SNAK = 1};
                    OTG_FS.DOEP[i].DOEPCTL = ctl;
                }
                break;

            default:
                abort();
        }
    }

    // Disable interrupts.
    {
        OTG_FS_GINTMSK_t gintmsk = {0};
        OTG_FS.GINTMSK           = gintmsk;
    }
}

/**
 * \cond INTERNAL
 * \brief The stack internal task.
 */
static void udev_task(void *UNUSED(param))
{
    // Sanity check.
    assert(udev_state == UDEV_STATE_DETACHED);

    // The task is created on udev_init, but the device should not attach to
    // the bus until udev_attach. There may be a race condition between
    // initializing the engine and ensuring SDIS is set to not attach to the
    // bus. So, instead, we just wait here and don’t initialize the engine at
    // all until we’re ready to attach for the first time.
    udev_task_detached();

    // Reset the module and enable the clock
    rcc_enable_reset(AHB2, OTGFS);

    // Reset the USB core and configure device-wide parameters.
    {
        OTG_FS_GUSBCFG_t tmp = {
            .CTXPKT = 0,  // This bit should never be set.
            .FDMOD  = 1,  // Force to device mode (no cable ID used).
            .FHMOD  = 0,  // Do not force to host mode.
            .TRDT   = 6,  // Turnaround time 6 PHY clocks.
            .HNPCAP = 0,  // Not host-negotiation-protocol capable.
            .SRPCAP = 0,  // Not session-request-protocol capable.
            .PHYSEL = 1,  // This bit is always set.
            .TOCAL  = 0,  // Do not add additional bit times to interpacket timeout (the
                         // STMicro library leaves this at zero; I assume this is fine for
                         // the on-chip PHY).
        };
        OTG_FS.GUSBCFG = tmp;
    }
    while (!OTG_FS.GRSTCTL.AHBIDL)
        ;                      // Wait until AHB is idle.
    OTG_FS.GRSTCTL.CSRST = 1;  // Core soft reset.
    while (OTG_FS.GRSTCTL.CSRST)
        ;  // Wait for reset to be complete.
    // Wait:
    // - at least 3 PHY clocks (would be 62.5 ns) after CSRST for the PHY to be ready
    // - at least 25 milliseconds for a change to FDMOD to take effect
    vTaskDelay((25U + portTICK_PERIOD_MS - 1U) / portTICK_PERIOD_MS);
    while (!OTG_FS.GRSTCTL.AHBIDL)
        ;  // Wait until AHB is idle.
    {
        OTG_FS_GCCFG_t tmp = {
            .NOVBUSSENS = udev_info->flags.vbus_sensing
                              ? 0
                              : 1,  // VBUS sensing may or may not be done.
            .SOFOUTEN = 0,          // Do not output SOF pulses to I/O pin.
            .VBUSBSEN = 1,          // VBUS sensing B device enabled.
            .VBUSASEN = 0,          // VBUS sensing A device disabled.
            .PWRDWN   = 1,          // Transceiver active.
        };
        OTG_FS.GCCFG = tmp;
    }
    {
        OTG_FS_GAHBCFG_t gahbcfg = {
            .PTXFELVL = 0,  // Only used in host mode.
            .TXFELVL  = udev_info->flags.minimize_interrupts
                           ? 1
                           : 0,  // Interrupt on TX FIFO half empty or fully empty.
            .GINTMSK = 1,        // Enable interrupts.
        };
        OTG_FS.GAHBCFG = gahbcfg;
    }
    {
        OTG_FS_DCFG_t tmp = OTG_FS.DCFG;  // This register may have nonzero reserved bits
                                          // that must be maintained.
        tmp.DAD = 0,                      // Address is zero on initial attach.
            tmp.NZLSOHSK =
                1,  // Send STALL on receipt of non-zero-length status transaction.
            tmp.DSPD    = 3,  // Run at full speed.
            OTG_FS.DCFG = tmp;
    }
    {
        OTG_FS_GOTGCTL_t tmp = {
            .DHNPEN  = 0,  // Host negotiation protocol disabled.
            .HSHNPEN = 0,  // Host negotiation protocol has not been enabled on the peer.
            .HNPRQ   = 0,  // Do not issue host negotiation protocol request.
            .SRQ     = 0,  // Do not issue session request.
        };
        OTG_FS.GOTGCTL = tmp;
    }

    // Enable applicable interrupts.
    {
        OTG_FS_DAINTMSK_t tmp = {.IEPM = (1U << (UEP_MAX_ENDPOINT + 1U)) - 1U,
                                 .OEPM = (1U << (UEP_MAX_ENDPOINT + 1U)) - 1U};
        OTG_FS.DAINTMSK       = tmp;
    }
    {
        OTG_FS_DIEPMSK_t tmp = {
            .INEPNEM = 1,
            .EPDM    = 1,
            .XFRCM   = 1,
        };
        OTG_FS.DIEPMSK = tmp;
    }
    {
        OTG_FS_DOEPMSK_t tmp = {
            .EPDM = 1,
        };
        OTG_FS.DOEPMSK = tmp;
    }

    // Globally enable USB interrupts.
    portENABLE_HW_INTERRUPT(NVIC_IRQ_OTG_FS);

    // Run the state machine.
    for (;;)
    {
        switch (udev_state)
        {
            case UDEV_STATE_DETACHED:
                // Acknowledge whoever called udev_detach.
                xSemaphoreGive(udev_attach_detach_sem);
                udev_task_detached();
                break;

            case UDEV_STATE_ATTACHED:
                // Acknowledge whoever called udev_attach.
                xSemaphoreGive(udev_attach_detach_sem);
                udev_task_attached();
                break;

            default:
                abort();
        }
    }
}

/**
 * \brief Flushes the receive FIFO.
 */
void udev_flush_rx_fifo(void)
{
    taskENTER_CRITICAL();
    while (!OTG_FS.GRSTCTL.AHBIDL)
        ;
    OTG_FS_GRSTCTL_t ctl = {.RXFFLSH = 1U};
    OTG_FS.GRSTCTL       = ctl;
    while (OTG_FS.GRSTCTL.RXFFLSH)
        ;
    while (!OTG_FS.GRSTCTL.AHBIDL)
        ;
    taskEXIT_CRITICAL();
}

/**
 * \brief Flushes a transmit FIFO.
 *
 * \param[in] ep the endpoint address of the endpoint whose FIFO to flush
 */
void udev_flush_tx_fifo(unsigned int ep)
{
    assert(UEP_DIR(ep));
    assert(UEP_NUM(ep) <= UEP_MAX_ENDPOINT);
    taskENTER_CRITICAL();
    while (!OTG_FS.GRSTCTL.AHBIDL)
        ;
    OTG_FS_GRSTCTL_t ctl = {.TXFNUM = UEP_NUM(ep), .TXFFLSH = 1U};
    OTG_FS.GRSTCTL       = ctl;
    while (OTG_FS.GRSTCTL.TXFFLSH)
        ;
    while (!OTG_FS.GRSTCTL.AHBIDL)
        ;
    taskEXIT_CRITICAL();
}

/**
 * \brief Extracts a packet from the receive FIFO.
 *
 * \param[out] dest the memory location to copy the packet to
 * \param[in] bytes the number of bytes to copy out
 * \param[in] extra the number of bytes left in the FIFO after the packet is
 * copied out, due to overflow, which are extracted and discarded
 */
static void udev_rx_copy_out(void *dest, size_t bytes, size_t extra)
{
    uint8_t *pc        = dest;
    size_t whole_words = bytes / 4U;
    while (whole_words--)
    {
        uint32_t word = OTG_FS_FIFO[0U][0U];
        memcpy(pc, &word, sizeof(word));
        pc += sizeof(word);
    }
    bytes %= 4U;
    if (bytes)
    {
        extra -= MIN(extra, 4U - bytes);
        uint32_t word = OTG_FS_FIFO[0U][0U];
        while (bytes--)
        {
            *pc++ = (uint8_t)word;
            word >>= 8U;
        }
    }
    extra = (extra + 3U) / 4U;
    while (extra--)
    {
        (void)OTG_FS_FIFO[0U][0U];
    }
}

/**
 * \brief Writes a packet to a transmit FIFO.
 *
 * \param[out] dest the FIFO to store into
 *
 * \param[in] source the packet to copy in
 *
 * \param[in] bytes the length of the packet in bytes
 */
void udev_tx_copy_in(volatile void *dest, const void *source, size_t bytes)
{
    volatile uint32_t *dptr = dest;
    const uint8_t *pc       = source;
    size_t whole_words      = bytes / 4U;
    while (whole_words--)
    {
        uint32_t word;
        memcpy(&word, pc, sizeof(word));
        *dptr = word;
        pc += sizeof(word);
    }
    bytes %= 4U;
    if (bytes)
    {
        uint32_t word = 0U;
        for (size_t i = 0; i < bytes; ++i)
        {
            word |= ((uint32_t)pc[i]) << (i * 8U);
        }
        *dptr = word;
    }
}
/**
 * \endcond
 */

/**
 * \brief Handles USB interrupts.
 *
 * This function should be registered in the application’s interrupt vector
 * table at position 67.
 *
 * \internal
 *
 * The general philosophy for handling interrupts in the USB stack is to do the
 * minimum amount of work in the ISR that is sensible, but no less. This leads
 * to the following policy:
 * \li For receive FIFO non-empty interrupts, an element is popped from the
 * FIFO and examined; received packets are copied into their final locations,
 * while other events are marked in event groups for later consideration by
 * tasks.
 * \li For transmit FIFO empty interrupts, data to send is copied from the
 * source buffer into the FIFO, and once the transfer is fully satisfied, the
 * FIFO empty interrupt is masked.
 * \li For state-change interrupts (such as reset, session end, and session
 * request), \ref udev_state is updated immediately and an event is queued for
 * the stack internal task.
 * \li For most other interrupt sources, the interrupt is acknowledged and an
 * event is queued for later consideration by a task.
 *
 * This policy means that, during normal operation, an entire physical transfer
 * will complete entirely under the ISR’s control before the initiating task is
 * finally woken through its event queue to take a transfer complete event.
 * This is a middle ground between ultimate efficiency (wherein the ISR does
 * everything, including decomposing logical transfers into multiple physical
 * transfers and dealing with zero-length packets) and ultimate minimalism
 * (wherein the ISR only ever queues events, and a task must awaken to read or
 * write every single data packet). It also means that the tasks become more
 * closely involved in unusual situations, such as uenxpected disabling of an
 * endpoint; this is acceptable because these unusual situations are not in
 * performance-critical paths and introduce a lot of complexity which does not
 * belong in an ISR.
 */
void udev_isr(void)
{
    OTG_FS_GINTMSK_t msk  = OTG_FS.GINTMSK;
    OTG_FS_GINTSTS_t sts  = OTG_FS.GINTSTS;
    OTG_FS_GOTGINT_t otg  = OTG_FS.GOTGINT;
    bool give_udev_notify = false;
    BaseType_t yield      = pdFALSE;

    // Handle OUT endpoints.
    if (msk.RXFLVLM && sts.RXFLVL)
    {
        do
        {
            OTG_FS_GRXSTSR_device_t elt = OTG_FS.GRXSTSP.device;
            switch (elt.PKTSTS)
            {
                case 0b0001:
                    // Global OUT NAK effective. GONAK is only ever requested
                    // by the endpoint module when it wants to asynchronously
                    // disable an OUT endpoint during a transfer. It stores the
                    // endpoint address in udev_gonak_disable_ep before calling
                    // for GONAK.
                    {
                        OTG_FS_DOEPCTLx_t ctl =
                            OTG_FS.DOEP[UEP_NUM(udev_gonak_disable_ep) - 1U].DOEPCTL;
                        if (ctl.EPENA)
                        {
                            // Endpoint is currently enabled. There is no
                            // longer any race against it being disabled by
                            // XFRC: because GONAKEFF, no further transactions
                            // can arrive and cause an XFRC that has not
                            // already happened and been observed; the GONAKEFF
                            // acts as a fence in GRXSTSP. Start disabling the
                            // endpoint now. Keep GONAK until the endpoint
                            // finishes disabling.
                            ctl.EPENA = 0;
                            ctl.EPDIS = 1;
                            ctl.SNAK  = 1;
                            OTG_FS.DOEP[UEP_NUM(udev_gonak_disable_ep) - 1U].DOEPCTL =
                                ctl;
                        }
                        else
                        {
                            // Endpoint had already disabled itself by the time
                            // we got here due to XFRC. Notify whoever was
                            // waiting for the endpoint to disable that it is
                            // indeed disabled, then release GONAK as we do not
                            // need it.
                            BaseType_t ok = xSemaphoreGiveFromISR(
                                uep_eps[UEP_IDX(udev_gonak_disable_ep)].disabled_sem,
                                &yield);
                            assert(ok == pdTRUE);
                            OTG_FS.DCTL.CGONAK = 1;
                        }
                    }
                    break;

                case 0b0010:
                    // OUT data packet received. Copy data into target buffer;
                    // do not queue any events.
                    if (elt.EPNUM)
                    {
                        uep_ep_t *ctx = &uep_eps[UEP_IDX(0x00 | elt.EPNUM)];
                        if (elt.BCNT)
                        {
                            size_t to_copy = MIN(elt.BCNT, ctx->bytes_left);
                            udev_rx_copy_out(ctx->transfer.out.wptr, to_copy,
                                             elt.BCNT - to_copy);
                            ctx->transfer.out.wptr += to_copy;
                            ctx->bytes_left -= to_copy;
                        }
                        else
                        {
                            ctx->flags.zlp = 1;
                        }
                    }
                    else
                    {
                        // No need to report ZLPs because OUT endpoint 0 should
                        // never see a ZLP (except in the status stage, which
                        // need not be reported). For data stages, the host
                        // should indicate in wLength exactly how much it will
                        // send and then send it.
                        size_t to_copy = MIN(elt.BCNT, uep0_out_length);
                        udev_rx_copy_out(uep0_out_wptr, to_copy, elt.BCNT - to_copy);
                        uep0_out_length = elt.BCNT;
                    }
                    break;

                case 0b0011:
                    // OUT transfer completed.
                    if (elt.EPNUM)
                    {
                        uep_queue_event_from_isr(0x00 | elt.EPNUM, UEP_EVENT_XFRC,
                                                 &yield);
                    }
                    else
                    {
                        uep0_xfrc |= 1;
                        give_udev_notify = true;
                    }
                    break;

                case 0b0100:
                    // SETUP transaction completed.
                    uep0_setup_event = UEP0_SETUP_EVENT_FINISHED;
                    give_udev_notify = true;
                    break;

                case 0b0110:
                    // SETUP packet received.
                    // Copy data into target buffer; do not set any events.
                    assert(elt.EPNUM == 0);
                    assert(elt.BCNT == sizeof(usb_setup_packet_t));
                    udev_rx_copy_out(uep0_writable_setup_packet(),
                                     sizeof(usb_setup_packet_t), 0);
                    uep0_setup_event = UEP0_SETUP_EVENT_STARTED;
                    give_udev_notify = true;
                    break;

                default:
                    abort();
            }
        } while (OTG_FS.GINTSTS.RXFLVL);
    }
    if (msk.OEPINT && sts.OEPINT)
    {
        unsigned int epmsk = OTG_FS.DAINT.OEPINT;
        // Endpoint zero never exhibits EPDISD because it can never be disabled
        // under application request.
        for (unsigned int ep = 1U; ep <= UEP_MAX_ENDPOINT; ++ep)
        {
            if (epmsk & (1U << ep))
            {
                OTG_FS_DOEPINTx_t doepint = OTG_FS.DOEP[ep - 1U].DOEPINT;
                if (doepint.EPDISD)
                {
                    // EPDISD interrupt occurs only when we set EPDIS=1
                    // explicitly. This occurs only when we are asynchronously
                    // disabling an endpoint under GONAK. Clear the interrupt,
                    // report the situation, and release GONAK.
                    OTG_FS_DOEPINTx_t tmp        = {.EPDISD = 1};
                    OTG_FS.DOEP[ep - 1U].DOEPINT = tmp;
                    BaseType_t ok                = xSemaphoreGiveFromISR(
                        uep_eps[UEP_IDX(0x00 | ep)].disabled_sem, &yield);
                    assert(ok == pdTRUE);
                    uep_queue_event_from_isr(0x00 | ep, UEP_EVENT_EPDISD, &yield);
                    OTG_FS.DCTL.CGONAK = 1;
                }
            }
        }
    }

    // Handle IN endpoints.
    if (msk.IEPINT && sts.IEPINT)
    {
        unsigned int epint = OTG_FS.DAINT.IEPINT;
        if (epint & 1U)
        {
            OTG_FS_DIEPINTx_t diepint = OTG_FS.DIEPINT0;
            OTG_FS.DIEPINT0           = diepint;
            if (diepint.XFRC)
            {
                uep0_xfrc |= 2;
                give_udev_notify = true;
            }
        }
        uint32_t empmsk = OTG_FS.DIEPEMPMSK;
        for (unsigned int ep = 1U; ep <= UEP_MAX_ENDPOINT; ++ep)
        {
            if (epint & (1U << ep))
            {
                OTG_FS_DIEPINTx_t diepint = OTG_FS.DIEP[ep - 1U].DIEPINT;
                uep_ep_t *ctx             = &uep_eps[UEP_IDX(0x80 | ep)];

                // Only push packets to the FIFO if we actually should; we
                // might get in here for a reason other than empty FIFO, and if
                // empty FIFO is not an interrupt source for this endpoint, it
                // means somebody doesn’t want us pushing packets, so don’t.
                if (empmsk & (1U << ep))
                {
                    // Shovel a packet at a time until either:
                    // (1) there is not enough space in the FIFO for another
                    //     packet, or
                    // (2) the physical transfer is finished
                    size_t ep_max_packet       = OTG_FS.DIEP[ep - 1U].DIEPCTL.MPSIZ;
                    size_t ep_max_packet_words = (ep_max_packet + 3U) / 4U;
                    while (OTG_FS.DIEP[ep - 1U].DIEPTSIZ.XFRSIZ &&
                           OTG_FS.DIEP[ep - 1U].DTXFSTS.INEPTFSAV >= ep_max_packet_words)
                    {
                        size_t this_packet_bytes = MIN(ep_max_packet, ctx->bytes_left);
                        udev_tx_copy_in(&OTG_FS_FIFO[ep][0U], ctx->transfer.in.rptr,
                                        this_packet_bytes);
                        ctx->bytes_left -= this_packet_bytes;
                        ctx->transfer.in.rptr += this_packet_bytes;
                    }

                    // If we are out of data to transfer, stop taking FIFO
                    // empty interrupts.
                    if (!OTG_FS.DIEP[ep - 1U].DIEPTSIZ.XFRSIZ)
                    {
                        empmsk &= ~(1U << ep);
                    }
                }

                // Check for other interrupts on the endpoint.
                OTG_FS_DIEPINTx_t to_clear = {0};
                if (diepint.XFRC)
                {
                    to_clear.XFRC = 1;
                    uep_queue_event_from_isr(0x80 | ep, UEP_EVENT_XFRC, &yield);
                }
                if (diepint.INEPNE)
                {
                    to_clear.INEPNE = 1;
                    // This occurs only when a task wishes to asynchronously
                    // disable the endpoint. It initiates the asynchronous
                    // disable by setting SNAK=1. This causes INEPNE regardless
                    // of the prior value of EPENA or NAKSTS, bringing us here.
                    // Proceed with disabling the endpoint.
                    OTG_FS_DIEPCTLx_t ctl = OTG_FS.DIEP[ep - 1U].DIEPCTL;
                    if (ctl.EPENA)
                    {
                        // Endpoint is currently enabled. There is no longer
                        // any race against it being disabled by XFRC: because
                        // NAKSTS (implied by INEPNE), no further transactions
                        // can arrive and cause an XFRC that has not already
                        // been observed; the NAKSTS acts as a fence. Start
                        // disabling the endpoint now.
                        ctl.EPENA                    = 0;
                        ctl.EPDIS                    = 1;
                        OTG_FS.DIEP[ep - 1U].DIEPCTL = ctl;
                    }
                    else
                    {
                        // Endpoint had already disabled itself by the time we
                        // got here due to XFRC. Notify whoever was waiting for
                        // the endpoint to disable that it has indeed disabled.
                        BaseType_t ok = xSemaphoreGiveFromISR(ctx->disabled_sem, &yield);
                        assert(ok == pdTRUE);
                    }
                }
                if (diepint.EPDISD)
                {
                    to_clear.EPDISD = 1;
                    // EPDISD interrupt occurs only when we set EPDIS=1
                    // explicitly. This occurs only when we are asynchronously
                    // disabling the endpoint under NAKSTS, started just above
                    // in the INEPNE branch. Report the situation.
                    BaseType_t ok = xSemaphoreGiveFromISR(ctx->disabled_sem, &yield);
                    assert(ok == pdTRUE);
                    uep_queue_event_from_isr(0x80 | ep, UEP_EVENT_EPDISD, &yield);
                }
                OTG_FS.DIEP[ep - 1U].DIEPINT = to_clear;
            }
        }
        OTG_FS.DIEPEMPMSK = empmsk;
    }

    // Handle global, non-endpoint-related activity.
    if (msk.OTGINT && sts.OTGINT)
    {
        if (otg.SEDET)
        {
            // This interrupt arrives when VBUS falls.
            OTG_FS_GOTGINT_t tmp = {.SEDET = 1};
            OTG_FS.GOTGINT       = tmp;
            // We could be in the process of detaching when the reset event
            // occurs; in such a case, leave udev_state_change set to
            // UDEV_STATE_CHANGE_DETACH as, in the face of that action, this
            // event is uninteresting.
            if (udev_state_change == UDEV_STATE_CHANGE_NONE)
            {
                udev_state_change = UDEV_STATE_CHANGE_SEDET;
                give_udev_notify  = true;
            }
            // Ensure a SETUP packet received before the reset won’t get
            // handled after it.
            uep0_setup_event = UEP0_SETUP_EVENT_NONE;
        }
        else
        {
            abort();
        }
    }
    if (msk.USBRST && sts.USBRST)
    {
        // This interrupt arrives when USB reset signalling starts.
        OTG_FS_GINTSTS_t tmp = {.USBRST = 1};
        OTG_FS.GINTSTS       = tmp;
        // We could be in the process of detaching when the reset event occurs;
        // in such a case, leave udev_state_change set to
        // UDEV_STATE_CHANGE_DETACH as, in the face of that action, this event
        // is uninteresting.
        if (udev_state_change == UDEV_STATE_CHANGE_NONE)
        {
            udev_state_change = UDEV_STATE_CHANGE_RESET;
            give_udev_notify  = true;
        }
        // Reset signalling requires the device to respond to address zero.
        OTG_FS_DCFG_t dcfg = {.DAD = 0, .NZLSOHSK = 1, .DSPD = 3};
        OTG_FS.DCFG        = dcfg;
        // Ensure a SETUP packet received before the reset won’t get handled
        // after it.
        uep0_setup_event = UEP0_SETUP_EVENT_NONE;
    }

    if (give_udev_notify)
    {
        vTaskNotifyGiveFromISR(udev_task_handle, &yield);
    }

    if (yield)
    {
        portYIELD_FROM_ISR();
    }

    EXCEPTION_RETURN_BARRIER();
}

/**
 * \brief Initializes the USB subsystem.
 *
 * This must be the first USB stack function the application calls.
 *
 * This function can safely be called before the FreeRTOS scheduler is started.
 *
 * \param[in] info the device configuration table, which must remain valid for the entire
 * time the subsystem is enabled
 *
 * \pre The subsystem must be uninitialized or the device must be detached from the bus.
 *
 * \post The USB subsystem is initialized with the device soft-detached from the bus.
 */
void udev_init(const udev_info_t *info)
{
    assert(info);

    switch (__atomic_load_n(&udev_state, __ATOMIC_RELAXED))
    {
        case UDEV_STATE_UNINITIALIZED:
            // Initialize the stack.
            ;
            static uint8_t ep_event_queue_buffer[UEP_MAX_ENDPOINT * 2];
            static StaticSemaphore_t attach_detach_sem_storage, gonak_mutex_storage,
                ep_mutex_storage[UEP_MAX_ENDPOINT * 2],
                ep_disabled_sem_storage[UEP_MAX_ENDPOINT * 2];
            static StaticQueue_t ep_event_queue_storage[UEP_MAX_ENDPOINT * 2];
            udev_info  = info;
            udev_state = UDEV_STATE_DETACHED;
            udev_attach_detach_sem =
                xSemaphoreCreateBinaryStatic(&attach_detach_sem_storage);
            udev_gonak_mutex = xSemaphoreCreateMutexStatic(&gonak_mutex_storage);
            for (unsigned int i = 0; i < UEP_MAX_ENDPOINT * 2U; ++i)
            {
                uep_eps[i].mutex = xSemaphoreCreateMutexStatic(&ep_mutex_storage[i]);
                uep_eps[i].state = UEP_STATE_INACTIVE;
                uep_eps[i].event_queue = xQueueCreateStatic(
                    1U, 1U, &ep_event_queue_buffer[i], &ep_event_queue_storage[i]);
                uep_eps[i].disabled_sem =
                    xSemaphoreCreateBinaryStatic(&ep_disabled_sem_storage[i]);
                uep_eps[i].flags.zlp      = false;
                uep_eps[i].flags.overflow = false;
                uep_eps[i].interface      = UINT_MAX;
                if (i < UEP_MAX_ENDPOINT)
                {
                    uep_eps[i].transfer.out.buffer = 0;
                    uep_eps[i].transfer.out.wptr   = 0;
                }
                else
                {
                    uep_eps[i].transfer.in.data = 0;
                    uep_eps[i].transfer.in.rptr = 0;
                }
                uep_eps[i].bytes_left = 0U;
                uep_eps[i].async_cb   = 0;
            }
            static StaticTask_t task_storage;
            udev_task_handle = xTaskCreateStatic(
                &udev_task, "legacy_usb",
                info->internal_task_stack_size / sizeof(unsigned long), 0,
                info->internal_task_priority, info->internal_task_stack, &task_storage);
            break;

        case UDEV_STATE_DETACHED:
            // Only change the configuration table and a few global control flags.
            udev_info               = info;
            OTG_FS.GCCFG.NOVBUSSENS = info->flags.vbus_sensing ? 0U : 1U;
            OTG_FS.GAHBCFG.TXFELVL  = info->flags.minimize_interrupts ? 1U : 0U;
            break;

        default:
            abort();
    }
}

/**
 * \brief Attaches to the bus.
 *
 * \pre The subsystem must be initialized.
 *
 * \pre The device must be detached from the bus.
 *
 * \pre Neither this function nor \ref udev_detach may already be running.
 *
 * \post The device is attached to the bus and will exhibit a pull-up resistor
 * on D+ when VBUS is present (or always, if \ref udev_info_t::vbus_sensing is
 * zero).
 */
void udev_attach(void)
{
    // Physically connect to the bus.
    taskENTER_CRITICAL();
    OTG_FS.DCTL.SDIS = 0;
    taskEXIT_CRITICAL();

    // We want to be notified once the stack internal task acknowledges the
    // attachment request.
    udev_attach_detach_sem_needed = true;

    // Prevent the write to udev_attach_detach_sem_needed from being sunk below
    // this point.
    __atomic_signal_fence(__ATOMIC_RELEASE);

    // Issue the state change.
    __atomic_store_n(&udev_state_change, UDEV_STATE_CHANGE_ATTACH, __ATOMIC_RELAXED);
    xTaskNotifyGive(udev_task_handle);

    // Wait until the stack internal task acknowledges the state change.
    BaseType_t ok = xSemaphoreTake(udev_attach_detach_sem, portMAX_DELAY);
    assert(ok);
}

/**
 * \brief Detaches from the bus.
 *
 * This function blocks, only returning once the detach operation is complete.
 *
 * Note that this function will invoke the exit callbacks for the active
 * configuration and all interface alternate settings. Therefore, those
 * callbacks must be safely invokable from the point at which this function is
 * called. This must therefore not be called from a configuration or interface
 * alternate setting enter/exit callback. It must also normally not be called
 * from a task performing nonzero endpoint operations, as the exit callback
 * would deadlock waiting for the task to terminate.
 *
 * All endpoints are deactivated as a result of this call.
 *
 * \pre The subsystem must be initialized.
 *
 * \pre The device must be attached to the bus.
 *
 * \pre Neither this function nor \ref udev_attach may already be running.
 *
 * \post The device is detached from the bus and does not exhibit a pull-up
 * resistor on D+.
 *
 * \post Exit callbacks for all previously active configurations and interface
 * alternate settings have completed.
 */
void udev_detach(void)
{
    // Physically disconnect from the bus.
    taskENTER_CRITICAL();
    OTG_FS.DCTL.SDIS = 1;
    taskEXIT_CRITICAL();

    if (xTaskGetCurrentTaskHandle() == udev_task_handle)
    {
        // We are being called from the stack internal task, so we will not
        // block until detachment is acknowledged by the stack internal task—to
        // do so would deadlock. Thus, we do not need to be notified once the
        // state change takes place.
        udev_attach_detach_sem_needed = false;
    }
    else
    {
        // We are being called from some task other than the stack internal
        // task, so we will block until the stack internal task finishes
        // cleanup.
        udev_attach_detach_sem_needed = true;
    }

    // Prevent the write to udev_attach_detach_sem_needed from being sunk below
    // this point.
    __atomic_signal_fence(__ATOMIC_RELEASE);

    // Issue the state change.
    __atomic_store_n(&udev_state_change, UDEV_STATE_CHANGE_DETACH, __ATOMIC_RELAXED);
    xTaskNotifyGive(udev_task_handle);

    if (udev_attach_detach_sem_needed)
    {
        // Wait until the stack internal task acknowledges the state change.
        BaseType_t ok = xSemaphoreTake(udev_attach_detach_sem, portMAX_DELAY);
        assert(ok);
    }

    // Wait a little bit to allow the bus to settle in its detached state.
    vTaskDelay(1U);
}

/**
 * @}
 */
