#ifndef USB_USB_INTERNAL_H
#define USB_USB_INTERNAL_H

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/**
 * \cond INTERNAL
 */

/**
 * \ingroup UDEV
 * \brief The possible states the device can be in.
 */
typedef enum
{
    /**
     * \brief The subsystem has not yet been initialized.
     */
    UDEV_STATE_UNINITIALIZED,

    /**
     * \brief The device is detached from the bus and will not exhibit a D+
     * pull-up resistor.
     */
    UDEV_STATE_DETACHED,

    /**
     * \brief The device is attached to the bus. If VBUS is present, a D+
     * pull-up resistor is exhibited.
     */
    UDEV_STATE_ATTACHED,
} udev_state_t;

/**
 * \ingroup UDEV
 * \brief The possible state changes that can happen.
 */
typedef enum
{
    /**
     * \brief Indicates that no state change has happened.
     */
    UDEV_STATE_CHANGE_NONE,

    /**
     * \brief Indicates that a call was made to \ref udev_attach.
     */
    UDEV_STATE_CHANGE_ATTACH,

    /**
     * \brief Indicates that a call was made to \ref udev_detach.
     */
    UDEV_STATE_CHANGE_DETACH,

    /**
     * \brief Indicates that VBUS fell below 5 volts.
     */
    UDEV_STATE_CHANGE_SEDET,

    /**
     * \brief Indicates that reset signalling started.
     */
    UDEV_STATE_CHANGE_RESET,
} udev_state_change_t;



/**
 * \ingroup UEP0
 * \brief The possible SETUP-related events that can happen.
 */
typedef enum
{
    /**
     * \brief No SETUP-related event is pending.
     */
    UEP0_SETUP_EVENT_NONE,

    /**
     * \brief A SETUP packet has arrived from the host.
     */
    UEP0_SETUP_EVENT_STARTED,

    /**
     * \brief Since the last SETUP packet arrived, an IN or OUT token was
     * NAKed, indicating that the setup stage of the control transfer is
     * finished and the data or status stage is starting.
     */
    UEP0_SETUP_EVENT_FINISHED,
} uep0_setup_event_t;



/**
 * \ingroup UEP
 * \brief The states that a (non-zero) endpoint can be in.
 */
typedef enum
{
    /**
     * \brief The endpoint is not active in the current configuration or
     * interface alternate setting.
     */
    UEP_STATE_INACTIVE,

    /**
     * \brief The endpoint is ready to use.
     *
     * No operation is running. A task may start a transfer.
     */
    UEP_STATE_IDLE,

    /**
     * \brief A data transfer is running.
     *
     * In the case of a synchronous transfer, \ref uep_read or \ref uep_write
     * has not returned yet. In the case of an asynchronous transfer, \ref
     * uep_async_read_finish or \ref uep_async_write_finish has not been
     * invoked successfully yet.
     */
    UEP_STATE_RUNNING,

    /**
     * \brief The endpoint is halted.
     *
     * No halt-wait is running. A task may start one.
     */
    UEP_STATE_HALTED,

    /**
     * \brief The endpoint is halted and a halt-wait is running.
     *
     * In the case of a synchronous halt-wait, \ref uep_halt_wait has not
     * returned yet. In the case of an asynchronous halt-wait, \ref
     * uep_async_halt_wait_finish has not been invoked successfully yet.
     */
    UEP_STATE_HALTED_WAITING,

    /**
     * \brief The endpoint was halted and a CLEAR FEATURE request was received
     * clearing the halt feature.
     *
     * No halt-wait was running when the request was received, so the
     * application still needs to start one in order to observe the prior halt
     * state.
     */
    UEP_STATE_CLEAR_HALT_PENDING,
} uep_state_t;

/**
 * \ingroup UEP
 * \brief The event numbers that can be delivered via a nonzero endpoint’s \ref
 * uep_ep_t::event_queue "event_queue" member.
 */
typedef enum
{
    /**
     * \brief A physical transfer has finished.
     *
     * This event is queued by the ISR when the endpoint is in \ref
     * UEP_STATE_RUNNING when the physical transfer finishes.
     */
    UEP_EVENT_XFRC,

    /**
     * \brief The endpoint has been disabled during a physical transfer.
     *
     * This event is queued by the ISR when the endpoint is in \ref
     * UEP_STATE_RUNNING after the endpoint is fully disabled.
     */
    UEP_EVENT_EPDISD,

    /**
     * \brief A halted endpoint was deactivated by a configuration or interface
     * alternate setting change.
     *
     * This event is queued by the internal task when the endpoint is in \ref
     * UEP_STATE_HALTED_WAITING and is deactivated.
     */
    UEP_EVENT_DEACTIVATED_WHILE_HALTED,

    /**
     * \brief A halted endpoint had its halt status cleared.
     *
     * This event is queued by the internal task when the endpoint is in \ref
     * UEP_STATE_HALTED_WAITING and a CLEAR FEATURE request arrives which
     * successfully clears the endpoint’s halt status.
     */
    UEP_EVENT_HALT_CLEARED,
} uep_event_t;

/**
 * \ingroup UEP
 * \brief The type of data associated with a nonzero endpoint.
 */
typedef struct
{
    /**
     * \brief A mutex held by the task manipulating the endpoint.
     *
     * Not all fields in this structure are protected by the mutex. See details
     * of individual fields.
     */
    SemaphoreHandle_t mutex;

    /**
     * \brief The endpoint’s current state.
     *
     * This field is protected by the mutex.
     */
    uep_state_t state;

    /**
     * \brief The endpoint’s event queue.
     *
     * The queue carries single bytes. Each byte is a member of the \ref
     * uep_event_t enumeration representing the type of event that occurred.
     * Events are sent from a variety of sources and always received by the
     * task that is running a transfer or a halt-wait on the endpoint.
     *
     * A task pushing to the queue must hold the mutex. A task blocking on the
     * queue to pop must not hold the mutex.
     *
     * This queue is only one element long. This is adequate, because events
     * are always used to notify the task currently running a transfer, and
     * each event terminates the current physical transfer or halt-wait,
     * requiring service from the application before any subsequent event can
     * be queued.
     */
    QueueHandle_t event_queue;

    /**
     * \brief A semaphore given by the USB interrupt service routine after an
     * endpoint is disabled due to application request (rather than due to
     * transfer complete).
     *
     * A task blocking on the semaphore must hold the mutex.
     */
    SemaphoreHandle_t disabled_sem;

    struct
    {
        /**
         * \brief Whether a zero-length packet has been received or is pending
         * to send.
         *
         * For OUT endpoints, this indicates whether or not a zero-length
         * packet was received in the last transfer. For IN endpoints, this
         * indicates whether or not a zero-length packet needs to be, and has
         * not yet been, sent at the end of the logical transfer.
         */
        bool zlp : 1;

        /**
         * \brief Whether or not the last transfer overflowed.
         */
        bool overflow : 1;
    }
    /**
     * \brief Flags regarding the endpoint.
     *
     * This field is only touched by the task performing a transfer or by the
     * ISR. It is not protected by the mutex.
     */
    flags;

    /**
     * \brief The interface that contains this endpoint, or UINT_MAX if none.
     *
     * This field is only touched by the stack internal task. It is not
     * protected by the mutex.
     */
    unsigned int interface;

    union
    {
        struct
        {
            /**
             * \brief A pointer to the start of the receive buffer.
             */
            uint8_t *buffer;

            /**
             * \brief A pointer to the next byte to write into.
             */
            uint8_t *wptr;
        } out;

        struct
        {
            /**
             * \brief A pointer to the start of the data to send.
             */
            const uint8_t *data;

            /**
             * \brief A pointer to the next byte to read and send.
             */
            const uint8_t *rptr;
        } in;
    }
    /**
     * \brief Information about the transfer that is specific to the direction.
     *
     * This field is only touched by the task performing a transfer or by the
     * ISR. It is not protected by the mutex.
     */
    transfer;

    /**
     * \brief The number of bytes left in the logical transfer.
     *
     * This field is only touched by the task performing a transfer or by the
     * ISR. It is not protected by the mutex.
     */
    size_t bytes_left;

    /**
     * \brief The callback to invoke when the endpoint completes an operation.
     *
     * This field is only touched by the task performing a transfer or by the
     * ISR. It is not protected by the mutex.
     */
    uep_async_cb_t async_cb;
} uep_ep_t;

/**
 * \endcond
 */



extern udev_state_change_t udev_state_change;
extern const udev_info_t *udev_info;
extern SemaphoreHandle_t udev_gonak_mutex;
extern unsigned int udev_gonak_disable_ep;
void udev_flush_rx_fifo(void);
void udev_flush_tx_fifo(unsigned int ep);
void udev_tx_copy_in(volatile void *dest, const void *source, size_t bytes);

extern uep0_setup_event_t uep0_setup_event;
extern void *uep0_out_wptr;
extern size_t uep0_out_length;
extern unsigned int uep0_xfrc;
extern const udev_config_info_t *uep0_current_configuration;
extern uint8_t *uep0_alternate_settings;
void *uep0_writable_setup_packet(void);
void uep0_run_transfer(void);
void uep0_handle_reset(void);

extern uep_ep_t uep_eps[UEP_MAX_ENDPOINT * 2U];
void uep_queue_event(unsigned int ep, uint8_t event);
void uep_queue_event_from_isr(unsigned int ep, uint8_t event, BaseType_t *yield);
bool uep_halt_with_cb(unsigned int ep, void (*cb)(void));
bool uep_clear_halt(unsigned int ep, bool (*cancb)(void), void (*cb)(void));
void uep_activate(const usb_endpoint_descriptor_t *descriptor, unsigned int interface);
void uep_deactivate(const usb_endpoint_descriptor_t *descriptor);

const usb_interface_descriptor_t *uutil_find_interface_descriptor(
    const usb_configuration_descriptor_t *config, unsigned int interface,
    unsigned int altsetting);
const udev_endpoint_info_t *uutil_find_endpoint_info(unsigned int ep);

#endif
