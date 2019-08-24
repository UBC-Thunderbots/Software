#pragma once

#include <libusb.h>

#include "devicehandle.h"
#include "util/async_operation.h"
#include "util/noncopyable.h"

namespace USB
{
    /* Forward declarations for C linkage */
    extern "C" void usb_transfer_handle_completed_transfer_trampoline(
        libusb_transfer *transfer);

    /**
     * A libusb transfer.
     */
    class Transfer : public AsyncOperation<void>
    {
       public:
        /**
         * Destroys the transfer.
         *
         * If the transfer is still executing, it will be cancelled.
         */
        virtual ~Transfer();

        /**
         * Sets whether stalled transfers will be retried.
         *
         * By default, stalled transfers are retried a number of times before
         * failing as transfers can very occasionally stall spuriously.
         *
         * @param[in] retry \c true to retry stalled transfers, or \c false to fail
         * on the first stall
         */
        void retry_on_stall(bool retry)
        {
            retry_on_stall_ = retry;
        }

        /**
         * Checks the outcome of the transfer.
         *
         * If the transfer failed, a corresponding exception will be thrown.
         */
        void result() const override;

        /**
         * Starts the transfer.
         */
        void submit();

       protected:
        friend void usb_transfer_handle_completed_transfer_trampoline(
            libusb_transfer *transfer);

        DeviceHandle &device;
        libusb_transfer *transfer;
        bool submitted_, done_;
        bool retry_on_stall_;
        unsigned int stall_retries_left;

        explicit Transfer(DeviceHandle &dev);
        void handle_completed_transfer();
    };

    class TransferMetadata final : public NonCopyable
    {
       public:
        static TransferMetadata *get(libusb_transfer *transfer)
        {
            return static_cast<TransferMetadata *>(transfer->user_data);
        }

        explicit TransferMetadata(USB::Transfer &transfer, USB::DeviceHandle &device)
            : transfer_(&transfer), device_(device)
        {
        }

        USB::Transfer *transfer() const
        {
            return transfer_;
        }

        USB::DeviceHandle &device() const
        {
            return device_;
        }

        void disown()
        {
            transfer_ = nullptr;
        }

       private:
        USB::Transfer *transfer_;
        USB::DeviceHandle &device_;
    };

    extern "C"
    {
        void usb_transfer_handle_completed_transfer_trampoline(libusb_transfer *transfer);
    }
}  // namespace USB
