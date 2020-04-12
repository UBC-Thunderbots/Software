#include "software/backend/output/radio/mrf/usb/transfer.h"

#include <iostream>

#include "software/backend/output/radio/mrf/usb/errors.h"
#include "software/backend/output/radio/mrf/usb/misc.h"
#include "software/logger/logger.h"

USB::Transfer::~Transfer()
{
    try
    {
        if (submitted_)
        {
            // The transfer is submitted.
            // Initiate transfer cancellation.
            // Instead of waiting for cancellation to complete, "disown" the
            // transfer object.
            // It will be freed by the trampoline.
            //
            // There is a libusb bug where cancelling transfers sometimes makes
            // the endpoint completely unresponsive in future.
            // It’s not a USB bug; libusb actually never submits future
            // transfers for that endpoint!
            // We should show a warning if a transfer is cancelled.
            // However, cancelling transfers at shutdown would result in
            // unnecessary spam, so squelch those.
            if (!device.shutting_down)
            {
                LOG(WARNING) << "Destroying in-progress transfer to USB "
                             << ((transfer->endpoint & LIBUSB_ENDPOINT_DIR_MASK) ==
                                         LIBUSB_ENDPOINT_IN
                                     ? "in"
                                     : "out")
                             << " endpoint "
                             << static_cast<unsigned int>(transfer->endpoint &
                                                          LIBUSB_ENDPOINT_ADDRESS_MASK)
                             << "; this is unreliable and may be a problem if not "
                                "happening during system shutdown!"
                             << std::endl;
            }
            libusb_cancel_transfer(transfer);
            TransferMetadata::get(transfer)->disown();
        }
        else
        {
            // The transfer is not submitted and therefore can safely be freed.
            delete TransferMetadata::get(transfer);
            delete[] transfer->buffer;
            libusb_free_transfer(transfer);
        }
    }
    catch (const std::exception &exp)
    {
        try
        {
            LOG(WARNING) << exp.what() << std::endl;
        }
        catch (...)
        {
        }
    }
}

void USB::Transfer::result() const
{
    assert(done_);
    switch (transfer->status)
    {
        case LIBUSB_TRANSFER_COMPLETED:
            return;

        case LIBUSB_TRANSFER_ERROR:
            throw TransferError(transfer->endpoint, "Transfer error");

        case LIBUSB_TRANSFER_TIMED_OUT:
            throw TransferTimeoutError(transfer->endpoint);

        case LIBUSB_TRANSFER_CANCELLED:
            throw TransferCancelledError(transfer->endpoint);

        case LIBUSB_TRANSFER_STALL:
            throw TransferStallError(transfer->endpoint);

        case LIBUSB_TRANSFER_NO_DEVICE:
            throw TransferError(transfer->endpoint, "Device was disconnected");

        case LIBUSB_TRANSFER_OVERFLOW:
            throw TransferError(transfer->endpoint,
                                "Device sent more data than requested");

        default:
            throw std::runtime_error("Error fetching error message");
    }
}

void USB::Transfer::submit()
{
    assert(!submitted_);
    submitted_         = true;
    done_              = false;
    stall_retries_left = retry_on_stall_ ? 30 : 0;
    ++device.submitted_transfer_count;

    check_fn("libusb_submit_transfer", libusb_submit_transfer(transfer),
             transfer->endpoint);
}

USB::Transfer::Transfer(DeviceHandle &dev)
    : device(dev),
      transfer(libusb_alloc_transfer(0)),
      submitted_(false),
      done_(false),
      retry_on_stall_(true),
      stall_retries_left(0)
{
    if (!transfer)
    {
        throw std::bad_alloc();
    }
    try
    {
        transfer->user_data = new TransferMetadata(*this, dev);
    }
    catch (...)
    {
        libusb_free_transfer(transfer);
        throw;
    }
    transfer->flags = 0;
}

void USB::Transfer::handle_completed_transfer()
{
    if (transfer->status == LIBUSB_TRANSFER_STALL && stall_retries_left)
    {
        LOG(INFO) << "Retrying stalled transfer." << std::endl;
        --stall_retries_left;
        check_fn("libusb_submit_transfer", libusb_submit_transfer(transfer),
                 transfer->endpoint);
        ++device.submitted_transfer_count;
        return;
    }
    done_      = true;
    submitted_ = false;
    signal_done(*this);
}

void USB::usb_transfer_handle_completed_transfer_trampoline(libusb_transfer *transfer)
{
    try
    {
        TransferMetadata *md = TransferMetadata::get(transfer);
        --md->device().submitted_transfer_count;
        if (md->transfer())
        {
            md->transfer()->handle_completed_transfer();
        }
        else
        {
            // This happens if the Transfer object has been destroyed but the
            // transfer was submitted at the time.
            // The disowned libusb_transfer needs to be allowed to finish
            // cancelling before being freed.
            delete md;
            delete[] transfer->buffer;
            libusb_free_transfer(transfer);
        }
    }
    catch (std::exception &e)
    {
        // libusb is C code, so exception cannot safely propagate through it
        // doing a normal stack unwind.
        // Save the exception in the main loop and do a normal return, which
        // will let libusb unwind itself properly before the exception continues
        // unwinding.
        LOG(FATAL) << "Something went wrong with libusb: " << e.what() << std::endl;
        throw;
    }
}
