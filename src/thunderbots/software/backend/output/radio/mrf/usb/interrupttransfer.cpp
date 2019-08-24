#include "interrupttransfer.h"

#include <cstring>

USB::InterruptInTransfer::InterruptInTransfer(DeviceHandle &dev, unsigned char endpoint,
                                              std::size_t len, bool exact_len,
                                              unsigned int timeout)
    : Transfer(dev)
{
    assert((endpoint & LIBUSB_ENDPOINT_ADDRESS_MASK) == endpoint);
    libusb_fill_interrupt_transfer(transfer, dev.handle, endpoint | LIBUSB_ENDPOINT_IN,
                                   new unsigned char[len], static_cast<int>(len),
                                   &usb_transfer_handle_completed_transfer_trampoline,
                                   transfer->user_data, timeout);
    if (exact_len)
    {
        transfer->flags |= LIBUSB_TRANSFER_SHORT_NOT_OK;
    }
}

USB::InterruptOutTransfer::InterruptOutTransfer(DeviceHandle &dev, unsigned char endpoint,
                                                const void *data, std::size_t len,
                                                std::size_t max_len, unsigned int timeout)
    : Transfer(dev)
{
    assert((endpoint & LIBUSB_ENDPOINT_ADDRESS_MASK) == endpoint);
    libusb_fill_interrupt_transfer(transfer, dev.handle, endpoint | LIBUSB_ENDPOINT_OUT,
                                   new unsigned char[len], static_cast<int>(len),
                                   &usb_transfer_handle_completed_transfer_trampoline,
                                   transfer->user_data, timeout);
    if (!max_len || len != max_len)
    {
        transfer->flags |= LIBUSB_TRANSFER_ADD_ZERO_PACKET;
    }
    std::memcpy(transfer->buffer, data, len);
}
