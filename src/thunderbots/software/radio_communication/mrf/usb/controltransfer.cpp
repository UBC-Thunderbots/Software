#include "controltransfer.h"

USB::ControlNoDataTransfer::ControlNoDataTransfer(DeviceHandle &dev, uint8_t request_type,
                                                  uint8_t request, uint16_t value,
                                                  uint16_t index, unsigned int timeout)
    : Transfer(dev)
{
    unsigned char *buffer = new unsigned char[8];
    libusb_fill_control_setup(buffer, request_type, request, value, index, 0);
    libusb_fill_control_transfer(transfer, dev.handle, buffer,
                                 &usb_transfer_handle_completed_transfer_trampoline,
                                 transfer->user_data, timeout);
}

USB::ControlInTransfer::ControlInTransfer(DeviceHandle &dev, uint8_t request_type,
                                          uint8_t request, uint16_t value, uint16_t index,
                                          std::size_t len, bool exact_len,
                                          unsigned int timeout)
    : Transfer(dev)
{
    unsigned char *buffer = new unsigned char[8 + len];
    libusb_fill_control_setup(buffer, request_type | LIBUSB_ENDPOINT_IN, request, value,
                              index, static_cast<uint16_t>(len));
    libusb_fill_control_transfer(transfer, dev.handle, buffer,
                                 &usb_transfer_handle_completed_transfer_trampoline,
                                 transfer->user_data, timeout);
    if (exact_len)
    {
        transfer->flags |= LIBUSB_TRANSFER_SHORT_NOT_OK;
    }
}
