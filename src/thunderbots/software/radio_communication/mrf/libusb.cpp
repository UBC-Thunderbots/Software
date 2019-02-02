#include "libusb.h"

#include <poll.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <string>

#include "util/logger/init.h"

#define STALL_RETRIES 3

USB::Context::Context()
{
    check_fn("libusb_init", libusb_init(&context), 0);
}

USB::Context::~Context()
{
    libusb_exit(context);
    context = nullptr;
}

void USB::Context::handle_usb_fds()
{
    timeval tv = {0, 0};
    check_fn("libusb_handle_events_timeout", libusb_handle_events_timeout(context, &tv),
             0);
}

USB::ConfigurationSetter::ConfigurationSetter(DeviceHandle &device, int configuration)
    : device(device)
{
    original_configuration = device.get_configuration();
    device.set_configuration(configuration);
}

USB::ConfigurationSetter::~ConfigurationSetter()
{
    try
    {
        device.set_configuration(original_configuration ? original_configuration : -1);
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

USB::InterfaceClaimer::InterfaceClaimer(DeviceHandle &device, int interface)
    : device(device), interface(interface)
{
    device.claim_interface(interface);
}

USB::InterfaceClaimer::~InterfaceClaimer()
{
    try
    {
        device.release_interface(interface);
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

USB::BulkInTransfer::BulkInTransfer(DeviceHandle &dev, unsigned char endpoint,
                                    std::size_t len, bool exact_len, unsigned int timeout)
    : Transfer(dev)
{
    assert((endpoint & LIBUSB_ENDPOINT_ADDRESS_MASK) == endpoint);
    libusb_fill_bulk_transfer(transfer, dev.handle, endpoint | LIBUSB_ENDPOINT_IN,
                              new unsigned char[len], static_cast<int>(len),
                              &usb_transfer_handle_completed_transfer_trampoline,
                              transfer->user_data, timeout);
    if (exact_len)
    {
        transfer->flags |= LIBUSB_TRANSFER_SHORT_NOT_OK;
    }
}

USB::BulkOutTransfer::BulkOutTransfer(DeviceHandle &dev, unsigned char endpoint,
                                      const void *data, std::size_t len,
                                      std::size_t max_len, unsigned int timeout)
    : Transfer(dev)
{
    assert((endpoint & LIBUSB_ENDPOINT_ADDRESS_MASK) == endpoint);
    libusb_fill_bulk_transfer(transfer, dev.handle, endpoint | LIBUSB_ENDPOINT_OUT,
                              new unsigned char[len], static_cast<int>(len),
                              &usb_transfer_handle_completed_transfer_trampoline,
                              transfer->user_data, timeout);
    if (!max_len || len != max_len)
    {
        transfer->flags |= LIBUSB_TRANSFER_ADD_ZERO_PACKET;
    }
    std::memcpy(transfer->buffer, data, len);
}
