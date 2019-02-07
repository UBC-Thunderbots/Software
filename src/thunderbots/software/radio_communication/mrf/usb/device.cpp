#include "device.h"

#include "libusb.h"
#include "misc.h"

USB::Device::Device(const Device &copyref) : device(libusb_ref_device(copyref.device))
{
    check_fn("libusb_get_device_descriptor",
             libusb_get_device_descriptor(device, &device_descriptor), 0);
}

USB::Device::~Device()
{
    libusb_unref_device(device);
}

USB::Device &USB::Device::operator=(const Device &assgref)
{
    if (assgref.device != device)
    {
        libusb_unref_device(device);
        device = libusb_ref_device(assgref.device);
    }
    return *this;
}

USB::Device::Device(libusb_device *device) : device(libusb_ref_device(device))
{
    check_fn("libusb_get_device_descriptor",
             libusb_get_device_descriptor(device, &device_descriptor), 0);
}

std::string USB::Device::serial_number() const
{
    if (!device_descriptor.iSerialNumber)
    {
        return "";
    }

    std::string value;
    {
        DeviceHandle devh(*this);
        value = devh.string_descriptor(device_descriptor.iSerialNumber);
    }
    return value;
}

USB::DeviceList::DeviceList(Context &context)
{
    ssize_t ssz;
    check_fn("libusb_get_device_list",
             ssz = libusb_get_device_list(context.context, &devices), 0);
    size_ = static_cast<std::size_t>(ssz);
}

USB::DeviceList::~DeviceList()
{
    libusb_free_device_list(devices, 1);
}

USB::Device USB::DeviceList::operator[](const std::size_t i) const
{
    assert(i < size());
    return Device(devices[i]);
}
