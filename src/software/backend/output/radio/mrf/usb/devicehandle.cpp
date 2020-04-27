#include "software/backend/output/radio/mrf/usb/devicehandle.h"

#include "software/backend/output/radio/mrf/usb/libusb.h"
#include "software/backend/output/radio/mrf/usb/misc.h"
#include "software/logger/logger.h"

namespace
{
    bool matches_vid_pid_serial(const USB::Device &device, unsigned int vendor_id,
                                unsigned int product_id, const char *serial_number)
    {
        if (device.vendor_id() != vendor_id || device.product_id() != product_id)
        {
            return false;
        }

        if (!serial_number)
        {
            return true;
        }

        return device.serial_number() == serial_number;
    }

}  // namespace

USB::DeviceHandle::DeviceHandle(const Device &device)
    : context(device.context), submitted_transfer_count(0), shutting_down(false)
{
    check_fn("libusb_open", libusb_open(device.device, &handle), 0);
    init_descriptors();
}

USB::DeviceHandle::DeviceHandle(Context &context, unsigned int vendor_id,
                                unsigned int product_id, const char *serial_number)
    : context(context.context), submitted_transfer_count(0), shutting_down(false)
{
    DeviceList lst(context);
    for (std::size_t i = 0; i < lst.size(); ++i)
    {
        const Device &device = lst[i];
        if (matches_vid_pid_serial(device, vendor_id, product_id, serial_number))
        {
            for (std::size_t j = i + 1; j < lst.size(); ++j)
            {
                const Device &device = lst[j];
                if (matches_vid_pid_serial(device, vendor_id, product_id, serial_number))
                {
                    throw std::runtime_error("Multiple matching USB devices attached");
                }
            }

            check_fn("libusb_open", libusb_open(device.device, &handle), 0);
            init_descriptors();
            context.open_devices.push_back(this);
            return;
        }
    }

    throw std::runtime_error("No matching USB devices attached");
}

USB::DeviceHandle::~DeviceHandle()
{
    libusb_close(handle);
}

void USB::DeviceHandle::reset()
{
    check_fn("libusb_reset_device", libusb_reset_device(handle), 0);
}

const libusb_device_descriptor &USB::DeviceHandle::device_descriptor() const
{
    return device_descriptor_;
}

const libusb_config_descriptor &USB::DeviceHandle::configuration_descriptor(
    uint8_t index) const
{
    return *config_descriptors[index];
}

const libusb_config_descriptor &USB::DeviceHandle::configuration_descriptor_by_value(
    uint8_t value) const
{
    for (const auto &i : config_descriptors)
    {
        if (i->bConfigurationValue == value)
        {
            return *i;
        }
    }
    throw std::runtime_error("No such configuration.");
}

std::string USB::DeviceHandle::string_descriptor(uint8_t index) const
{
    std::vector<unsigned char> buf(8);
    int rc;
    do
    {
        buf.resize(buf.size() * 2);
        rc = libusb_get_string_descriptor_ascii(handle, index, &buf[0],
                                                static_cast<int>(buf.size()));
    } while (rc >= static_cast<int>(buf.size()) - 1);
    check_fn("libusb_get_string_descriptor_ascii", rc, 0);
    return std::string(buf.begin(), buf.begin() + rc);
}

int USB::DeviceHandle::get_configuration() const
{
    int conf;
    check_fn("libusb_get_configuration", libusb_get_configuration(handle, &conf), 0);
    return conf;
}

void USB::DeviceHandle::set_configuration(int config)
{
    check_fn("libusb_set_configuration", libusb_set_configuration(handle, config), 0);
}

void USB::DeviceHandle::claim_interface(int interface)
{
    check_fn("libusb_claim_interface", libusb_claim_interface(handle, interface), 0);
}

void USB::DeviceHandle::release_interface(int interface)
{
    check_fn("libusb_release_interface", libusb_release_interface(handle, interface), 0);
}

void USB::DeviceHandle::set_interface_alt_setting(int interface, int alternate_setting)
{
    check_fn("libusb_set_interface_alt_setting",
             libusb_set_interface_alt_setting(handle, interface, alternate_setting), 0);
}

void USB::DeviceHandle::clear_halt_in(unsigned char endpoint)
{
    check_fn("libusb_clear_halt",
             libusb_clear_halt(handle, endpoint | LIBUSB_ENDPOINT_IN),
             endpoint | LIBUSB_ENDPOINT_IN);
}

void USB::DeviceHandle::clear_halt_out(unsigned char endpoint)
{
    check_fn("libusb_clear_halt",
             libusb_clear_halt(handle, endpoint | LIBUSB_ENDPOINT_OUT),
             endpoint | LIBUSB_ENDPOINT_OUT);
}

void USB::DeviceHandle::control_no_data(uint8_t request_type, uint8_t request,
                                        uint16_t value, uint16_t index,
                                        unsigned int timeout)
{
    assert((request_type & LIBUSB_ENDPOINT_DIR_MASK) == 0);
    check_fn("libusb_control_transfer",
             libusb_control_transfer(handle, request_type | LIBUSB_ENDPOINT_OUT, request,
                                     value, index, nullptr, 0, timeout),
             0);
}

std::size_t USB::DeviceHandle::control_in(uint8_t request_type, uint8_t request,
                                          uint16_t value, uint16_t index, void *buffer,
                                          std::size_t len, unsigned int timeout)
{
    assert((request_type & LIBUSB_ENDPOINT_DIR_MASK) == 0);
    assert(len < 65536);
    return static_cast<std::size_t>(check_fn(
        "libusb_control_transfer",
        libusb_control_transfer(handle, request_type | LIBUSB_ENDPOINT_IN, request, value,
                                index, static_cast<unsigned char *>(buffer),
                                static_cast<uint16_t>(len), timeout),
        0));
}

void USB::DeviceHandle::control_out(uint8_t request_type, uint8_t request, uint16_t value,
                                    uint16_t index, const void *buffer, std::size_t len,
                                    unsigned int timeout)
{
    assert((request_type & LIBUSB_ENDPOINT_DIR_MASK) == 0);
    assert(len < 65536);
    check_fn("libusb_control_transfer",
             libusb_control_transfer(
                 handle, request_type | LIBUSB_ENDPOINT_OUT, request, value, index,
                 static_cast<unsigned char *>(const_cast<void *>(buffer)),
                 static_cast<uint16_t>(len), timeout),
             0);
}

std::size_t USB::DeviceHandle::interrupt_in(unsigned char endpoint, void *data,
                                            std::size_t length, unsigned int timeout)
{
    assert((endpoint & LIBUSB_ENDPOINT_ADDRESS_MASK) == endpoint);
    assert(length < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    int transferred = -1;
    check_fn("libusb_interrupt_transfer",
             libusb_interrupt_transfer(handle, endpoint | LIBUSB_ENDPOINT_IN,
                                       static_cast<unsigned char *>(data),
                                       static_cast<int>(length), &transferred, timeout),
             endpoint | LIBUSB_ENDPOINT_IN);
    assert(transferred >= 0);
    return static_cast<std::size_t>(transferred);
}

void USB::DeviceHandle::interrupt_out(unsigned char endpoint, const void *data,
                                      std::size_t length, unsigned int timeout)
{
    assert((endpoint & LIBUSB_ENDPOINT_ADDRESS_MASK) == endpoint);
    assert(length < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    int transferred;
    check_fn("libusb_interrupt_transfer",
             libusb_interrupt_transfer(
                 handle, endpoint | LIBUSB_ENDPOINT_OUT,
                 const_cast<unsigned char *>(static_cast<const unsigned char *>(data)),
                 static_cast<int>(length), &transferred, timeout),
             endpoint | LIBUSB_ENDPOINT_OUT);
    if (transferred != static_cast<int>(length))
    {
        throw TransferError(1, "Device accepted wrong amount of data");
    }
}

std::size_t USB::DeviceHandle::bulk_in(unsigned char endpoint, void *data,
                                       std::size_t length, unsigned int timeout)
{
    assert((endpoint & LIBUSB_ENDPOINT_ADDRESS_MASK) == endpoint);
    assert(length < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    int transferred = -1;
    check_fn("libusb_bulk_transfer",
             libusb_bulk_transfer(handle, endpoint | LIBUSB_ENDPOINT_IN,
                                  static_cast<unsigned char *>(data),
                                  static_cast<uint16_t>(length), &transferred, timeout),
             endpoint | LIBUSB_ENDPOINT_IN);
    assert(transferred >= 0);
    return static_cast<std::size_t>(transferred);
}

void USB::DeviceHandle::mark_shutting_down()
{
    shutting_down = true;
}

void USB::DeviceHandle::init_descriptors()
{
    check_fn("libusb_get_device_descriptor",
             libusb_get_device_descriptor(libusb_get_device(handle), &device_descriptor_),
             0);
    for (uint8_t i = 0; i < device_descriptor_.bNumConfigurations; ++i)
    {
        libusb_config_descriptor *desc = nullptr;
        check_fn("libusb_get_configuration_descriptor",
                 libusb_get_config_descriptor(libusb_get_device(handle), i, &desc), 0);
        std::unique_ptr<libusb_config_descriptor, void (*)(libusb_config_descriptor *)>
            ptr(desc, &libusb_free_config_descriptor);
        config_descriptors.push_back(std::move(ptr));
    }
}
