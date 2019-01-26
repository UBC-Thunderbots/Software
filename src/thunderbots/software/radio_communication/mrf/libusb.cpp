#include "libusb.h"

#include <glibmm/convert.h>
#include <glibmm/main.h>
#include <glibmm/ustring.h>
#include "util/logger/init.h"
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

#define STALL_RETRIES 3

namespace
{
    long check_fn(const char *call, long err, unsigned int endpoint)
    {
        if (err >= 0)
        {
            return err;
        }

        const char *msg;
        const char *detail = nullptr;
        switch (err)
        {
            case LIBUSB_ERROR_IO:
                msg    = "Input/output error";
                detail = std::strerror(errno);
                break;

            case LIBUSB_ERROR_INVALID_PARAM:
                msg = "Invalid parameter";
                break;

            case LIBUSB_ERROR_ACCESS:
                msg = "Access denied (insufficient permissions)";
                break;

            case LIBUSB_ERROR_NO_DEVICE:
                msg = "No such device (it may have been disconnected)";
                break;

            case LIBUSB_ERROR_NOT_FOUND:
                msg = "Entity not found";
                break;

            case LIBUSB_ERROR_BUSY:
                msg = "Resource busy";
                break;

            case LIBUSB_ERROR_TIMEOUT:
                throw USB::TransferTimeoutError(endpoint);

            case LIBUSB_ERROR_OVERFLOW:
                msg = "Overflow";
                break;

            case LIBUSB_ERROR_PIPE:
                throw USB::TransferStallError(endpoint);

            case LIBUSB_ERROR_INTERRUPTED:
                msg = "System call interrupted (perhaps due to signal)";
                break;

            case LIBUSB_ERROR_NO_MEM:
                throw std::bad_alloc();

            case LIBUSB_ERROR_NOT_SUPPORTED:
                msg = "Operation not supported or unimplemented on this platform";
                break;

            case LIBUSB_ERROR_OTHER:
                msg = "Other error";
                break;

            default:
                throw std::runtime_error("Error fetching error message");
        }
        std::string s;
        s.reserve(std::strlen(call) + 2 + std::strlen(msg) +
                  (detail ? (3 + std::strlen(detail)) : 0));
        s.append(call);
        s.append(": ");
        s.append(msg);
        if (detail)
        {
            s.append(" (");
            s.append(detail);
            s.append(")");
        }
        throw USB::Error(s);
    }

    std::string make_transfer_error_message(unsigned int endpoint,
                                            const char *msg /* UTF8 */)
    {
        return Glib::locale_from_utf8(Glib::ustring::compose(
            u8"%1 on %2 endpoint %3", msg, (endpoint & 0x80) ? u8"IN" : u8"OUT",
            endpoint & 0x7F));
    }

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
}  // namespace

void USB::usb_transfer_handle_completed_transfer_trampoline(libusb_transfer *transfer)
{
    try
    {
        TransferMetadata *md = TransferMetadata::get(transfer);
        --md->device().submitted_transfer_count;
        if (md->transfer())
        {
            // printf("Before handle_completed_transfer\n");
            md->transfer()->handle_completed_transfer();
        }
        else
        {
            // This happens if the Transfer object has been destroyed but the
            // transfer was submitted at the time.
            // The disowned libusb_transfer needs to be allowed to finish
            // cancelling before being freed.
            printf("Transfer object destroyed\n");
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
        std::cerr << "Something went wrong with libusb: " << e.what() << std::endl;
        throw;
    }
}

USB::Error::Error(const std::string &msg) : std::runtime_error(msg) {}

USB::TransferError::TransferError(unsigned int endpoint, const char *msg)
    : Error(make_transfer_error_message(endpoint, msg))
{
}

USB::TransferTimeoutError::TransferTimeoutError(unsigned int endpoint)
    : TransferError(endpoint, u8"Transfer timed out")
{
}

USB::TransferStallError::TransferStallError(unsigned int endpoint)
    : TransferError(endpoint, u8"Transfer stalled")
{
}

USB::TransferCancelledError::TransferCancelledError(unsigned int endpoint)
    : TransferError(endpoint, u8"Transfer cancelled")
{
}

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
            return;
        }
    }

    throw std::runtime_error("No matching USB devices attached");
}

USB::DeviceHandle::~DeviceHandle()
{
    try
    {
        while (submitted_transfer_count)
        {
            check_fn("libusb_handle_events", libusb_handle_events(context), 0);
        }
    }
    catch (const std::exception &exp)
    {
        try
        {
            LOG(WARNING) << Glib::locale_to_utf8(exp.what()) << std::endl;
        }
        catch (...)
        {
        }
    }
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
        throw TransferError(1, u8"Device accepted wrong amount of data");
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
            LOG(WARNING) << Glib::locale_to_utf8(exp.what()) << std::endl;
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
            LOG(WARNING) << Glib::locale_to_utf8(exp.what()) << std::endl;
        }
        catch (...)
        {
        }
    }
}

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
                LOG(WARNING) << Glib::ustring::compose(
                    u8"Destroying in-progress transfer to USB %1 endpoint %2; "
                    u8"this is unreliable and may be a problem if not "
                    u8"happening during system shutdown!",
                    ((transfer->endpoint & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN
                         ? u8"in"
                         : u8"out"),
                    static_cast<unsigned int>(transfer->endpoint &
                                              LIBUSB_ENDPOINT_ADDRESS_MASK)) << std::endl;
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
            LOG(WARNING) << Glib::locale_to_utf8(exp.what()) << std::endl;
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
            throw TransferError(transfer->endpoint, u8"Transfer error");

        case LIBUSB_TRANSFER_TIMED_OUT:
            throw TransferTimeoutError(transfer->endpoint);

        case LIBUSB_TRANSFER_CANCELLED:
            throw TransferCancelledError(transfer->endpoint);

        case LIBUSB_TRANSFER_STALL:
            throw TransferStallError(transfer->endpoint);

        case LIBUSB_TRANSFER_NO_DEVICE:
            throw TransferError(transfer->endpoint, u8"Device was disconnected");

        case LIBUSB_TRANSFER_OVERFLOW:
            throw TransferError(transfer->endpoint,
                                u8"Device sent more data than requested");

        default:
            throw std::runtime_error("Error fetching error message");
    }
}

void USB::Transfer::submit()
{
    assert(!submitted_);
    check_fn("libusb_submit_transfer", libusb_submit_transfer(transfer),
             transfer->endpoint);
    submitted_         = true;
    done_              = false;
    stall_retries_left = retry_on_stall_ ? 30 : 0;
    ++device.submitted_transfer_count;
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
        LOG(INFO) << u8"Retrying stalled transfer." << std::endl;
        --stall_retries_left;
        check_fn("libusb_submit_transfer", libusb_submit_transfer(transfer),
                 transfer->endpoint);
        ++device.submitted_transfer_count;
        return;
    }
    done_      = true;
    submitted_ = false;
    signal_done.emit(*this);
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
