#ifndef UTIL_LIBUSB_H
#define UTIL_LIBUSB_H

#include <libusb.h>
#include <sigc++/connection.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "libusb_errors.h"
#include "libusb_device.h"

#include "async_operation.h"
#include "util/noncopyable.h"

namespace USB
{
    /**
     * These are for internal use only.
     */
    extern "C"
    {
        void usb_context_pollfd_add_trampoline(int fd, short events, void *user_data);
        void usb_context_pollfd_remove_trampoline(int fd, void *user_data);
        void usb_transfer_handle_completed_transfer_trampoline(libusb_transfer *transfer);
    }

    /**
     * A libusb context.
     */
    class Context final : public NonCopyable
    {
       public:
        /**
         * Initializes the library and creates a context.
         */
        explicit Context();

        void handle_usb_fds();

        /**
         * Deinitializes the library and destroys the context.
         *
         * This must be invoked after all associated DeviceList and DeviceHandle
         * objects have been destroyed.
         */
        ~Context();

       private:
        friend class DeviceList;
        friend class DeviceHandle;

        libusb_context *context;
    };

    /**
     * A libusb device handle which can be used to communicate with a device.
     */
    class DeviceHandle final : public NonCopyable
    {
       public:
        /**
         * Opens a handle to a device based on its vendor ID, product ID,
         * and, optionally, serial number.
         *
         * @param[in] context the context in which to open the handle
         *
         * @param[in] vendor_id the vendor ID of the device to open
         *
         * @param[in] product_id the product ID of the device to open
         *
         * @param[in] serial_number the serial number of the device to open, or null
         * to open a device with matching vendor and product ID but any serial
         * number
         */
        explicit DeviceHandle(Context &context, unsigned int vendor_id,
                              unsigned int product_id,
                              const char *serial_number = nullptr);

        /**
         * Opens a handle to a specific device.
         *
         * @param[in] device the device to open
         */
        explicit DeviceHandle(const Device &device);

        /**
         * Closes the device handle.
         *
         * If any outstanding asynchronous transfers are in progress, the destructor
         * blocks until they complete.
         */
        ~DeviceHandle();

        /**
         * Issues a bus reset to the device.
         *
         * The device handle should be closed after the reset is issued.
         */
        void reset();

        /**
         * Returns the device descriptor from the device.
         *
         * @return the device descriptor
         */
        const libusb_device_descriptor &device_descriptor() const;

        /**
         * Returns a configuration descriptor from the device.
         *
         * @param[in] index the zero-based index of the descriptor to return
         *
         * @return the configuration descriptor
         */
        const libusb_config_descriptor &configuration_descriptor(uint8_t index) const;

        /**
         * Returns a configuration descriptor from the device.
         *
         * @param[in] value the configuration value for the descriptor
         *
         * @return the configuration descriptor
         */
        const libusb_config_descriptor &configuration_descriptor_by_value(
            uint8_t value) const;

        /**
         * Reads a string descriptor from the device.
         *
         * @param[in] index the index of the string descriptor to read
         *
         * @return the descriptor
         */
        std::string string_descriptor(uint8_t index) const;

        /**
         * Returns the current configuration number.
         *
         * @return the configuration
         */
        int get_configuration() const;

        /**
         * Sets the device’s configuration.
         *
         * @param[in] config the configuration number to set
         */
        void set_configuration(int config);

        /**
         * Locks an interface so no other software can use it.
         *
         * @param[in] interface the interface number to claim
         */
        void claim_interface(int interface);

        /**
         * Releases a locked interface.
         *
         * @param[in] interface the interface to release
         */
        void release_interface(int interface);

        /**
         * Sets an interface into a particular alternate setting.
         *
         * @param[in] interface the interface to affect, which should be claimed
         *
         * @param[in] alternate_setting the alternate setting to switch the
         * interface into
         */
        void set_interface_alt_setting(int interface, int alternate_setting);

        /**
         * Attempts to clear halt status on an IN endpoint.
         *
         * @param[in] endpoint the endpoint number to operate on
         */
        void clear_halt_in(unsigned char endpoint);

        /**
         * Attempts to clear halt status on an OUT endpoint.
         *
         * @param[in] endpoint the endpoint number to operate on
         */
        void clear_halt_out(unsigned char endpoint);

        /**
         * Synchronously executes a control transfer with no data stage.
         *
         * @param[in] request_type the request type field of the setup transaction
         *
         * @param[in] request the request field of the setup transaction
         *
         * @param[in] value the value field of the setup transaction
         *
         * @param[in] index the index field of the setup transaction
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         */
        void control_no_data(uint8_t request_type, uint8_t request, uint16_t value,
                             uint16_t index, unsigned int timeout);

        /**
         * Synchronously executes a control transfer with an inbound data
         * stage.
         *
         * @param[in] request_type the request type field of the setup transaction
         *
         * @param[in] request the request field of the setup transaction
         *
         * @param[in] value the value field of the setup transaction
         *
         * @param[in] index the index field of the setup transaction
         *
         * @param[out] buffer a buffer in which to store the received data
         *
         * @param[in] len the maximum number of bytes to accept from the device
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         *
         * @return the number of bytes actually sent by the device
         */
        std::size_t control_in(uint8_t request_type, uint8_t request, uint16_t value,
                               uint16_t index, void *buffer, std::size_t len,
                               unsigned int timeout);

        /**
         * Synchronously executes a control transfer with an outbound data
         * stage.
         *
         * @param[in] request_type the request type field of the setup transaction
         *
         * @param[in] request the request field of the setup transaction
         *
         * @param[in] value the value field of the setup transaction
         *
         * @param[in] index the index field of the setup transaction
         *
         * @param[in] buffer the data to send in the data stage
         *
         * @param[in] len the number of bytes to send in the data stage
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         */
        void control_out(uint8_t request_type, uint8_t request, uint16_t value,
                         uint16_t index, const void *buffer, std::size_t len,
                         unsigned int timeout);

        /**
         * Synchronously executes an inbound transfer from an interrupt
         * endpoint.
         *
         * @param[in] endpoint the endpoint number on which to transfer
         *
         * @param[out] data a buffer in which to store the received data
         *
         * @param[in] length the maximum number of bytes to accept from the device
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         *
         * @return the number of bytes actually sent by the device
         */
        std::size_t interrupt_in(unsigned char endpoint, void *data, std::size_t length,
                                 unsigned int timeout);

        /**
         * Synchronously executes an outbound transfer to an interrupt
         * endpoint.
         *
         * @param[in] endpoint the endpoint number on which to transfer
         *
         * @param[in] data the data to send
         *
         * @param[in] length the number of bytes to send
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         */
        void interrupt_out(unsigned char endpoint, const void *data, std::size_t length,
                           unsigned int timeout);

        /**
         * Synchronously executes an inbound transfer from a bulk endpoint.
         *
         * @param[in] endpoint the endpoint number on which to transfer
         *
         * @param[out] data a buffer in which to store the received data
         *
         * @param[in] length the maximum number of bytes to accept from the device
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         *
         * @return the number of bytes actually sent by the device
         */
        std::size_t bulk_in(unsigned char endpoint, void *data, std::size_t length,
                            unsigned int timeout);

        /**
         * Mark the device as shutting down, so cancelled transfers will not
         * issue warnings.
         */
        void mark_shutting_down();

       private:
        friend class Transfer;
        friend class ControlNoDataTransfer;
        friend class ControlInTransfer;
        friend class InterruptOutTransfer;
        friend class InterruptInTransfer;
        friend class BulkOutTransfer;
        friend class BulkInTransfer;
        friend void usb_transfer_handle_completed_transfer_trampoline(
            libusb_transfer *transfer);

        libusb_context *context;
        libusb_device_handle *handle;
        libusb_device_descriptor device_descriptor_;
        std::vector<std::unique_ptr<libusb_config_descriptor,
                                    void (*)(libusb_config_descriptor *)>>
            config_descriptors;
        unsigned int submitted_transfer_count;
        bool shutting_down;

        void init_descriptors();
    };

    /**
     * An RAII-style way to set a configuration on a device.
     */
    class ConfigurationSetter final : public NonCopyable
    {
       public:
        /**
         * Sets a device’s configuration.
         *
         * @param[in] device the device to set
         *
         * @param[in] configuration the configuration number to set
         */
        explicit ConfigurationSetter(DeviceHandle &device, int configuration);

        /**
         * Resets the device to its original configuration.
         */
        ~ConfigurationSetter();

       private:
        DeviceHandle &device;
        int original_configuration;
    };

    /**
     * An RAII-style way to claim an interface on a device.
     */
    class InterfaceClaimer final : public NonCopyable
    {
       public:
        /**
         * Claims an interface on a device.
         *
         * @param[in] device the device whose interface should be claimed
         *
         * @param[in] interface the interface to claim
         */
        explicit InterfaceClaimer(DeviceHandle &device, int interface);

        /**
         * Releases the interface.
         */
        ~InterfaceClaimer();

       private:
        DeviceHandle &device;
        int interface;
    };

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

    /**
     * A libusb control transfer with no data.
     */
    class ControlNoDataTransfer final : public Transfer
    {
       public:
        /**
         * Constructs a new transfer.
         *
         * @param[in] dev the device to which to send the request
         *
         * @param[in] request_type the request type field of the setup transaction
         *
         * @param[in] request the request field of the setup transaction
         *
         * @param[in] value the value field of the setup transaction
         *
         * @param[in] index the index field of the setup transaction
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         */
        explicit ControlNoDataTransfer(DeviceHandle &dev, uint8_t request_type,
                                       uint8_t request, uint16_t value, uint16_t index,
                                       unsigned int timeout);
    };

    /**
     * A libusb control transfer with inbound data.
     */
    class ControlInTransfer final : public Transfer
    {
       public:
        /**
         * Constructs a new transfer.
         *
         * @param[in] dev the device to which to send the request
         *
         * @param[in] request_type the request type field of the setup transaction
         *
         * @param[in] request the request field of the setup transaction
         *
         * @param[in] value the value field of the setup transaction
         *
         * @param[in] index the index field of the setup transaction
         *
         * @param[in] len the maximum number of bytes to receive
         *
         * @param[in] exact_len \c true to consider the transfer a failure if it
         * transfers fewer than the requested number of bytes, or \c false to
         * consider such a result successful
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         */
        explicit ControlInTransfer(DeviceHandle &dev, uint8_t request_type,
                                   uint8_t request, uint16_t value, uint16_t index,
                                   std::size_t len, bool exact_len, unsigned int timeout);

        /**
         * Returns the received data.
         *
         * @return the received data
         */
        const uint8_t *data() const
        {
            assert(done_);
            return libusb_control_transfer_get_data(transfer);
        }

        /**
         * Returns the number of received bytes.
         *
         * @return the size of the data
         */
        std::size_t size() const
        {
            assert(done_);
            return static_cast<std::size_t>(transfer->actual_length);
        }
    };

    /**
     * A libusb inbound interrupt transfer.
     */
    class InterruptInTransfer final : public Transfer
    {
       public:
        /**
         * Constructs a new transfer.
         *
         * @param[in] dev the device from which to receive data
         *
         * @param[in] endpoint the endpoint number on which to transfer data
         *
         * @param[in] len the maximum number of bytes to receive
         *
         * @param[in] exact_len \c true to consider the transfer a failure if it
         * transfers fewer than the requested number of bytes, or \c false to
         * consider such a result successful
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         */
        explicit InterruptInTransfer(DeviceHandle &dev, unsigned char endpoint,
                                     std::size_t len, bool exact_len,
                                     unsigned int timeout);

        /**
         * Returns the received data.
         *
         * @return the received data
         */
        const uint8_t *data() const
        {
            assert(done_);
            return transfer->buffer;
        }

        /**
         * Returns the number of received bytes.
         *
         * @return the size of the data
         */
        std::size_t size() const
        {
            assert(done_);
            return static_cast<std::size_t>(transfer->actual_length);
        }
    };

    /**
     * A libusb outbound interrupt transfer.
     */
    class InterruptOutTransfer final : public Transfer
    {
       public:
        /**
         * Constructs a new transfer.
         *
         * @param[in] dev the device to which to send data
         *
         * @param[in] endpoint the endpoint number on which to send data
         *
         * @param[in] data the data to send, which is copied internally before the
         * constructor returns
         *
         * @param[in] len the number of bytes to send
         *
         * @param[in] max_len the maximum number of bytes the device is expecting to
         * receive, which is used to compute whether a zero-length packet is needed
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         */
        explicit InterruptOutTransfer(DeviceHandle &dev, unsigned char endpoint,
                                      const void *data, std::size_t len,
                                      std::size_t max_len, unsigned int timeout);
    };

    /**
     * A libusb inbound bulk transfer.
     */
    class BulkInTransfer final : public Transfer
    {
       public:
        /**
         * Constructs a new transfer.
         *
         * @param[in] dev the device from which to receive data
         *
         * @param[in] endpoint the endpoint number on which to transfer data
         *
         * @param[in] len the maximum number of bytes to receive
         *
         * @param[in] exact_len \c true to consider the transfer a failure if it
         * transfers fewer than the requested number of bytes, or \c false to
         * consider such a result successful
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         */
        explicit BulkInTransfer(DeviceHandle &dev, unsigned char endpoint,
                                std::size_t len, bool exact_len, unsigned int timeout);

        /**
         * Returns the received data.
         *
         * @return the received data
         */
        const uint8_t *data() const
        {
            assert(done_);
            return transfer->buffer;
        }

        /**
         * Returns the number of received bytes.
         *
         * @return the size of the data
         */
        std::size_t size() const
        {
            assert(done_);
            return static_cast<std::size_t>(transfer->actual_length);
        }
    };

    /**
     * A libusb outbound bulk transfer.
     */
    class BulkOutTransfer final : public Transfer
    {
       public:
        /**
         * Constructs a new transfer.
         *
         * @param[in] dev the device to which to send data
         *
         * @param[in] endpoint the endpoint number on which to send data
         *
         * @param[in] data the data to send, which is copied internally before the
         * constructor returns
         *
         * @param[in] len the number of bytes to send
         *
         * @param[in] max_len the maximum number of bytes the device is expecting to
         * receive, which is used to compute whether a zero-length packet is needed
         *
         * @param[in] timeout the maximum length of time to let the transfer run, in
         * milliseconds, or zero for no timeout
         */
        explicit BulkOutTransfer(DeviceHandle &dev, unsigned char endpoint,
                                 const void *data, std::size_t len, std::size_t max_len,
                                 unsigned int timeout);
    };
}  // namespace USB

#endif
