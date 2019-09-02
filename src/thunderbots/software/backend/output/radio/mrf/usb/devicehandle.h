#pragma once

#include <libusb.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "device.h"
#include "util/noncopyable.h"

namespace USB
{
    /* Forward declaration of the libusb context */
    class Context;
    extern "C" void usb_transfer_handle_completed_transfer_trampoline(
        libusb_transfer *transfer);

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

}  // namespace USB
