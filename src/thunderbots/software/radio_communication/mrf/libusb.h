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

#include "libusb_misc.h"
#include "libusb_errors.h"
#include "libusb_device.h"
#include "libusb_transfer.h"
#include "libusb_devicehandle.h"

#include "async_operation.h"
#include "util/noncopyable.h"

namespace USB
{
    /**
     * A libusb context. This is the entry point to the libusb wrapper.
     */
    class Context final : public NonCopyable
    {
       public:
        /**
         * Initializes the library and creates a context.
         */
        explicit Context();

        /**
         * This function must be called in order to update pending libusb events.
         */
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
