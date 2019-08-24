#pragma once
#include "transfer.h"

namespace USB
{
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
}  // namespace USB
