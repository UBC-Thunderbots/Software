#pragma once

#include "transfer.h"

namespace USB
{
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

}  // namespace USB
