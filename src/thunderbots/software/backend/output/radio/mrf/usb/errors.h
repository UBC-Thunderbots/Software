#pragma once

#include <stdexcept>

namespace USB
{
    /**
     * An error that occurs in a libusb library function.
     */
    class Error : public std::runtime_error
    {
       public:
        /**
         * Constructs a new error object with a message.
         *
         * @param[in] msg a detail message explaining the error
         */
        explicit Error(const std::string &msg);
    };

    /**
     * An error that occurs on a USB transfer.
     */
    class TransferError : public Error
    {
       public:
        /**
         * Constructs a new transfer error.
         *
         * @param[in] endpoint the endpoint number, with bit 7 used to indicate
         * direction
         *
         * @param[in] msg a detail message explaining the error (UTF-8 encoded)
         */
        explicit TransferError(unsigned int endpoint, const char *msg);
    };

    /**
     * An error that occurs when a USB transfer times out.
     */
    class TransferTimeoutError final : public TransferError
    {
       public:
        /**
         * Constructs a new timeout error object.
         *
         * @param[in] endpoint the endpoint number, with bit 7 used to indicate
         * direction
         */
        explicit TransferTimeoutError(unsigned int endpoint);
    };

    /**
     * An error that occurs when a USB stall occurs.
     */
    class TransferStallError final : public TransferError
    {
       public:
        /**
         * Constructs a new stall error object.
         *
         * @param[in] endpoint the endpoint number, with bit 7 used to indicate
         * direction
         */
        explicit TransferStallError(unsigned int endpoint);
    };

    /**
     * An error that occurs when a USB transfer is cancelled.
     */
    class TransferCancelledError final : public TransferError
    {
       public:
        /**
         * Constructs a new cancelled transfer error object.
         *
         * @param[in] endpoint the endpoint number, with bit 7 used to indicate
         * direction
         */
        explicit TransferCancelledError(unsigned int endpoint);
    };

}  // namespace USB
