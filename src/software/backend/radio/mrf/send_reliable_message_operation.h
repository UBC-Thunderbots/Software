#pragma once

#include "software/backend/radio/mrf/usb/libusb.h"
#include "software/backend/radio/mrf/util/async_operation.h"

class MRFDongle;

class SendReliableMessageOperation final : public AsyncOperation<void>
{
   public:
    /**
     * Thrown if a message cannot be delivered because the recipient
     * robot was not associated.
     */
    class NotAssociatedError;

    /**
     * Thrown if a message cannot be delivered because it was not
     * acknowledged by its recipient.
     */
    class NotAcknowledgedError;

    /**
     * Thrown if a message cannot be delivered because a clear channel
     * could not be found.
     */
    class ClearChannelError;

    /**
     * Queues a message for transmission.
     *
     * \param[in] dongle the dongle on which to send the message
     * \param[in] robot the robot index to which to send the message
     * \param[in] tries the number of times to try sending the message
     * \param[in] data the data to send (the data is copied into an internal
     * buffer)
     * \param[in] len the length of the data, including the header
     */
    explicit SendReliableMessageOperation(MRFDongle &dongle, unsigned int robot,
                                          unsigned int tries, const void *data,
                                          std::size_t len);

    /**
     * Checks for the success of the operation.
     *
     * If the operation failed, this function throws the relevant exception
     */
    void result() const override;

   private:
    MRFDongle &dongle;
    uint8_t message_id, delivery_status;
    std::unique_ptr<USB::BulkOutTransfer> transfer;

    /**
     * scoped_connection allows for the automatic disconnection of
     * this signal when this object gets destroyed
     *
     * replaces inheriting from sigc::trackable
     */
    boost::signals2::scoped_connection mdr_connection;

    void out_transfer_done(AsyncOperation<void> &);
    void message_delivery_report(uint8_t id, uint8_t code);
};

class SendReliableMessageOperation::NotAssociatedError final : public std::runtime_error
{
   public:
    /**
     * Constructs a NotAssociatedError.
     */
    explicit NotAssociatedError();
};

class SendReliableMessageOperation::NotAcknowledgedError final : public std::runtime_error
{
   public:
    /**
     * Constructs a NotAcknowledgedError.
     */
    explicit NotAcknowledgedError();
};

class SendReliableMessageOperation::ClearChannelError final : public std::runtime_error
{
   public:
    /**
     * Constructs a ClearChannelError.
     */
    explicit ClearChannelError();
};
