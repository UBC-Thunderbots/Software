#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/networking/unix_sender.h"

template <class SendProto>
class ProtoUnixSender
{
   public:
    /**
     * Creates a ProtoUnixSender
     *
     * @param io_service The io service
     * @param unix_path The unix socket pack to send on
     */
    ProtoUnixSender(boost::asio::io_service& io_service, const std::string& unix_path);

    virtual ~ProtoUnixSender() = default;

    /**
     * Sends a protobuf message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * If any errors occur, a warning message will be logged.
     *
     * @param message The protobuf message to send
     */
    void sendProto(const SendProto& message);

   private:
    std::string data_buffer;
    UnixSender unix_sender_;
};

template <class SendProto>
ProtoUnixSender<SendProto>::ProtoUnixSender(boost::asio::io_service& io_service,
                                            const std::string& unix_path)
    : unix_sender_(io_service, unix_path)
{
}

template <class SendProto>
void ProtoUnixSender<SendProto>::sendProto(const SendProto& message)
{
    message.SerializeToString(&data_buffer);
    unix_sender_.sendString(data_buffer);
}
