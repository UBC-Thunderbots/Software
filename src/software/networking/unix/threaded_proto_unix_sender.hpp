#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/networking/unix/threaded_unix_sender.h"

template <class SendProto>
class ThreadedProtoUnixSender : private ThreadedUnixSender
{
   public:
    /**
     * Create a ThreadedProtoUnixSender
     *
     * @param unix_path The unix socket path to send on
     */
    ThreadedProtoUnixSender(const std::string& unix_path) : ThreadedUnixSender(unix_path) {}

    /**
     * Sends a protobuf message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * @param message The protobuf message to send
     */
    void sendProto(const SendProto& message);

   private:
    std::string data_buffer;
};

template <class SendProtoT>
void ThreadedProtoUnixSender<SendProtoT>::sendProto(const SendProtoT& message)
{
    message.SerializeToString(&data_buffer);
    sendString(data_buffer);
}
