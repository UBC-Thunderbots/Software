#pragma once

#include <string>

#include "software/networking/radio/threaded_radio_sender.h"

template <class SendProto>
class ThreadedProtoRadioSender : private ThreadedRadioSender
{
   public:
    /**
     * Creates a ThreadedProtoRadioSender that sends the SendProto over radio.
     */
    ThreadedProtoRadioSender() : ThreadedRadioSender() {}

    /**
     * Sends a protobuf message over radio.
     *
     * @param message The protobuf message to send
     */
    void sendProto(const SendProto& message);

   private:
    std::string data_buffer;
};

template <class SendProto>
void ThreadedProtoRadioSender<SendProto>::sendProto(const SendProto& message)
{
    message.SerializeToString(&data_buffer);
    sendString(data_buffer);
}
