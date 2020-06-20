#pragma once

template <class SendProto>
ThreadedProtoMulticastSender<SendProto>::ThreadedProtoMulticastSender(
                                                      const std::string& ip_address,
                                                      const unsigned short port)
    : io_service(), multicast_sender(io_service, ip_address, port)
{
    // start the thread to run the io_service in the background
    io_service_thread = std::thread([this]() { io_service.run(); });
}

template <class SendProto>
ThreadedProtoMulticastSender<SendProto>::~ThreadedProtoMulticastSender()
{
    io_service_thread.join();
}

template <class SendProto>
void ThreadedProtoMulticastSender<SendProto>::sendProto(const SendProto& message)
{
    multicast_sender.sendProto(message);
}


