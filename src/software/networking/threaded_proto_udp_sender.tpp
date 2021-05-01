#pragma once

template <class SendProto>
ThreadedProtoUdpSender<SendProto>::ThreadedProtoUdpSender(const std::string& ip_address,
                                                          const unsigned short port,
                                                          bool multicast)
    : io_service(),
      udp_sender(io_service, ip_address, port, multicast),
      io_service_thread([this]() { io_service.run(); })
{
}

template <class SendProto>
ThreadedProtoUdpSender<SendProto>::~ThreadedProtoUdpSender()
{
    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise the
    // io_service will not stop and the thread will not join
    io_service.stop();

    // Join the io_service_thread so that we wait for it to exit before destructing the
    // thread object. If we do not wait for the thread to finish executing, it will call
    // `std::terminate` when we deallocate the thread object and kill our whole program
    io_service_thread.join();
}

template <class SendProto>
void ThreadedProtoUdpSender<SendProto>::sendProto(const SendProto& message)
{
    udp_sender.sendProto(message);
}
