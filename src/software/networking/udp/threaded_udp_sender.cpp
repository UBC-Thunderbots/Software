#include "software/networking/udp/threaded_udp_sender.h"

ThreadedUdpSender::ThreadedUdpSender(const std::string& ip_address,
                                     const unsigned short port,
                                     const std::string& interface, bool multicast,
                                     std::optional<std::string>& error)
    : io_service(),
      udp_sender(io_service, ip_address, port, interface, multicast, error),
      io_service_thread([this]() { io_service.run(); })
{
}

ThreadedUdpSender::~ThreadedUdpSender()
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

std::string ThreadedUdpSender::getInterface() const
{
    return udp_sender.getInterface();
}

std::string ThreadedUdpSender::getIpAddress() const
{
    return udp_sender.getIpAddress();
}

void ThreadedUdpSender::sendString(const std::string& message, bool async)
{
    if (async)
    {
        udp_sender.sendStringAsync(message);
    }
    else
    {
        udp_sender.sendString(message);
    }
}
