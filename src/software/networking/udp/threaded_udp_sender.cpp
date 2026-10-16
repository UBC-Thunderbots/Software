#include "software/networking/udp/threaded_udp_sender.h"

ThreadedUdpSender::ThreadedUdpSender(std::shared_ptr<ThreadedIoContext> io_context,
                                     const std::string& ip_address,
                                     const unsigned short port,
                                     const std::string& interface, bool multicast)
    : io_context_(std::move(io_context)),
      udp_sender_(io_context_->getIoContext(), ip_address, port, interface, multicast)
{
}

std::string ThreadedUdpSender::getInterface() const
{
    return udp_sender_.getInterface();
}

std::string ThreadedUdpSender::getIpAddress() const
{
    return udp_sender_.getIpAddress();
}

void ThreadedUdpSender::sendString(const std::string& message, bool async)
{
    if (async)
    {
        udp_sender_.sendStringAsync(message);
    }
    else
    {
        udp_sender_.sendString(message);
    }
}
