#include "udp_sender.h"
#include <iostream>

UdpSender::UdpSender(boost::asio::io_service& io_service,
                    const std::string& ip_address,
                    const unsigned short port, bool multicast)
    : socket_(io_service)
{
    boost::asio::ip::address addr = boost::asio::ip::make_address(ip_address);

    // The receiver endpoint identifies where this UdpSender will send data to
    receiver_endpoint = boost::asio::ip::udp::endpoint(addr, port);

    socket_.open(receiver_endpoint.protocol());

    if (multicast)
    {
        socket_.set_option(boost::asio::ip::multicast::join_group(addr));
    }
}

void UdpSender::sendString(const std::string& message)
{
    socket_.send_to(boost::asio::buffer(message, message.length()), receiver_endpoint);
}

UdpSender::~UdpSender()
{
    socket_.close();
}
