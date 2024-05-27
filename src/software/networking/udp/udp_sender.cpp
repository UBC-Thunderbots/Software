#include "udp_sender.h"

#include "software/networking/udp/network_utils.h"

#include <iostream>

UdpSender::UdpSender(boost::asio::io_service& io_service, const std::string& ip_address,
                     const unsigned short port, const std::string& interface, bool multicast)
    : socket_(io_service)
{
    boost::asio::ip::address boost_ip = boost::asio::ip::make_address(ip_address);
    if (isIpv6(ip_address))
    {
        boost_ip = boost::asio::ip::make_address(ip_address + "%" + interface);
    }

    // The receiver endpoint identifies where this UdpSender will send data to
    receiver_endpoint = boost::asio::ip::udp::endpoint(boost_ip, port);

    socket_.open(receiver_endpoint.protocol());

    if (multicast)
    {
        setupMulticast(boost_ip, interface);
    }
}

void UdpSender::sendString(const std::string& message)
{
    socket_.send_to(boost::asio::buffer(message, message.length()), receiver_endpoint);
}

void UdpSender::setupMulticast(const boost::asio::ip::address& ip_address, const std::string& interface)
{
    if (ip_address.is_v4())
    {
        std::string interface_ip;
        if (!getLocalIp(interface, interface_ip))
        {
            std::cerr << "UdpSender: Could not get the local IP address for the interface "
                          "specified. (interface = "
                       << interface << ")" << std::endl;
        }
        
        socket_.set_option(boost::asio::ip::multicast::join_group(ip_address.to_v4(),
                    boost::asio::ip::address::from_string(interface_ip).to_v4()));
        return;
    }

    socket_.set_option(boost::asio::ip::multicast::join_group(ip_address));
}


UdpSender::~UdpSender()
{
    socket_.close();
}
