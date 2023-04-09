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
    // TODO: Add try catch
//    if (i < 10) {
//        std::cerr << "About to send " << std::to_string(i++) << "th message over UDP" << std::endl;
//    }
//    try {
        socket_.send_to(boost::asio::buffer(message, message.length()), receiver_endpoint);
//    } catch (std::exception& e) {
//        std::cerr << "UDP Sender exception caught. Skipping message to " << /*receiver_endpoint <<*/ "\n Error: " << e.what();
//    }

//    if (i++ < 10)
//    {
//        std::cerr << "About to send " << std::to_string(i) << "th message over UDP" << std::endl;
//    }
//
//    try
//    {
//        socket_.send_to(boost::asio::buffer(message, message.length()), receiver_endpoint);
//    }
//    catch (const std::exception& e)
//    {
//        std::cerr << "Exception caught in UDPSender: " << e.what() << std::endl;
//    }
//    catch (...)
//    {
//        std::cerr << "Exception caught in UDPSender" << std::endl;
//    }
}

UdpSender::~UdpSender()
{
    socket_.close();
}
