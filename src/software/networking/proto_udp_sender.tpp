#pragma once

#include "software/logger/logger.h"

template <class SendProto>
ProtoUdpSender<SendProto>::ProtoUdpSender(boost::asio::io_service& io_service,
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

template <class SendProto>
void ProtoUdpSender<SendProto>::sendProto(const SendProto& message)
{
    message.SerializeToString(&data_buffer);
    socket_.send_to(boost::asio::buffer(data_buffer), receiver_endpoint);
}

template <class SendProto>
ProtoUdpSender<SendProto>::~ProtoUdpSender()
{
    socket_.close();
}
