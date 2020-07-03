#pragma once

#include "software/logger/logger.h"

template <class SendProto>
ProtoMulticastSender<SendProto>::ProtoMulticastSender(boost::asio::io_service& io_service,
                                                      const std::string& ip_address,
                                                      const unsigned short port)
    : socket_(io_service)
{
    boost::asio::ip::address multicast_addr = boost::asio::ip::make_address(ip_address);

    // The receiver endpoint identifies where this MulticastSender will send data to
    receiver_endpoint = boost::asio::ip::udp::endpoint(multicast_addr, port);

    socket_.open(receiver_endpoint.protocol());
    socket_.set_option(boost::asio::ip::multicast::join_group(multicast_addr));
}

template <class SendProto>
void ProtoMulticastSender<SendProto>::sendProto(const SendProto& message)
{
    message.SerializeToString(&data_buffer);
    socket_.send_to(boost::asio::buffer(data_buffer), receiver_endpoint);
}

template <class SendProto>
ProtoMulticastSender<SendProto>::~ProtoMulticastSender()
{
    socket_.close();
}
