#pragma once

#include "software/logger/logger.h"

template <class SendProto>
ProtoMulticastSender<SendProto>::ProtoMulticastSender(boost::asio::io_service& io_service,
                                                      const std::string& ip_address,
                                                      const unsigned short port)
    : socket_(io_service)
{
    boost::asio::ip::address multicast_addr = boost::asio::ip::make_address(ip_address);

    // Create the sender_enpoint. This is the endpoint that identifies the sender, and
    // must be different than the receiver (otherwise multiple sockets will try bind to
    // the same port and cause errors). Port 0 is a special value that tells the operating
    // system to dynamically assign an available port.
    // https://www.lifewire.com/port-0-in-tcp-and-udp-818145
    boost::asio::ip::udp::endpoint sender_endpoint =
        boost::asio::ip::udp::endpoint(multicast_addr, 0);
    ;
    // The receiver endpoint identifies where this MulticastSender will send data to
    receiver_endpoint = boost::asio::ip::udp::endpoint(multicast_addr, port);

    socket_.open(receiver_endpoint.protocol());
    socket_.set_option(boost::asio::ip::multicast::join_group(multicast_addr));

    try
    {
        socket_.bind(sender_endpoint);
    }
    catch (const boost::exception& ex)
    {
        LOG(FATAL) << "ProtoMulticastSender: There was an issue binding the socket to "
                      "the sender_endpoint when trying to connect to the provided port "
                      "(ip = "
                   << ip_address << ", port = " << port
                   << "). "
                      "Please make sure no other program is using the port."
                   << std::endl;
    }
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
