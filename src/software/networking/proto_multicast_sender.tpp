#pragma once

#include "software/logger/logger.h"

template <class SendProto>
ProtoMulticastSender<SendProto>::ProtoMulticastSender(
    boost::asio::io_service& io_service,
    const std::string ip_address,
    const unsigned short port)
    : socket(io_service)
{
    boost::asio::ip::address multicast_addr =
        boost::asio::ip::address_v6::from_string(multicast_address);

    receiver_endpoint = udp::endpoint(multicast_addr, multicast_port);
    socket.open(receiver_endpoint.protocol());

    // join the group
    socket.set_option(boost::asio::ip::multicast::join_group(multicast_addr));

    try
    {
        socket.bind(receiver_endpoint);
    }
    catch (const boost::exception& ex)
    {
        LOG(WARNING) << "There was an issue binding the socket to the endpoint when"
                        "trying to connect to the provided port"
                        "Please make sure no other program is using the port"
                     << std::endl;

        // Throw this exception up to top-level, as we have no valid
        // recovery action here
        throw;
    }
}

template <class SendProto>
void ProtoMulticastSender<SendProto>::sendData(const SendProto& message)
{
    message.SerializeToString(&data_buffer);
    socket.send_to(boost::asio::buffer(data_buffer), receiver_endpoint);
}

