#pragma once

#include "software/logger/logger.h"

template <class SendProto>
ProtoMulticastSender<SendProto>::ProtoMulticastSender(boost::asio::io_service& io_service,
                                                      const std::string ip_address,
                                                      const unsigned short port)
    : socket_(io_service)
{
    boost::asio::ip::address multicast_addr =
        boost::asio::ip::make_address(address);

    receiver_endpoint = udp::endpoint(multicast_addr, multicast_port);
    
    socket_.open(receiver_endpoint.protocol());
    socket_.set_option(boost::asio::ip::multicast::join_group(multicast_addr));

    try
    {
        socket_.bind(receiver_endpoint);
    }
    catch (const boost::exception& ex)
    {
        LOG(WARNING) << "There was an issue binding the socket_ to the endpoint when"
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
    socket_.send_to(boost::asio::buffer(data_buffer), receiver_endpoint);
}

void ProtoMulticastSender::~ProtoMulticastSender()
{
    socket_.close();
}
