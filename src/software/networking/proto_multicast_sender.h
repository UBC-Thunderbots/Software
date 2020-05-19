#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

template <class SendProto>
class ProtoMulticastSender
{
   public:
    /**
     * Creates a ProtoMulticastSender that sends the SendProto over the network on the given
     * address and port.
     *
     * @param io_service The io_service to use to service outgoing SendProto data
     * @param ip_address The ip address of the multicast group to send data on
     * @param port The port to send SendProto data on
     */
    ProtoMulticastSender(boost::asio::io_service& io_service, std::string ip_address,
                           unsigned short port);

    /**
     * Sends a protobuf message over the initialized multicast group and port
     * This function returns after the message has been sent.
     *
     * @param message The protobuf message to send over the joined multicast group
     */
    sendProto(const SendProto& message);

   private:
    // A UDP socket to send data over
    boost::asio::ip::udp::socket socket;
    // The endpoint for the receiver
    boost::asio::ip::udp::endpoint receiver_endpoint;
};

#include "software/networking/proto_multicast_sender.tpp"
