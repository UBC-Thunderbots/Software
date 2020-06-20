#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

template <class SendProto>
class ProtoMulticastSender
{
   public:
    /**
     * Creates a ProtoMulticastSender that sends the SendProto over the network on the
     * given address and port.
     *
     * @param io_service The io_service to use to service outgoing SendProto data
     * @param ip_address The ip address of the multicast group to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port to send SendProto data on
     */
    ProtoMulticastSender(boost::asio::io_service& io_service,
                         const std::string& ip_address, unsigned short port);

    virtual ~ProtoMulticastSender();

    /**
     * Sends a protobuf message over the initialized multicast group and port
     * This function returns after the message has been sent.
     *
     * @param message The protobuf message to send over the multicast group
     */
    void sendProto(const SendProto& message);

   private:
    // A UDP socket to send data over
    boost::asio::ip::udp::socket socket_;

    // The endpoint for the receiver
    boost::asio::ip::udp::endpoint receiver_endpoint;

    // Buffer to hold serialized protobuf data
    std::string data_buffer;
};

#include "software/networking/proto_multicast_sender.tpp"
