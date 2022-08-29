#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include "base64.h"

template <class SendProto>
class ProtoUdpSender
{
   public:
    /**
     * Creates a ProtoUdpSender that sends the SendProto over the network to the
     * given address and port.
     *
     * The ProtoUdpSender will be assigned a random available port on creation
     * so it doesn't conflict with anything else on the system.
     *
     * @param io_service The io_service to use to service outgoing SendProto data
     * @param ip_address The ip address to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port to send SendProto data to
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ProtoUdpSender(boost::asio::io_service& io_service, const std::string& ip_address,
                   unsigned short port, bool multicast);

    virtual ~ProtoUdpSender();

    /**
     * Sends a protobuf message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * @param message The protobuf message to send
     */
    void sendProto(const SendProto& message);
    void sendProto(const std::string &message);

   private:
    // A UDP socket to send data over
    boost::asio::ip::udp::socket socket_;

    // The endpoint for the receiver
    boost::asio::ip::udp::endpoint receiver_endpoint;

    // Buffer to hold serialized protobuf data
    std::string data_buffer;
};

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
void ProtoUdpSender<SendProto>::sendProto(const std::string &buffer)
{
    socket_.send_to(boost::asio::buffer(buffer), receiver_endpoint);
}

template <class SendProto>
ProtoUdpSender<SendProto>::~ProtoUdpSender()
{
    socket_.close();
}

#include "software/networking/proto_udp_sender.hpp"
